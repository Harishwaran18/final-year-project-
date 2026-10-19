// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>

#include "sil_sensor_fusion/ekf_tracker.hpp"

namespace sil_sensor_fusion
{
namespace
{

/// Default configuration used by most tests.
EkfConfig DefaultConfig()
{
  return EkfConfig{};
}

TEST(EkfTrackerTest, IsUninitialisedOnConstruction)
{
  EkfTracker tracker;
  EXPECT_FALSE(tracker.initialized());
  EXPECT_FALSE(tracker.Predict(0.1));
  EXPECT_FALSE(tracker.Update(MeasurementVector(10.0, 0.0)).accepted);
}

TEST(EkfTrackerTest, InitializeSeedsCartesianStateFromPolarMeasurement)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(10.0, 0.0), 1.0);

  ASSERT_TRUE(tracker.initialized());
  EXPECT_DOUBLE_EQ(1.0, tracker.timestamp());
  EXPECT_NEAR(10.0, tracker.state()(0), 1e-9);
  EXPECT_NEAR(0.0, tracker.state()(1), 1e-9);
  EXPECT_DOUBLE_EQ(0.0, tracker.state()(2));
  EXPECT_DOUBLE_EQ(0.0, tracker.state()(3));
}

TEST(EkfTrackerTest, InitializeHandlesNonZeroBearing)
{
  EkfTracker tracker;
  const double bearing = 0.3;
  tracker.Initialize(MeasurementVector(20.0, bearing), 0.0);

  EXPECT_NEAR(20.0 * std::cos(bearing), tracker.state()(0), 1e-9);
  EXPECT_NEAR(20.0 * std::sin(bearing), tracker.state()(1), 1e-9);
  EXPECT_NEAR(20.0, tracker.Range(), 1e-9);
}

TEST(EkfTrackerTest, PredictHoldsPositionWithoutMeasurement)
{
  // The constant-velocity model has no acceleration input, so with a zero
  // initial velocity the predicted position must not move on its own.
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(10.0, 0.0), 0.0);

  ASSERT_TRUE(tracker.Predict(0.5));
  EXPECT_NEAR(10.0, tracker.state()(0), 1e-9);
  EXPECT_NEAR(0.0, tracker.state()(1), 1e-9);
  EXPECT_DOUBLE_EQ(0.5, tracker.timestamp());
}

TEST(EkfTrackerTest, PredictRejectsNonMonotonicTimestamps)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(10.0, 0.0), 1.0);

  EXPECT_FALSE(tracker.Predict(1.0));
  EXPECT_FALSE(tracker.Predict(0.5));
  EXPECT_TRUE(tracker.Predict(1.1));
}

TEST(EkfTrackerTest, PredictRejectsStaleGapBeyondMaxDt)
{
  EkfConfig config = DefaultConfig();
  config.max_dt = 0.2;
  EkfTracker tracker(config);
  tracker.Initialize(MeasurementVector(10.0, 0.0), 0.0);

  EXPECT_FALSE(tracker.Predict(0.5));
}

TEST(EkfTrackerTest, PredictGrowsCovariance)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(10.0, 0.0), 0.0);
  const double before = tracker.PositionUncertainty();

  ASSERT_TRUE(tracker.Predict(0.1));
  EXPECT_GT(tracker.PositionUncertainty(), before);
}

TEST(EkfTrackerTest, MeasurementJacobianMatchesNumericalJacobian)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(12.0, 0.25), 0.0);

  const MeasurementMatrix analytic = tracker.MeasurementJacobian();
  const MeasurementMatrix numerical = tracker.NumericalMeasurementJacobian();

  for (int row = 0; row < analytic.rows(); ++row) {
    for (int col = 0; col < analytic.cols(); ++col) {
      EXPECT_NEAR(analytic(row, col), numerical(row, col), 1e-6)
        << "mismatch at (" << row << ", " << col << ")";
    }
  }
}

TEST(EkfTrackerTest, MeasurementJacobianHasExpectedStructure)
{
  // At (x=10, y=0) the range derivative is [1, 0] and the bearing derivative
  // is [0, 0.1].
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(10.0, 0.0), 0.0);
  const MeasurementMatrix h = tracker.MeasurementJacobian();

  EXPECT_NEAR(1.0, h(0, 0), 1e-12);
  EXPECT_NEAR(0.0, h(0, 1), 1e-12);
  EXPECT_NEAR(0.0, h(1, 0), 1e-12);
  EXPECT_NEAR(0.1, h(1, 1), 1e-12);
}

TEST(EkfTrackerTest, UpdateAcceptsConsistentMeasurementAndShrinksCovariance)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(20.0, 0.0), 0.0);
  const double before = tracker.PositionUncertainty();

  const EkfTracker::UpdateResult result = tracker.Update(MeasurementVector(20.05, 0.0));

  EXPECT_TRUE(result.accepted);
  EXPECT_LT(result.mahalanobis, tracker.config().gate_threshold);
  EXPECT_LT(tracker.PositionUncertainty(), before);
}

TEST(EkfTrackerTest, UpdateRejectsOutlierBeyondGate)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(20.0, 0.0), 0.0);
  const StateVector state_before = tracker.state();
  const CovarianceMatrix covariance_before = tracker.covariance();

  // 40 m of range error against a 0.25 m sigma is far outside the gate.
  const EkfTracker::UpdateResult result = tracker.Update(MeasurementVector(60.0, 0.0));

  EXPECT_FALSE(result.accepted);
  EXPECT_GT(result.mahalanobis, tracker.config().gate_threshold);
  // The rejected measurement must leave the state untouched.
  EXPECT_EQ(state_before, tracker.state());
  EXPECT_EQ(covariance_before, tracker.covariance());
}

TEST(EkfTrackerTest, UpdateRejectsNonFiniteMeasurement)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(20.0, 0.0), 0.0);

  EXPECT_FALSE(tracker.Update(MeasurementVector(
    std::numeric_limits<double>::quiet_NaN(), 0.0)).accepted);
  EXPECT_FALSE(tracker.Update(MeasurementVector(
    20.0, std::numeric_limits<double>::infinity())).accepted);
}

TEST(EkfTrackerTest, UpdateHonoursNoiseOverride)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(20.0, 0.0), 0.0);

  MeasurementCovariance tight = MeasurementCovariance::Zero();
  tight(0, 0) = 0.01 * 0.01;
  tight(1, 1) = 0.001 * 0.001;

  MeasurementCovariance loose = MeasurementCovariance::Zero();
  loose(0, 0) = 25.0;
  loose(1, 1) = 0.25;

  const double tight_gate = tracker.MahalanobisDistance(
    MeasurementVector(30.0, 0.0), &tight);
  const double loose_gate = tracker.MahalanobisDistance(
    MeasurementVector(30.0, 0.0), &loose);
  EXPECT_GT(tight_gate, loose_gate);

  // The loose covariance lets the same measurement through.
  EXPECT_TRUE(tracker.Update(MeasurementVector(30.0, 0.0), &loose).accepted);
}

TEST(EkfTrackerTest, BearingResidualWrapsAcrossPiSeam)
{
  // A target at bearing +3.0 rad observed at -3.0 rad is only ~0.28 rad away,
  // not 6 rad. Without wrapping the gate would reject it.
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(20.0, 3.0), 0.0);

  const double wrapped = tracker.MahalanobisDistance(MeasurementVector(20.0, -3.0));
  EXPECT_LT(wrapped, tracker.config().gate_threshold);
  EXPECT_TRUE(tracker.Update(MeasurementVector(20.0, -3.0)).accepted);
}

TEST(EkfTrackerTest, ClosingSpeedIsPositiveWhenApproaching)
{
  // A 20 m/s approach: 40 m -> 30 m -> 20 m at 0.5 s spacing, so the range
  // changes by 20 m per step and each step is well inside the gate.
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(40.0, 0.0), 0.0);

  double timestamp = 0.0;
  for (double range = 30.0; range >= 20.0; range -= 10.0) {
    timestamp += 0.5;
    ASSERT_TRUE(tracker.Predict(timestamp));
    ASSERT_TRUE(tracker.Update(MeasurementVector(range, 0.0)).accepted)
      << "range " << range << " at t=" << timestamp;
  }

  EXPECT_GT(tracker.ClosingSpeed(), 0.0);
  EXPECT_NEAR(20.0, tracker.ClosingSpeed(), 5.0);
}

TEST(EkfTrackerTest, ClosingSpeedIsZeroForStationaryTarget)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(30.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(0.0, tracker.ClosingSpeed());
}

TEST(EkfTrackerTest, CovarianceStaysSymmetricAndPositiveDefinite)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(30.0, 0.2), 0.0);

  double timestamp = 0.0;
  for (int step = 0; step < 50; ++step) {
    timestamp += 0.02;
    ASSERT_TRUE(tracker.Predict(timestamp));
    tracker.Update(MeasurementVector(30.0 - 0.1 * static_cast<double>(step), 0.2));
  }

  const CovarianceMatrix & p = tracker.covariance();
  EXPECT_LT((p - p.transpose()).norm(), 1e-9);

  Eigen::SelfAdjointEigenSolver<CovarianceMatrix> solver(p);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(EkfTrackerTest, NormalizeAngleWrapsIntoMinusPiPi)
{
  const double pi = 3.14159265358979323846;
  EXPECT_NEAR(0.0, NormalizeAngle(0.0), 1e-12);
  EXPECT_NEAR(0.5, NormalizeAngle(0.5), 1e-12);
  // The range is half-open: [-pi, pi). Three turns of +0.5 collapse to +0.5.
  EXPECT_NEAR(0.5, NormalizeAngle(0.5 + 2.0 * pi), 1e-9);
  EXPECT_NEAR(0.5, NormalizeAngle(0.5 - 4.0 * pi), 1e-9);
  EXPECT_NEAR(0.5, NormalizeAngle(0.5 - 2.0 * pi), 1e-9);
  EXPECT_NEAR(-2.0, NormalizeAngle(2.0 * pi - 2.0), 1e-9);
  // +pi maps to -pi, the inclusive lower end of the range.
  EXPECT_NEAR(-pi, NormalizeAngle(pi), 1e-9);
  EXPECT_NEAR(-pi, NormalizeAngle(-pi), 1e-9);
  EXPECT_DOUBLE_EQ(0.0, NormalizeAngle(std::numeric_limits<double>::quiet_NaN()));
}

TEST(EkfTrackerTest, InnovationCovarianceIsPositiveDefinite)
{
  EkfTracker tracker;
  tracker.Initialize(MeasurementVector(15.0, -0.1), 0.0);
  const MeasurementCovariance s = tracker.InnovationCovariance();

  Eigen::SelfAdjointEigenSolver<MeasurementCovariance> solver(s);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

}  // namespace
}  // namespace sil_sensor_fusion
