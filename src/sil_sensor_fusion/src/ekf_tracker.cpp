// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_sensor_fusion/ekf_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sil_sensor_fusion
{
namespace
{
/// Range below which the polar measurement Jacobian is singular.
constexpr double kMinRange = 1e-3;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double NormalizeAngle(double angle)
{
  if (!std::isfinite(angle)) {
    return 0.0;
  }
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

EkfTracker::EkfTracker(const EkfConfig & config)
: config_(config) {}

StateMatrix EkfTracker::TransitionMatrix(double dt)
{
  StateMatrix f = StateMatrix::Identity();
  f(0, 2) = dt;
  f(1, 3) = dt;
  return f;
}

StateMatrix EkfTracker::ProcessNoise(double dt) const
{
  const double q = config_.process_noise_std * config_.process_noise_std;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;

  StateMatrix q_mat = StateMatrix::Zero();
  q_mat(0, 0) = dt4 / 4.0;
  q_mat(0, 2) = dt3 / 2.0;
  q_mat(2, 0) = dt3 / 2.0;
  q_mat(2, 2) = dt2;
  q_mat(1, 1) = dt4 / 4.0;
  q_mat(1, 3) = dt3 / 2.0;
  q_mat(3, 1) = dt3 / 2.0;
  q_mat(3, 3) = dt2;
  return q * q_mat;
}

MeasurementCovariance EkfTracker::MeasurementNoise() const
{
  MeasurementCovariance r = MeasurementCovariance::Zero();
  r(0, 0) = config_.range_noise_std * config_.range_noise_std;
  r(1, 1) = config_.bearing_noise_std * config_.bearing_noise_std;
  return r;
}

void EkfTracker::Initialize(const MeasurementVector & z, double timestamp)
{
  const double range = z(0);
  const double bearing = z(1);
  x_(0) = range * std::cos(bearing);
  x_(1) = range * std::sin(bearing);
  x_(2) = 0.0;
  x_(3) = 0.0;

  p_.setZero();
  p_(0, 0) = config_.initial_position_std * config_.initial_position_std;
  p_(1, 1) = p_(0, 0);
  p_(2, 2) = config_.initial_velocity_std * config_.initial_velocity_std;
  p_(3, 3) = p_(2, 2);

  timestamp_ = timestamp;
  initialized_ = true;
}

bool EkfTracker::Predict(double timestamp)
{
  if (!initialized_) {
    return false;
  }
  const double dt = timestamp - timestamp_;
  if (dt <= 0.0 || dt > config_.max_dt) {
    return false;
  }

  const StateMatrix f = TransitionMatrix(dt);
  x_ = f * x_;
  p_ = f * p_ * f.transpose() + ProcessNoise(dt);

  // Keep the covariance symmetric against floating point drift.
  p_ = 0.5 * (p_ + p_.transpose());
  timestamp_ = timestamp;
  return true;
}

MeasurementVector EkfTracker::PredictedMeasurement() const
{
  const double range = std::hypot(x_(0), x_(1));
  return MeasurementVector(range, std::atan2(x_(1), x_(0)));
}

MeasurementMatrix EkfTracker::MeasurementJacobian() const
{
  const double px = x_(0);
  const double py = x_(1);
  const double range2 = px * px + py * py;
  const double range = std::sqrt(range2);

  MeasurementMatrix h = MeasurementMatrix::Zero();
  if (range < kMinRange) {
    return h;
  }
  const double inv_range = 1.0 / range;
  const double inv_range2 = 1.0 / range2;

  // d(range)/d(x, y)
  h(0, 0) = px * inv_range;
  h(0, 1) = py * inv_range;
  // d(atan2(y, x))/d(x, y)
  h(1, 0) = -py * inv_range2;
  h(1, 1) = px * inv_range2;
  return h;
}

MeasurementMatrix EkfTracker::NumericalMeasurementJacobian(double epsilon) const
{
  MeasurementMatrix h = MeasurementMatrix::Zero();
  for (int i = 0; i < StateVector::RowsAtCompileTime; ++i) {
    StateVector plus = x_;
    StateVector minus = x_;
    plus(i) += epsilon;
    minus(i) -= epsilon;

    const MeasurementVector h_plus(
      std::hypot(plus(0), plus(1)), std::atan2(plus(1), plus(0)));
    const MeasurementVector h_minus(
      std::hypot(minus(0), minus(1)), std::atan2(minus(1), minus(0)));

    h(0, i) = (h_plus(0) - h_minus(0)) / (2.0 * epsilon);
    h(1, i) = NormalizeAngle(h_plus(1) - h_minus(1)) / (2.0 * epsilon);
  }
  return h;
}

MeasurementCovariance EkfTracker::InnovationCovariance(
  const MeasurementCovariance * noise_override) const
{
  const MeasurementMatrix h = MeasurementJacobian();
  const MeasurementCovariance r = noise_override != nullptr ?
    *noise_override : MeasurementNoise();
  return h * p_ * h.transpose() + r;
}

double EkfTracker::MahalanobisDistance(
  const MeasurementVector & z, const MeasurementCovariance * noise_override) const
{
  const MeasurementVector h_x = PredictedMeasurement();
  MeasurementVector y = z - h_x;
  y(1) = NormalizeAngle(y(1));

  const MeasurementCovariance s = InnovationCovariance(noise_override);
  const Eigen::Matrix2d s_inv = s.inverse();
  return y.transpose() * s_inv * y;
}

EkfTracker::UpdateResult EkfTracker::Update(
  const MeasurementVector & z, const MeasurementCovariance * noise_override)
{
  UpdateResult result;
  if (!initialized_ || !z.allFinite()) {
    return result;
  }

  const double range = std::hypot(x_(0), x_(1));
  if (range < kMinRange) {
    return result;
  }

  const MeasurementVector h_x = PredictedMeasurement();
  MeasurementVector y = z - h_x;
  y(1) = NormalizeAngle(y(1));

  const MeasurementMatrix h = MeasurementJacobian();
  const MeasurementCovariance r = noise_override != nullptr ?
    *noise_override : MeasurementNoise();
  const MeasurementCovariance s = h * p_ * h.transpose() + r;

  const Eigen::Matrix2d s_inv = s.inverse();
  const double d2 = y.transpose() * s_inv * y;
  result.mahalanobis = d2;

  if (!std::isfinite(d2) || d2 > config_.gate_threshold) {
    return result;
  }

  const Eigen::Matrix<double, 4, 2> k = p_ * h.transpose() * s_inv;
  x_ += k * y;

  // Joseph form keeps the covariance symmetric and positive semi-definite.
  const CovarianceMatrix i_kh = CovarianceMatrix::Identity() - k * h;
  p_ = i_kh * p_ * i_kh.transpose() + k * r * k.transpose();
  p_ = 0.5 * (p_ + p_.transpose());

  result.accepted = true;
  return result;
}

double EkfTracker::ClosingSpeed() const
{
  const double range = std::hypot(x_(0), x_(1));
  if (range < kMinRange) {
    return 0.0;
  }
  // Radial rate of change, positive when the range is shrinking.
  return -(x_(0) * x_(2) + x_(1) * x_(3)) / range;
}

double EkfTracker::Range() const
{
  return std::hypot(x_(0), x_(1));
}

double EkfTracker::PositionUncertainty() const
{
  return std::sqrt(std::max(0.0, p_(0, 0) + p_(1, 1)));
}

}  // namespace sil_sensor_fusion
