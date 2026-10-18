// Copyright 2026 SIL AEB Maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIL_SENSOR_FUSION__EKF_TRACKER_HPP_
#define SIL_SENSOR_FUSION__EKF_TRACKER_HPP_

#include <Eigen/Core>
#include <Eigen/LU>

namespace sil_sensor_fusion
{

/// Constant-velocity target state: X = [x, y, vx, vy]^T in the ego frame.
using StateVector = Eigen::Vector4d;
using StateMatrix = Eigen::Matrix4d;
using CovarianceMatrix = Eigen::Matrix4d;

/// Radar/LiDAR style polar measurement: Z = [range, bearing]^T.
using MeasurementVector = Eigen::Vector2d;
using MeasurementMatrix = Eigen::Matrix<double, 2, 4>;
using MeasurementCovariance = Eigen::Matrix2d;

/// Tuning for one constant-velocity EKF.
struct EkfConfig
{
  /// Continuous-time acceleration noise PSD [m^2/s^3]. Sets how quickly the
  /// filter trusts measured manoeuvres over its constant-velocity prior.
  double process_noise_std{1.5};

  /// 1-sigma range noise [m].
  double range_noise_std{0.25};

  /// 1-sigma bearing noise [rad].
  double bearing_noise_std{0.02};

  /// Mahalanobis gate for innovation rejection. Default is the 99% quantile of
  /// a chi-square distribution with 2 degrees of freedom (9.2103).
  double gate_threshold{9.2103};

  /// Upper bound on the integration step [s]. Longer gaps reset the filter
  /// rather than integrating a stale constant-velocity prior.
  double max_dt{0.5};

  /// Initial 1-sigma position uncertainty [m] on track birth.
  double initial_position_std{2.0};

  /// Initial 1-sigma velocity uncertainty [m/s] on track birth.
  double initial_velocity_std{10.0};
};

/// Single-target Extended Kalman Filter for range/bearing observations.
///
/// The filter is intentionally free of ROS, PCL and OpenCV so that it can be
/// unit tested in isolation and reused by offline replay tooling.
class EkfTracker
{
public:
  explicit EkfTracker(const EkfConfig & config = EkfConfig{});

  /// Seeds the state from the first polar observation with a stationary prior.
  /// @param z      initial [range, bearing] measurement.
  /// @param timestamp measurement time [s].
  void Initialize(const MeasurementVector & z, double timestamp);

  /// Propagates the state to @p timestamp using the constant-velocity model.
  /// @return false when the tracker is uninitialised or @p timestamp is not
  ///         strictly newer than the last update.
  bool Predict(double timestamp);

  /// Outcome of an innovation-gated measurement update.
  struct UpdateResult
  {
    /// False when the innovation failed the Mahalanobis gate.
    bool accepted{false};
    /// Squared Mahalanobis distance of the innovation, d^2 = y^T S^-1 y.
    double mahalanobis{0.0};
  };

  /// Applies a gated EKF measurement update.
  ///
  /// The state is left untouched when the gate rejects the measurement, which
  /// lets the caller fall back to another measurement for the same track.
  ///
  /// @param z          measurement [range, bearing]
  /// @param noise_override optional measurement covariance for this update;
  ///        pass nullptr to use the configured default sigmas. Camera-derived
  ///        observations use this to declare their larger uncertainty.
  UpdateResult Update(
    const MeasurementVector & z,
    const MeasurementCovariance * noise_override = nullptr);

  /// Predicted measurement h(x) = [range, bearing] for the current state.
  MeasurementVector PredictedMeasurement() const;

  /// Analytic Jacobian of h(x) with respect to X.
  MeasurementMatrix MeasurementJacobian() const;

  /// Central-difference Jacobian of h(x); used to validate the analytic form.
  MeasurementMatrix NumericalMeasurementJacobian(double epsilon = 1e-6) const;

  /// Innovation covariance S = H P H^T + R.
  MeasurementCovariance InnovationCovariance(
    const MeasurementCovariance * noise_override = nullptr) const;

  /// Mahalanobis distance of @p z against the current state without mutating it.
  double MahalanobisDistance(
    const MeasurementVector & z,
    const MeasurementCovariance * noise_override = nullptr) const;

  /// Signed closing speed along the ego x-axis [m/s]; positive means closing.
  double ClosingSpeed() const;

  /// Euclidean range of the current state [m].
  double Range() const;

  /// 1-sigma positional uncertainty, sqrt(trace of the position block) [m].
  double PositionUncertainty() const;

  bool initialized() const {return initialized_;}
  double timestamp() const {return timestamp_;}
  const StateVector & state() const {return x_;}
  const CovarianceMatrix & covariance() const {return p_;}
  const EkfConfig & config() const {return config_;}

private:
  /// Constant-velocity transition matrix for a step of @p dt.
  static StateMatrix TransitionMatrix(double dt);

  /// Discrete process noise for a step of @p dt (piecewise-constant
  /// acceleration model).
  StateMatrix ProcessNoise(double dt) const;

  /// Measurement noise covariance from the configured range/bearing sigmas.
  MeasurementCovariance MeasurementNoise() const;

  EkfConfig config_;
  StateVector x_{StateVector::Zero()};
  CovarianceMatrix p_{CovarianceMatrix::Identity()};
  double timestamp_{0.0};
  bool initialized_{false};
};

/// Wraps @p angle into [-pi, pi); the bearing residual must be wrapped before
/// gating or the filter diverges at the +/-pi seam.
double NormalizeAngle(double angle);

}  // namespace sil_sensor_fusion

#endif  // SIL_SENSOR_FUSION__EKF_TRACKER_HPP_
