// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_CARLA_BRIDGE__FRAME_TRANSFORM_HPP_
#define SIL_CARLA_BRIDGE__FRAME_TRANSFORM_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sil_carla_bridge
{

/// Minimal pose in a CARLA-style world frame.
struct Pose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  /// Rotation as roll, pitch, yaw [rad]. Callers convert CARLA's degrees first.
  Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
};

/// Rigid transform from the CARLA world frame into the ROS REP-103 ego frame.
///
/// CARLA inherits the Unreal coordinate system, which is left-handed with
/// x forward, y right and z up. ROS REP-103 is right-handed with x forward,
/// y left and z up. A point therefore maps as
///
///   p_ego = F * R_ego^T * (p_world - t_ego)
///
/// where F = diag(1, -1, 1) mirrors the y axis to fix the handedness and
/// R_ego is the ego orientation expressed in the CARLA world frame.
///
/// F is orthogonal and its own inverse, so the composed matrix is orthogonal
/// and its transpose is the inverse. That is what makes @ref EgoToWorld exact.
class FrameTransform
{
public:
  FrameTransform() = default;

  /// Builds the transform from the ego pose in the CARLA world frame.
  explicit FrameTransform(const Pose & ego_pose);

  /// Converts a point from the CARLA world frame to the ROS ego frame.
  Eigen::Vector3d WorldToEgo(const Eigen::Vector3d & world_point) const;

  /// Converts a free vector (velocity, direction) into the ego frame.
  /// Translation is not applied; only the orientation changes.
  Eigen::Vector3d RotateVectorToEgo(const Eigen::Vector3d & world_vector) const;

  /// Converts a point from the ROS ego frame back into the CARLA world frame.
  Eigen::Vector3d EgoToWorld(const Eigen::Vector3d & ego_point) const;

  /// Rotation mapping CARLA world axes onto ROS ego axes, including the
  /// handedness reflection. Exposed for testing.
  const Eigen::Matrix3d & rotation() const {return rotation_;}

  /// Ego origin expressed in the CARLA world frame.
  const Eigen::Vector3d & translation() const {return translation_;}

  /// Ego heading in the CARLA world frame [rad], measured about +z.
  double EgoYaw() const;

  /// Yaw of a world-frame direction after conversion into the ego frame [rad].
  ///
  /// This is the value a box's @c yaw must carry once it is expressed in the
  /// ego frame, and it accounts for both the handedness reflection and the
  /// rotation-sense flip that the reflection introduces.
  double WorldYawToEgoYaw(double world_yaw) const;

private:
  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation_{Eigen::Vector3d::Zero()};
};

/// Builds a rotation matrix from roll, pitch, yaw using the intrinsic Z-Y-X
/// convention (yaw about z, then pitch about y, then roll about x).
Eigen::Matrix3d RotationFromRpy(const Eigen::Vector3d & rpy);

/// Converts CARLA degrees to radians.
double DegreesToRadians(double degrees);

/// Left-handed (CARLA) to right-handed (ROS) reflection matrix diag(1, -1, 1).
Eigen::Matrix3d HandednessReflection();

}  // namespace sil_carla_bridge

#endif  // SIL_CARLA_BRIDGE__FRAME_TRANSFORM_HPP_
