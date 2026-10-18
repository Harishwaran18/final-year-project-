// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_carla_bridge/frame_transform.hpp"

#include <cmath>

namespace sil_carla_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double DegreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

Eigen::Matrix3d HandednessReflection()
{
  // CARLA is left-handed with y to the right; ROS REP-103 has y to the left.
  // Negating y maps one onto the other.
  Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
  reflection(1, 1) = -1.0;
  return reflection;
}

Eigen::Matrix3d RotationFromRpy(const Eigen::Vector3d & rpy)
{
  const Eigen::AngleAxisd roll_angle(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(rpy.z(), Eigen::Vector3d::UnitZ());

  // Intrinsic Z-Y-X: R = Rz(yaw) * Ry(pitch) * Rx(roll).
  return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
}

FrameTransform::FrameTransform(const Pose & ego_pose)
: translation_(ego_pose.position)
{
  const Eigen::Matrix3d r_ego = RotationFromRpy(ego_pose.rpy);
  // p_ego = F * R_ego^T * (p_world - t_ego).
  rotation_ = HandednessReflection() * r_ego.transpose();
}

Eigen::Vector3d FrameTransform::WorldToEgo(const Eigen::Vector3d & world_point) const
{
  return rotation_ * (world_point - translation_);
}

Eigen::Vector3d FrameTransform::RotateVectorToEgo(const Eigen::Vector3d & world_vector) const
{
  return rotation_ * world_vector;
}

Eigen::Vector3d FrameTransform::EgoToWorld(const Eigen::Vector3d & ego_point) const
{
  // rotation_ is orthogonal, so its transpose is its inverse.
  return rotation_.transpose() * ego_point + translation_;
}

double FrameTransform::EgoYaw() const
{
  // The ego x-axis in world coordinates is R_ego * ex. Since
  // rotation_ = F * R_ego^T we have R_ego = rotation_.transpose() * F, and
  // F * ex == ex, so the world-frame forward direction is the first column of
  // rotation_.transpose().
  const Eigen::Vector3d forward = rotation_.transpose() * Eigen::Vector3d::UnitX();
  return std::atan2(forward.y(), forward.x());
}

double FrameTransform::WorldYawToEgoYaw(double world_yaw) const
{
  const Eigen::Vector3d world_direction(std::cos(world_yaw), std::sin(world_yaw), 0.0);
  const Eigen::Vector3d ego_direction = RotateVectorToEgo(world_direction);
  return std::atan2(ego_direction.y(), ego_direction.x());
}

}  // namespace sil_carla_bridge
