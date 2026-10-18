// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_carla_bridge/lidar_synthesizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace sil_carla_bridge
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}  // namespace

LidarSynthesizer::LidarSynthesizer(const LidarSynthesizerConfig & config)
: config_(config) {}

Eigen::Vector3d LidarSynthesizer::RayDirection(
  std::uint32_t channel, std::uint32_t azimuth) const
{
  const std::uint32_t channels = std::max(1U, config_.channels);
  const std::uint32_t samples = std::max(1U, config_.azimuth_samples);

  // Elevation sweeps the vertical FOV from top to bottom across the channels.
  const double elevation_step = channels > 1 ?
    config_.vertical_fov / static_cast<double>(channels - 1) : 0.0;
  const double elevation = config_.vertical_fov / 2.0 -
    static_cast<double>(channel) * elevation_step;

  const double azimuth_angle =
    kTwoPi * static_cast<double>(azimuth) / static_cast<double>(samples);

  const double cos_elevation = std::cos(elevation);
  return Eigen::Vector3d(
    cos_elevation * std::cos(azimuth_angle),
    cos_elevation * std::sin(azimuth_angle),
    std::sin(elevation));
}

bool LidarSynthesizer::IntersectRayBox(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & direction,
  const Box & box, double & distance)
{
  // Transform the ray into the box frame, then run the slab test. This handles
  // arbitrarily yawed boxes without needing their vertices.
  const double cos_yaw = std::cos(-box.yaw);
  const double sin_yaw = std::sin(-box.yaw);
  Eigen::Matrix3d rotation;
  rotation << cos_yaw, -sin_yaw, 0.0,
    sin_yaw, cos_yaw, 0.0,
    0.0, 0.0, 1.0;

  const Eigen::Vector3d local_origin = rotation * (origin - box.center);
  const Eigen::Vector3d local_direction = rotation * direction;

  double t_min = -std::numeric_limits<double>::infinity();
  double t_max = std::numeric_limits<double>::infinity();

  for (int axis = 0; axis < 3; ++axis) {
    const double half = box.half_extents(axis);
    const double o = local_origin(axis);
    const double d = local_direction(axis);

    if (std::abs(d) < 1e-9) {
      // Ray is parallel to this slab: a miss unless the origin is inside it.
      if (o < -half || o > half) {
        return false;
      }
      continue;
    }

    double t1 = (-half - o) / d;
    double t2 = (half - o) / d;
    if (t1 > t2) {
      std::swap(t1, t2);
    }
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
    if (t_min > t_max) {
      return false;
    }
  }

  if (t_max < 0.0) {
    return false;  // Box is entirely behind the ray origin.
  }

  // Prefer the entry point; if the origin is inside the box, use the exit.
  const double t_hit = t_min >= 0.0 ? t_min : t_max;
  if (t_hit < 0.0) {
    return false;
  }
  distance = t_hit;
  return true;
}

pcl::PointCloud<pcl::PointXYZI> LidarSynthesizer::Synthesize(
  const std::vector<Box> & boxes) const
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(static_cast<std::size_t>(config_.channels) * config_.azimuth_samples);

  std::mt19937 rng(config_.noise_seed);
  std::normal_distribution<double> noise(0.0, config_.range_noise_std);

  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  for (std::uint32_t channel = 0; channel < config_.channels; ++channel) {
    for (std::uint32_t azimuth = 0; azimuth < config_.azimuth_samples; ++azimuth) {
      const Eigen::Vector3d direction = RayDirection(channel, azimuth);

      double closest = std::numeric_limits<double>::infinity();
      double intensity = 0.0;

      for (const auto & box : boxes) {
        double hit_distance = 0.0;
        if (!IntersectRayBox(origin, direction, box, hit_distance)) {
          continue;
        }
        if (hit_distance < closest) {
          closest = hit_distance;
          intensity = box.intensity;
        }
      }

      if (!std::isfinite(closest)) {
        continue;
      }
      if (closest < config_.min_range || closest > config_.max_range) {
        continue;
      }
      if (intensity < config_.intensity_threshold) {
        continue;
      }

      const double measured = config_.range_noise_std > 0.0 ?
        closest + noise(rng) : closest;

      pcl::PointXYZI point;
      point.x = static_cast<float>(direction.x() * measured);
      point.y = static_cast<float>(direction.y() * measured);
      point.z = static_cast<float>(direction.z() * measured);
      point.intensity = static_cast<float>(intensity);
      cloud.push_back(point);
    }
  }

  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

LidarSynthesizer::Box TransformBoxToEgo(
  const FrameTransform & transform, const Eigen::Vector3d & center,
  const Eigen::Vector3d & half_extents, double yaw, double intensity,
  std::uint32_t actor_id, const std::string & class_label)
{
  LidarSynthesizer::Box box;
  box.center = transform.WorldToEgo(center);
  box.half_extents = half_extents;
  // The handedness reflection flips the sense of a rotation about z, so the
  // yaw must be recomputed rather than merely offset.
  box.yaw = transform.WorldYawToEgoYaw(yaw);
  box.intensity = intensity;
  box.actor_id = actor_id;
  box.class_label = class_label;
  return box;
}

}  // namespace sil_carla_bridge
