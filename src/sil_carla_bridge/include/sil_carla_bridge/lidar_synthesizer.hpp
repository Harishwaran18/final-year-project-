// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_CARLA_BRIDGE__LIDAR_SYNTHESIZER_HPP_
#define SIL_CARLA_BRIDGE__LIDAR_SYNTHESIZER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "sil_carla_bridge/frame_transform.hpp"

namespace sil_carla_bridge
{

/// Configuration of the synthetic LiDAR sweep.
struct LidarSynthesizerConfig
{
  /// Number of laser channels (rings).
  std::uint32_t channels{32};
  /// Azimuth samples per ring; the sweep spans a full 360 degrees.
  std::uint32_t azimuth_samples{1024};
  /// Vertical field of view [rad], centred on the horizon.
  double vertical_fov{0.5236};  // 30 degrees
  /// Minimum and maximum range [m].
  double min_range{1.0};
  double max_range{80.0};
  /// Returns below this intensity are dropped, modelling CARLA's drop-off.
  double intensity_threshold{0.0};
  /// Deterministic seed for the range noise.
  std::uint32_t noise_seed{20260101U};
  /// 1-sigma range noise [m]; 0 disables noise entirely.
  double range_noise_std{0.02};
};

/// Ray-casting LiDAR synthesiser used when CARLA is unavailable.
///
/// CARLA can publish its own LiDAR, but CI has no simulator. This class
/// ray-casts a small set of oriented boxes to produce a realistic
/// @c pcl::PointCloud<pcl::PointXYZI> so the full fusion pipeline can be
/// exercised headlessly and deterministically.
class LidarSynthesizer
{
public:
  explicit LidarSynthesizer(const LidarSynthesizerConfig & config = LidarSynthesizerConfig{});

  /// One oriented box obstacle in the ego frame.
  struct Box
  {
    /// Centre position in the ego frame [m].
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    /// Half extents along the ego axes [m].
    Eigen::Vector3d half_extents{Eigen::Vector3d::Ones()};
    /// Yaw of the box about the ego z-axis [rad].
    double yaw{0.0};
    /// Intensity assigned to returns from this box.
    double intensity{0.5};
    /// Actor id carried in the point's intensity high bits is not used; the id
    /// is kept here so callers can correlate clusters back to actors.
    std::uint32_t actor_id{0};
    std::string class_label{"vehicle"};
  };

  /// Ray-casts @p boxes and returns the resulting sweep.
  ///
  /// @param boxes obstacles in the ego frame
  /// @return an unorganised XYZI cloud with one point per hit ray
  pcl::PointCloud<pcl::PointXYZI> Synthesize(const std::vector<Box> & boxes) const;

  /// Intersects a single ray with a box.
  ///
  /// @param origin      ray origin (normally the sensor origin)
  /// @param direction   unit ray direction
  /// @param box         oriented box in the ego frame
  /// @param[out] distance hit distance along @p direction, unset on a miss
  /// @return true when the ray hits the box within the configured range
  static bool IntersectRayBox(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & direction,
    const Box & box, double & distance);

  /// Builds the unit direction for a (channel, azimuth) ray.
  Eigen::Vector3d RayDirection(std::uint32_t channel, std::uint32_t azimuth) const;

  const LidarSynthesizerConfig & config() const {return config_;}

private:
  LidarSynthesizerConfig config_;
};

/// Applies the CARLA world->ego transform to a box defined in the world frame.
LidarSynthesizer::Box TransformBoxToEgo(
  const FrameTransform & transform, const Eigen::Vector3d & center,
  const Eigen::Vector3d & half_extents, double yaw, double intensity,
  std::uint32_t actor_id, const std::string & class_label);

}  // namespace sil_carla_bridge

#endif  // SIL_CARLA_BRIDGE__LIDAR_SYNTHESIZER_HPP_
