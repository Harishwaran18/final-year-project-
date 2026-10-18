// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_SENSOR_FUSION__CLUSTER_GEOMETRY_HPP_
#define SIL_SENSOR_FUSION__CLUSTER_GEOMETRY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <vector>

namespace sil_sensor_fusion
{

/// Region-of-interest and clustering thresholds for the LiDAR front end.
struct ClusterConfig
{
  /// Points closer than this are discarded (ego body / sensor self-returns).
  double min_range{1.5};
  /// Points further than this are discarded (far-field noise).
  double max_range{80.0};
  /// Points below this height (ego frame, metres) are discarded after ground
  /// removal.
  double min_height{-1.8};
  /// Points above this height are discarded (overhead structures).
  double max_height{3.0};
  /// RANSAC inlier distance for the ground plane [m].
  double ground_distance_threshold{0.12};
  /// RANSAC iterations for the ground plane.
  int ground_ransac_iterations{60};
  /// Deterministic RANSAC seed so SIL replays are reproducible.
  unsigned int ground_ransac_seed{20260101U};
  /// Euclidean clustering tolerance [m].
  double cluster_tolerance{0.65};
  /// Clusters with fewer points are rejected as noise.
  std::size_t min_cluster_size{8};
  /// Clusters with more points are rejected (merged blobs / walls).
  std::size_t max_cluster_size{4000};
  /// Minimum lateral half-width for a plausible vehicle-sized cluster [m].
  double min_cluster_extent{0.25};
};

/// Plane in the form a*x + b*y + c*z + d = 0 with (a, b, c) unit length.
struct PlaneModel
{
  double a{0.0};
  double b{0.0};
  double c{1.0};
  double d{0.0};
  std::size_t inlier_count{0};
  bool valid{false};

  /// Signed distance from @p point to the plane [m].
  double SignedDistance(const Eigen::Vector3d & point) const
  {
    return a * point.x() + b * point.y() + c * point.z() + d;
  }
};

/// A clustered obstacle candidate extracted from one LiDAR sweep.
struct Cluster
{
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d min_corner{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_corner{Eigen::Vector3d::Zero()};
  std::size_t point_count{0};
  /// Mean intensity of the cluster points, carried through from PointXYZI.
  double mean_intensity{0.0};

  /// Range of the centroid from the sensor origin [m].
  double Range() const {return centroid.head<2>().norm();}
  /// Azimuth of the centroid in the ego frame [rad].
  double Bearing() const {return std::atan2(centroid.y(), centroid.x());}
};

/// Returns true when @p point lies inside the configured region of interest.
bool IsPointInRegion(const Eigen::Vector3d & point, const ClusterConfig & config);

/// Deterministic RANSAC plane fit, used to estimate the ground plane.
///
/// @param points    candidate points (typically the lowest decile of the sweep)
/// @param config    distance threshold, iteration count and seed
/// @return the best plane found; @c valid is false when no plane was supported.
PlaneModel FitGroundPlaneRansac(
  const std::vector<Eigen::Vector3d> & points, const ClusterConfig & config);

/// Arithmetic mean of @p points; returns the origin for an empty input.
Eigen::Vector3d ComputeCentroid(const std::vector<Eigen::Vector3d> & points);

/// Axis-aligned extents (max - min) of @p points; zero for an empty input.
Eigen::Vector3d ComputeExtents(const std::vector<Eigen::Vector3d> & points);

/// Applies the size and extent gates to a candidate cluster.
bool IsClusterAccepted(
  std::size_t point_count, const Eigen::Vector3d & extents, const ClusterConfig & config);

}  // namespace sil_sensor_fusion

#endif  // SIL_SENSOR_FUSION__CLUSTER_GEOMETRY_HPP_
