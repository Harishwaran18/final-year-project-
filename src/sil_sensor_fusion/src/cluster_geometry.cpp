// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_sensor_fusion/cluster_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace sil_sensor_fusion
{
namespace
{
/// Minimum number of points needed to fit a plane by RANSAC.
constexpr std::size_t kMinPointsForPlane = 3;

bool IsFinite(const Eigen::Vector3d & p)
{
  return p.allFinite();
}
}  // namespace

bool IsPointInRegion(const Eigen::Vector3d & point, const ClusterConfig & config)
{
  if (!IsFinite(point)) {
    return false;
  }
  const double range = point.head<2>().norm();
  if (range < config.min_range || range > config.max_range) {
    return false;
  }
  return point.z() >= config.min_height && point.z() <= config.max_height;
}

PlaneModel FitGroundPlaneRansac(
  const std::vector<Eigen::Vector3d> & points, const ClusterConfig & config)
{
  PlaneModel best;
  if (points.size() < kMinPointsForPlane || config.ground_ransac_iterations <= 0) {
    return best;
  }

  std::mt19937 rng(config.ground_ransac_seed);
  std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);

  for (int iteration = 0; iteration < config.ground_ransac_iterations; ++iteration) {
    const Eigen::Vector3d & p0 = points[pick(rng)];
    const Eigen::Vector3d & p1 = points[pick(rng)];
    const Eigen::Vector3d & p2 = points[pick(rng)];

    // Named locals rather than nested expression templates: this keeps cross()
    // on a plain MatrixBase and evaluates each difference once.
    const Eigen::Vector3d edge1 = p1 - p0;
    const Eigen::Vector3d edge2 = p2 - p0;

    const Eigen::Vector3d normal = edge1.cross(edge2);
    const double norm = normal.norm();
    if (norm < 1e-6) {
      continue;  // Degenerate (collinear) sample.
    }

    PlaneModel candidate;
    candidate.a = normal.x() / norm;
    candidate.b = normal.y() / norm;
    candidate.c = normal.z() / norm;
    candidate.d = -(candidate.a * p0.x() + candidate.b * p0.y() + candidate.c * p0.z());

    // Reject planes that are not roughly horizontal: the ground is the plane we
    // want, and a near-vertical plane would otherwise dominate the inliers.
    if (std::abs(candidate.c) < 0.85) {
      continue;
    }

    // The cross product fixes the normal's sign arbitrarily. Orient it upwards
    // so SignedDistance() is positive above the ground; otherwise a downward
    // normal inverts every above-ground test and removes the obstacles instead
    // of the ground.
    if (candidate.c < 0.0) {
      candidate.a = -candidate.a;
      candidate.b = -candidate.b;
      candidate.c = -candidate.c;
      candidate.d = -candidate.d;
    }

    std::size_t inliers = 0;
    for (const auto & point : points) {
      if (std::abs(candidate.SignedDistance(point)) <= config.ground_distance_threshold) {
        ++inliers;
      }
    }

    if (inliers > best.inlier_count) {
      candidate.inlier_count = inliers;
      candidate.valid = true;
      best = candidate;
    }
  }
  return best;
}

Eigen::Vector3d ComputeCentroid(const std::vector<Eigen::Vector3d> & points)
{
  if (points.empty()) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto & point : points) {
    sum += point;
  }
  return sum / static_cast<double>(points.size());
}

Eigen::Vector3d ComputeExtents(const std::vector<Eigen::Vector3d> & points)
{
  if (points.empty()) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d min_corner = points.front();
  Eigen::Vector3d max_corner = points.front();
  for (const auto & point : points) {
    min_corner = min_corner.cwiseMin(point);
    max_corner = max_corner.cwiseMax(point);
  }
  return max_corner - min_corner;
}

bool IsClusterAccepted(
  std::size_t point_count, const Eigen::Vector3d & extents, const ClusterConfig & config)
{
  if (point_count < config.min_cluster_size || point_count > config.max_cluster_size) {
    return false;
  }
  // A real object must have some lateral extent; a single scan line is noise.
  const double lateral = std::max(extents.x(), extents.y());
  return lateral >= config.min_cluster_extent;
}

}  // namespace sil_sensor_fusion
