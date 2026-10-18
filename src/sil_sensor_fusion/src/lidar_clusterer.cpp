// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_sensor_fusion/lidar_clusterer.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/common/point_tests.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sil_sensor_fusion
{
namespace
{
/// Fraction of the lowest points sampled for the ground plane RANSAC fit.
constexpr double kGroundSampleFraction = 0.35;
}  // namespace

LidarClusterer::LidarClusterer(const ClusterConfig & config)
: config_(config) {}

pcl::PointCloud<pcl::PointXYZI> LidarClusterer::Preprocess(
  const pcl::PointCloud<pcl::PointXYZI> & cloud) const
{
  // --- ROI crop and finiteness filter. -------------------------------------
  pcl::PointCloud<pcl::PointXYZI> cropped;
  cropped.reserve(cloud.size());
  for (const auto & point : cloud.points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const Eigen::Vector3d p(point.x, point.y, point.z);
    if (IsPointInRegion(p, config_)) {
      cropped.push_back(point);
    }
  }

  ground_plane_ = PlaneModel{};
  if (cropped.size() < 8) {
    return cropped;
  }

  // --- Ground plane fit on the lowest slice of the sweep. ------------------
  std::vector<Eigen::Vector3d> ground_candidates;
  ground_candidates.reserve(cropped.size());
  for (const auto & point : cropped.points) {
    ground_candidates.emplace_back(point.x, point.y, point.z);
  }
  std::sort(ground_candidates.begin(), ground_candidates.end(),
    [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) {return a.z() < b.z();});

  const std::size_t sample_size = std::max<std::size_t>(
    8, static_cast<std::size_t>(
      static_cast<double>(ground_candidates.size()) * kGroundSampleFraction));
  ground_candidates.resize(std::min(sample_size, ground_candidates.size()));

  ground_plane_ = FitGroundPlaneRansac(ground_candidates, config_);

  // --- Remove the ground. --------------------------------------------------
  pcl::PointCloud<pcl::PointXYZI> above_ground;
  above_ground.reserve(cropped.size());
  if (ground_plane_.valid && ground_plane_.inlier_count > 0) {
    const double height_band = config_.ground_distance_threshold;
    for (const auto & point : cropped.points) {
      const Eigen::Vector3d p(point.x, point.y, point.z);
      // Keep points strictly above the ground band; this also removes the
      // ground plane's own inliers.
      if (ground_plane_.SignedDistance(p) > height_band) {
        above_ground.push_back(point);
      }
    }
  } else {
    // No usable ground plane: fall back to the configured height window.
    above_ground = cropped;
  }

  return above_ground;
}

std::vector<Cluster> LidarClusterer::ExtractClusters(
  const pcl::PointCloud<pcl::PointXYZI> & cloud) const
{
  std::vector<Cluster> clusters;
  const pcl::PointCloud<pcl::PointXYZI> filtered = Preprocess(cloud);
  if (filtered.empty()) {
    return clusters;
  }

  // --- Euclidean clustering over a KdTree index. ---------------------------
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(filtered.makeShared());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> extractor;
  extractor.setClusterTolerance(config_.cluster_tolerance);
  extractor.setMinClusterSize(static_cast<int>(config_.min_cluster_size));
  extractor.setMaxClusterSize(static_cast<int>(config_.max_cluster_size));
  extractor.setSearchMethod(tree);
  extractor.setInputCloud(filtered.makeShared());
  extractor.extract(cluster_indices);

  for (const auto & indices : cluster_indices) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(indices.indices.size());
    double intensity_sum = 0.0;
    Eigen::Vector3d min_corner(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max());
    Eigen::Vector3d max_corner(
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest());

    for (const int index : indices.indices) {
      const auto & point = filtered.points[static_cast<std::size_t>(index)];
      const Eigen::Vector3d p(point.x, point.y, point.z);
      points.push_back(p);
      intensity_sum += point.intensity;
      min_corner = min_corner.cwiseMin(p);
      max_corner = max_corner.cwiseMax(p);
    }

    const Eigen::Vector3d extents = max_corner - min_corner;
    if (!IsClusterAccepted(points.size(), extents, config_)) {
      continue;
    }

    Cluster cluster;
    cluster.centroid = ComputeCentroid(points);
    cluster.min_corner = min_corner;
    cluster.max_corner = max_corner;
    cluster.point_count = points.size();
    cluster.mean_intensity = intensity_sum / static_cast<double>(points.size());
    clusters.push_back(cluster);
  }

  // Deterministic output: largest cluster first, centroid as tie breaker.
  std::sort(clusters.begin(), clusters.end(), [](const Cluster & a, const Cluster & b) {
    if (a.point_count != b.point_count) {
      return a.point_count > b.point_count;
    }
    if (a.centroid.x() != b.centroid.x()) {
      return a.centroid.x() < b.centroid.x();
    }
    return a.centroid.y() < b.centroid.y();
  });

  return clusters;
}

}  // namespace sil_sensor_fusion
