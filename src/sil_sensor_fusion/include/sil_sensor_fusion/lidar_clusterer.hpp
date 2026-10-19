// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_SENSOR_FUSION__LIDAR_CLUSTERER_HPP_
#define SIL_SENSOR_FUSION__LIDAR_CLUSTERER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "sil_sensor_fusion/cluster_geometry.hpp"

namespace sil_sensor_fusion
{

/// Ground-removal, ROI filtering and Euclidean clustering for one LiDAR sweep.
///
/// This class depends only on PCL (no ROS types) so it can be exercised by
/// GoogleTest on a plain build host and by offline SIL replay tools.
class LidarClusterer
{
public:
  explicit LidarClusterer(const ClusterConfig & config = ClusterConfig{});

  /// Clusters a single sweep into obstacle candidates.
  ///
  /// Pipeline: ROI crop -> ground plane removal (RANSAC) -> voxel-free
  /// Euclidean clustering -> size/extent gating -> centroid/intensity summary.
  ///
  /// @param cloud organised or unorganised XYZI sweep in the ego frame
  /// @return accepted clusters, ordered by descending point count
  std::vector<Cluster> ExtractClusters(const pcl::PointCloud<pcl::PointXYZI> & cloud) const;

  /// ROI-crops and de-grounds @p cloud; exposed for reuse and testing.
  pcl::PointCloud<pcl::PointXYZI> Preprocess(const pcl::PointCloud<pcl::PointXYZI> & cloud) const;

  /// Last ground plane estimated by ExtractClusters(); invalid before the first call.
  const PlaneModel & ground_plane() const {return ground_plane_;}

  const ClusterConfig & config() const {return config_;}

private:
  ClusterConfig config_;
  mutable PlaneModel ground_plane_;
};

}  // namespace sil_sensor_fusion

#endif  // SIL_SENSOR_FUSION__LIDAR_CLUSTERER_HPP_
