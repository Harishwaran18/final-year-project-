// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "sil_sensor_fusion/lidar_clusterer.hpp"

namespace sil_sensor_fusion
{
namespace
{

/// Adds a dense axis-aligned block of points, mimicking one object surface.
void AddBlock(pcl::PointCloud<pcl::PointXYZI> & cloud, double cx, double cy,
  double cz, double sx, double sy, double sz, double intensity, int per_axis = 6)
{
  for (int i = 0; i < per_axis; ++i) {
    for (int j = 0; j < per_axis; ++j) {
      for (int k = 0; k < per_axis; ++k) {
        pcl::PointXYZI point;
        point.x = static_cast<float>(cx - sx / 2.0 + sx * i / (per_axis - 1.0));
        point.y = static_cast<float>(cy - sy / 2.0 + sy * j / (per_axis - 1.0));
        point.z = static_cast<float>(cz - sz / 2.0 + sz * k / (per_axis - 1.0));
        point.intensity = static_cast<float>(intensity);
        cloud.push_back(point);
      }
    }
  }
}

/// Builds a ground plane plus one vehicle-sized block at @p range.
pcl::PointCloud<pcl::PointXYZI> MakeScene(double range)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  // Ground: a dense grid at z = -1.5, the sensor mounted 1.5 m up.
  for (int i = 0; i < 40; ++i) {
    for (int j = -20; j <= 20; ++j) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(i * 2.0);
      point.y = static_cast<float>(j * 0.5);
      point.z = -1.5F;
      point.intensity = 0.1F;
      cloud.push_back(point);
    }
  }
  // One object sitting on the ground, centred on the ego axis.
  AddBlock(cloud, range, 0.0, -0.75, 1.8, 1.8, 1.5, 0.8);
  return cloud;
}

TEST(LidarClustererTest, EmptyCloudProducesNoClusters)
{
  LidarClusterer clusterer;
  const pcl::PointCloud<pcl::PointXYZI> cloud;
  EXPECT_TRUE(clusterer.ExtractClusters(cloud).empty());
}

TEST(LidarClustererTest, PreprocessRejectsNonFinitePoints)
{
  LidarClusterer clusterer;
  pcl::PointCloud<pcl::PointXYZI> cloud;

  pcl::PointXYZI good;
  good.x = 10.0F;
  good.y = 0.0F;
  good.z = 0.0F;
  good.intensity = 0.5F;
  cloud.push_back(good);

  pcl::PointXYZI nan_point = good;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(nan_point);

  const pcl::PointCloud<pcl::PointXYZI> filtered = clusterer.Preprocess(cloud);
  EXPECT_LT(filtered.size(), cloud.size());
}

TEST(LidarClustererTest, PreprocessCropsOutsideTheRegionOfInterest)
{
  LidarClusterer clusterer;
  pcl::PointCloud<pcl::PointXYZI> cloud;

  for (int i = 0; i < 20; ++i) {
    pcl::PointXYZI near;
    near.x = static_cast<float>(i);
    near.y = 0.0F;
    near.z = 0.0F;
    near.intensity = 0.5F;
    cloud.push_back(near);

    // 500 m away: far outside the 80 m maximum range.
    pcl::PointXYZI far = near;
    far.x = 500.0F;
    cloud.push_back(far);
  }

  const pcl::PointCloud<pcl::PointXYZI> filtered = clusterer.Preprocess(cloud);
  for (const auto & point : filtered.points) {
    EXPECT_LT(point.x, 100.0F);
  }
}

TEST(LidarClustererTest, FindsOneClusterInASyntheticScene)
{
  LidarClusterer clusterer;
  const pcl::PointCloud<pcl::PointXYZI> cloud = MakeScene(20.0);

  const std::vector<Cluster> clusters = clusterer.ExtractClusters(cloud);

  ASSERT_EQ(1U, clusters.size()) << "expected exactly one object cluster";
  EXPECT_NEAR(20.0, clusters.front().centroid.x(), 1.5);
  EXPECT_NEAR(0.0, clusters.front().centroid.y(), 0.5);
  EXPECT_GT(clusters.front().point_count, 100U);
}

TEST(LidarClustererTest, EstimatesTheGroundPlane)
{
  LidarClusterer clusterer;
  clusterer.ExtractClusters(MakeScene(20.0));

  const PlaneModel & plane = clusterer.ground_plane();
  ASSERT_TRUE(plane.valid) << "the ground plane must be detected";
  EXPECT_GT(std::abs(plane.c), 0.85) << "the ground must be roughly horizontal";
  // The plane passes through z = -1.5, so with a unit z normal |d| is 1.5.
  EXPECT_NEAR(1.5, std::abs(plane.d), 0.3);
}

TEST(LidarClustererTest, FindsTwoSeparatedObjects)
{
  pcl::PointCloud<pcl::PointXYZI> cloud = MakeScene(20.0);
  AddBlock(cloud, 40.0, 0.0, -0.75, 1.8, 1.8, 1.5, 0.8);

  LidarClusterer clusterer;
  const std::vector<Cluster> clusters = clusterer.ExtractClusters(cloud);

  ASSERT_EQ(2U, clusters.size());
  // Deterministic ordering: the larger cluster comes first. Both blocks are the
  // same size here, so ordering falls back to the centroid x coordinate.
  EXPECT_LT(clusters.front().centroid.x(), clusters.back().centroid.x());
}

TEST(LidarClustererTest, MergesObjectsWithinTheClusterTolerance)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  AddBlock(cloud, 20.0, 0.0, 0.0, 1.8, 1.8, 1.5, 0.8);
  // 0.2 m apart: well inside the 0.65 m tolerance, so they merge.
  AddBlock(cloud, 20.2, 0.0, 0.0, 1.8, 1.8, 1.5, 0.8);

  LidarClusterer clusterer;
  const std::vector<Cluster> clusters = clusterer.ExtractClusters(cloud);
  EXPECT_EQ(1U, clusters.size());
}

TEST(LidarClustererTest, RejectsClustersSmallerThanTheMinimumSize)
{
  ClusterConfig config;
  config.min_cluster_size = 500;  // Nothing in the scene is that big.
  LidarClusterer clusterer(config);

  EXPECT_TRUE(clusterer.ExtractClusters(MakeScene(20.0)).empty());
}

TEST(LidarClustererTest, RejectsClustersLargerThanTheMaximumSize)
{
  ClusterConfig config;
  config.max_cluster_size = 50;
  LidarClusterer clusterer(config);

  EXPECT_TRUE(clusterer.ExtractClusters(MakeScene(20.0)).empty());
}

TEST(LidarClustererTest, ClusterOutputIsDeterministic)
{
  LidarClusterer clusterer;
  const pcl::PointCloud<pcl::PointXYZI> cloud = MakeScene(25.0);

  const std::vector<Cluster> first = clusterer.ExtractClusters(cloud);
  const std::vector<Cluster> second = clusterer.ExtractClusters(cloud);

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(first[i].point_count, second[i].point_count);
    EXPECT_EQ(first[i].centroid, second[i].centroid);
  }
}

TEST(LidarClustererTest, ClusterRangeAndBearingMatchTheCentroid)
{
  LidarClusterer clusterer;
  const std::vector<Cluster> clusters = clusterer.ExtractClusters(MakeScene(30.0));
  ASSERT_FALSE(clusters.empty());

  const Cluster & cluster = clusters.front();
  EXPECT_NEAR(std::hypot(cluster.centroid.x(), cluster.centroid.y()),
    cluster.Range(), 1e-9);
  EXPECT_NEAR(std::atan2(cluster.centroid.y(), cluster.centroid.x()),
    cluster.Bearing(), 1e-9);
}

TEST(LidarClustererTest, MeanIntensityIsAveragedOverTheCluster)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  AddBlock(cloud, 20.0, 0.0, 0.0, 2.0, 2.0, 1.0, 0.9, 8);

  LidarClusterer clusterer;
  const std::vector<Cluster> clusters = clusterer.ExtractClusters(cloud);
  ASSERT_FALSE(clusters.empty());
  EXPECT_NEAR(0.9, clusters.front().mean_intensity, 1e-6);
}

TEST(LidarClustererTest, ClusterCornersBracketTheCentroid)
{
  LidarClusterer clusterer;
  const std::vector<Cluster> clusters = clusterer.ExtractClusters(MakeScene(20.0));
  ASSERT_FALSE(clusters.empty());

  const Cluster & cluster = clusters.front();
  EXPECT_LE(cluster.min_corner.x(), cluster.centroid.x());
  EXPECT_GE(cluster.max_corner.x(), cluster.centroid.x());
  EXPECT_LE(cluster.min_corner.z(), cluster.centroid.z());
  EXPECT_GE(cluster.max_corner.z(), cluster.centroid.z());
}

TEST(LidarClustererTest, CloudWithoutGroundStillClustersObjects)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  AddBlock(cloud, 25.0, 0.0, 0.0, 1.8, 1.8, 1.5, 0.8);

  LidarClusterer clusterer;
  const std::vector<Cluster> clusters = clusterer.ExtractClusters(cloud);
  EXPECT_EQ(1U, clusters.size());
}

}  // namespace
}  // namespace sil_sensor_fusion
