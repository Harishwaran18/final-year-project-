// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

#include "sil_sensor_fusion/lidar_clusterer.hpp"
#include "sil_sensor_fusion/multi_target_tracker.hpp"

namespace sil_sensor_fusion
{
namespace
{

/// Budget for one fusion cycle, from the requirements [ms].
constexpr double kLatencyBudgetMs = 18.0;

/// Number of cycles averaged for the latency measurement. The first cycles are
/// discarded so that PCL's allocator warm-up does not pollute the statistic.
constexpr int kWarmupCycles = 5;
constexpr int kMeasuredCycles = 30;

/// Builds a 32-beam-like sweep with ground plus @p object_count vehicles.
pcl::PointCloud<pcl::PointXYZI> MakeSweep(std::size_t object_count)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;

  // Ground: a 60 x 40 m grid at z = -1.5 m.
  for (int x = 0; x < 60; ++x) {
    for (int y = -20; y <= 20; ++y) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y) * 0.5F;
      point.z = -1.5F;
      point.intensity = 0.1F;
      cloud.push_back(point);
    }
  }

  // Objects spread over the forward field of view.
  for (std::size_t index = 0; index < object_count; ++index) {
    const double cx = 10.0 + static_cast<double>(index) * 6.0;
    const double cy = (index % 2 == 0 ? 1.0 : -1.0) * 1.5;
    for (int i = 0; i < 8; ++i) {
      for (int j = 0; j < 8; ++j) {
        for (int k = 0; k < 6; ++k) {
          pcl::PointXYZI point;
          point.x = static_cast<float>(cx - 0.9 + 1.8 * i / 7.0);
          point.y = static_cast<float>(cy - 0.9 + 1.8 * j / 7.0);
          point.z = static_cast<float>(-1.5 + 0.2 + 1.5 * k / 5.0);
          point.intensity = 0.8F;
          cloud.push_back(point);
        }
      }
    }
  }
  return cloud;
}

TEST(FusionLatencyTest, ClusteringStaysWithinTheLatencyBudget)
{
  const pcl::PointCloud<pcl::PointXYZI> sweep = MakeSweep(6);
  LidarClusterer clusterer;

  // Warm-up.
  for (int cycle = 0; cycle < kWarmupCycles; ++cycle) {
    clusterer.ExtractClusters(sweep);
  }

  double total_ms = 0.0;
  for (int cycle = 0; cycle < kMeasuredCycles; ++cycle) {
    const auto start = std::chrono::steady_clock::now();
    clusterer.ExtractClusters(sweep);
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start);
    total_ms += elapsed.count();
  }

  const double mean_ms = total_ms / kMeasuredCycles;
  EXPECT_LT(mean_ms, kLatencyBudgetMs)
    << "mean clustering latency " << mean_ms << " ms exceeds the budget";
}

TEST(FusionLatencyTest, FullCycleStaysWithinTheLatencyBudget)
{
  const pcl::PointCloud<pcl::PointXYZI> sweep = MakeSweep(6);
  LidarClusterer clusterer;
  MultiTargetTracker tracker;

  auto run_cycle = [&](double timestamp) {
    const std::vector<Cluster> clusters = clusterer.ExtractClusters(sweep);
    std::vector<PolarObservation> observations;
    observations.reserve(clusters.size());
    for (const auto & cluster : clusters) {
      PolarObservation observation;
      observation.range = cluster.Range();
      observation.bearing = cluster.Bearing();
      observation.timestamp = timestamp;
      observation.modality = PolarObservation::Modality::kLidar;
      observations.push_back(observation);
    }
    tracker.Process(observations, timestamp);
  };

  double timestamp = 0.0;
  for (int cycle = 0; cycle < kWarmupCycles; ++cycle) {
    timestamp += 0.05;
    run_cycle(timestamp);
  }

  double total_ms = 0.0;
  for (int cycle = 0; cycle < kMeasuredCycles; ++cycle) {
    timestamp += 0.05;
    const auto start = std::chrono::steady_clock::now();
    run_cycle(timestamp);
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start);
    total_ms += elapsed.count();
  }

  const double mean_ms = total_ms / kMeasuredCycles;
  EXPECT_LT(mean_ms, kLatencyBudgetMs)
    << "mean fusion latency " << mean_ms << " ms exceeds the 18 ms budget";
}

TEST(FusionLatencyTest, TrackerProducesTracksFromASweepSequence)
{
  // Sanity check that the pipeline actually converges, not just that it is fast.
  const pcl::PointCloud<pcl::PointXYZI> sweep = MakeSweep(4);
  LidarClusterer clusterer;
  MultiTargetTracker tracker;

  double timestamp = 0.0;
  for (int cycle = 0; cycle < 20; ++cycle) {
    timestamp += 0.05;
    const std::vector<Cluster> clusters = clusterer.ExtractClusters(sweep);
    std::vector<PolarObservation> observations;
    for (const auto & cluster : clusters) {
      PolarObservation observation;
      observation.range = cluster.Range();
      observation.bearing = cluster.Bearing();
      observation.timestamp = timestamp;
      observations.push_back(observation);
    }
    tracker.Process(observations, timestamp);
  }

  EXPECT_GE(tracker.ConfirmedTracks().size(), 2U)
    << "the pipeline failed to confirm the synthetic objects";
}

TEST(FusionLatencyTest, TrackerStepIsFastWithManyTracks)
{
  // 40 targets is well beyond the AEB-relevant count; the cycle must still fit.
  MultiTargetTracker tracker;

  std::vector<PolarObservation> observations;
  for (int index = 0; index < 40; ++index) {
    PolarObservation observation;
    observation.range = 10.0 + static_cast<double>(index) * 1.5;
    observation.bearing = static_cast<double>(index % 7 - 3) * 0.05;
    observation.timestamp = 0.05;
    observations.push_back(observation);
  }

  double timestamp = 0.0;
  for (int cycle = 0; cycle < 6; ++cycle) {
    timestamp += 0.05;
    for (auto & observation : observations) {
      observation.timestamp = timestamp;
    }
    tracker.Process(observations, timestamp);
  }

  double total_ms = 0.0;
  constexpr int kMeasured = 30;
  for (int cycle = 0; cycle < kMeasured; ++cycle) {
    timestamp += 0.05;
    for (auto & observation : observations) {
      observation.timestamp = timestamp;
    }
    const auto start = std::chrono::steady_clock::now();
    tracker.Process(observations, timestamp);
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start);
    total_ms += elapsed.count();
  }

  EXPECT_LT(total_ms / kMeasured, kLatencyBudgetMs);
}

}  // namespace
}  // namespace sil_sensor_fusion
