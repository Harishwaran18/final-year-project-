// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "sil_sensor_fusion/multi_target_tracker.hpp"

namespace sil_sensor_fusion
{
namespace
{

PolarObservation Lidar(double range, double bearing, double timestamp)
{
  PolarObservation observation;
  observation.range = range;
  observation.bearing = bearing;
  observation.timestamp = timestamp;
  observation.modality = PolarObservation::Modality::kLidar;
  return observation;
}

PolarObservation Camera(double range, double bearing, double timestamp,
  double range_sigma = 2.0, double bearing_sigma = 0.05)
{
  PolarObservation observation;
  observation.range = range;
  observation.bearing = bearing;
  observation.timestamp = timestamp;
  observation.modality = PolarObservation::Modality::kCamera;
  observation.range_sigma = range_sigma;
  observation.bearing_sigma = bearing_sigma;
  return observation;
}

/// Feeds @p count cycles of a single stationary observation and returns the
/// tracker so the caller can inspect the resulting tracks.
void FeedStationary(MultiTargetTracker & tracker, int count, double range,
  double bearing, double dt = 0.1)
{
  double timestamp = 0.0;
  for (int step = 0; step < count; ++step) {
    timestamp += dt;
    tracker.Process({Lidar(range, bearing, timestamp)}, timestamp);
  }
}

TEST(MultiTargetTrackerTest, StartsEmpty)
{
  MultiTargetTracker tracker;
  EXPECT_TRUE(tracker.tracks().empty());
  EXPECT_TRUE(tracker.ConfirmedTracks().empty());
}

TEST(MultiTargetTrackerTest, FirstObservationBirthsATentativeTrack)
{
  MultiTargetTracker tracker;
  const TrackerCycleStats stats = tracker.Process({Lidar(20.0, 0.0, 0.1)}, 0.1);

  EXPECT_EQ(1U, stats.births);
  EXPECT_EQ(0U, stats.updates);
  EXPECT_EQ(1U, stats.active_tracks);
  ASSERT_EQ(1U, tracker.tracks().size());
  EXPECT_EQ(TrackStatus::kTentative, tracker.tracks().front().status);
}

TEST(MultiTargetTrackerTest, TrackIsConfirmedAfterTheConfiguredHits)
{
  MultiTargetTracker tracker;
  FeedStationary(tracker, 3, 20.0, 0.0);

  ASSERT_EQ(1U, tracker.tracks().size());
  EXPECT_EQ(TrackStatus::kConfirmed, tracker.tracks().front().status);
  EXPECT_EQ(1U, tracker.ConfirmedTracks().size());
}

TEST(MultiTargetTrackerTest, ConfirmHitsIsRespected)
{
  TrackerConfig config;
  config.confirm_hits = 5;
  MultiTargetTracker tracker(config);

  FeedStationary(tracker, 4, 20.0, 0.0);
  ASSERT_EQ(1U, tracker.tracks().size());
  EXPECT_EQ(TrackStatus::kTentative, tracker.tracks().front().status);

  FeedStationary(tracker, 2, 20.0, 0.0);
  ASSERT_EQ(1U, tracker.tracks().size());
  EXPECT_EQ(TrackStatus::kConfirmed, tracker.tracks().front().status);
}

TEST(MultiTargetTrackerTest, TrackIdsAreMonotonic)
{
  MultiTargetTracker tracker;
  double timestamp = 0.0;

  tracker.Process({Lidar(20.0, 0.0, 0.1)}, 0.1);
  tracker.Process({Lidar(20.0, 0.0, 0.2), Lidar(40.0, 0.3, 0.2)}, 0.2);
  tracker.Process({Lidar(20.0, 0.0, 0.3), Lidar(40.0, 0.3, 0.3),
    Lidar(60.0, -0.3, 0.3)}, 0.3);

  std::uint32_t previous = 0;
  for (const auto & track : tracker.tracks()) {
    EXPECT_GT(track.id, previous);
    previous = track.id;
  }
  static_cast<void>(timestamp);
}

TEST(MultiTargetTrackerTest, DistinctTargetsGetDistinctTracks)
{
  MultiTargetTracker tracker;
  const std::vector<PolarObservation> observations{
    Lidar(20.0, 0.0, 0.1),
    Lidar(40.0, 0.4, 0.1),
    Lidar(60.0, -0.4, 0.1),
  };

  const TrackerCycleStats stats = tracker.Process(observations, 0.1);
  EXPECT_EQ(3U, stats.births);
  EXPECT_EQ(3U, stats.active_tracks);
}

TEST(MultiTargetTrackerTest, ConsistentObservationsUpdateRatherThanRebirth)
{
  MultiTargetTracker tracker;
  FeedStationary(tracker, 5, 25.0, 0.1);

  EXPECT_EQ(1U, tracker.tracks().size()) << "the target must not be re-born each cycle";
  EXPECT_EQ(5U, tracker.tracks().front().hits);
  EXPECT_EQ(0U, tracker.tracks().front().misses);
}

TEST(MultiTargetTrackerTest, TentativeTrackDiesAfterASingleMiss)
{
  MultiTargetTracker tracker;
  tracker.Process({Lidar(20.0, 0.0, 0.1)}, 0.1);
  ASSERT_EQ(1U, tracker.tracks().size());

  // Nothing observed, so the tentative track must be dropped immediately.
  const TrackerCycleStats stats = tracker.Process({}, 0.2);
  EXPECT_EQ(1U, stats.deletions);
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST(MultiTargetTrackerTest, ConfirmedTrackCoastsForTheConfiguredCycles)
{
  TrackerConfig config;
  config.max_coasted_cycles = 3;
  MultiTargetTracker tracker(config);

  FeedStationary(tracker, 3, 20.0, 0.0);
  ASSERT_EQ(TrackStatus::kConfirmed, tracker.tracks().front().status);

  // Three empty cycles must not delete a confirmed track.
  double timestamp = 0.3;
  for (int step = 0; step < 3; ++step) {
    timestamp += 0.1;
    tracker.Process({}, timestamp);
    ASSERT_EQ(1U, tracker.tracks().size()) << "deleted too early at step " << step;
    EXPECT_EQ(TrackStatus::kConfirmed, tracker.tracks().front().status);
  }

  // The fourth miss exceeds the limit.
  timestamp += 0.1;
  const TrackerCycleStats stats = tracker.Process({}, timestamp);
  EXPECT_EQ(1U, stats.deletions);
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST(MultiTargetTrackerTest, ReacquiringATrackResetsTheMissCounter)
{
  MultiTargetTracker tracker;
  FeedStationary(tracker, 3, 20.0, 0.0);
  tracker.Process({}, 0.4);
  ASSERT_EQ(1U, tracker.tracks().size());
  EXPECT_EQ(1U, tracker.tracks().front().misses);

  tracker.Process({Lidar(20.0, 0.0, 0.5)}, 0.5);
  ASSERT_EQ(1U, tracker.tracks().size());
  EXPECT_EQ(0U, tracker.tracks().front().misses);
}

TEST(MultiTargetTrackerTest, CameraObservationMarksATrackAsMultimodal)
{
  MultiTargetTracker tracker;
  double timestamp = 0.0;
  for (int step = 0; step < 4; ++step) {
    timestamp += 0.1;
    tracker.Process({Lidar(30.0, 0.0, timestamp)}, timestamp);
  }
  ASSERT_EQ(1U, tracker.tracks().size());
  EXPECT_TRUE(tracker.tracks().front().seen_by_lidar);
  EXPECT_FALSE(tracker.tracks().front().seen_by_camera);

  // A camera observation close enough to associate must add the camera flag.
  timestamp += 0.1;
  tracker.Process({Camera(30.5, 0.0, timestamp)}, timestamp);

  ASSERT_EQ(1U, tracker.tracks().size()) << "the camera must not spawn a second track";
  EXPECT_TRUE(tracker.tracks().front().seen_by_camera);
}

TEST(MultiTargetTrackerTest, CameraObservationUsesItsWiderNoise)
{
  MultiTargetTracker tracker;
  double timestamp = 0.0;
  for (int step = 0; step < 4; ++step) {
    timestamp += 0.1;
    tracker.Process({Lidar(30.0, 0.0, timestamp)}, timestamp);
  }

  // A 6 m range disagreement is far outside the LiDAR gate but inside the
  // camera gate, so it must be associated thanks to the override.
  timestamp += 0.1;
  const TrackerCycleStats stats = tracker.Process(
    {Camera(36.0, 0.0, timestamp, 6.0, 0.1)}, timestamp);

  EXPECT_EQ(0U, stats.births);
  EXPECT_EQ(1U, stats.updates);
  EXPECT_EQ(1U, tracker.tracks().size());
}

TEST(MultiTargetTrackerTest, UnrelatedCameraObservationSpawnsItsOwnTrack)
{
  MultiTargetTracker tracker;
  double timestamp = 0.0;
  for (int step = 0; step < 4; ++step) {
    timestamp += 0.1;
    tracker.Process({Lidar(30.0, 0.0, timestamp)}, timestamp);
  }

  // 30 m away laterally with a tight camera sigma: a different object.
  timestamp += 0.1;
  tracker.Process({Camera(30.0, 0.5, timestamp, 0.5, 0.01)}, timestamp);

  EXPECT_EQ(2U, tracker.tracks().size());
}

TEST(MultiTargetTrackerTest, EachObservationIsUsedAtMostOnce)
{
  MultiTargetTracker tracker;
  // Two identical observations against a fresh tracker: one track is born and
  // the second observation must not double-update it.
  const TrackerCycleStats stats = tracker.Process(
    {Lidar(20.0, 0.0, 0.1), Lidar(20.0, 0.0, 0.1)}, 0.1);

  EXPECT_EQ(2U, stats.births);
  EXPECT_EQ(2U, stats.active_tracks);
}

TEST(MultiTargetTrackerTest, EmptyCycleWithNoTracksIsANoop)
{
  MultiTargetTracker tracker;
  const TrackerCycleStats stats = tracker.Process({}, 0.1);

  EXPECT_EQ(0U, stats.births);
  EXPECT_EQ(0U, stats.updates);
  EXPECT_EQ(0U, stats.deletions);
  EXPECT_EQ(0U, stats.active_tracks);
}

TEST(MultiTargetTrackerTest, OutputIsDeterministicAcrossIdenticalRuns)
{
  const std::vector<PolarObservation> observations{
    Lidar(20.0, 0.0, 0.1),
    Lidar(45.0, 0.3, 0.1),
    Lidar(70.0, -0.2, 0.1),
  };

  MultiTargetTracker first;
  MultiTargetTracker second;
  double timestamp = 0.0;
  for (int step = 0; step < 10; ++step) {
    timestamp += 0.1;
    std::vector<PolarObservation> cycle = observations;
    for (auto & observation : cycle) {
      observation.timestamp = timestamp;
    }
    first.Process(cycle, timestamp);
    second.Process(cycle, timestamp);
  }

  ASSERT_EQ(first.tracks().size(), second.tracks().size());
  for (std::size_t i = 0; i < first.tracks().size(); ++i) {
    EXPECT_EQ(first.tracks()[i].id, second.tracks()[i].id);
    EXPECT_EQ(first.tracks()[i].hits, second.tracks()[i].hits);
    EXPECT_EQ(first.tracks()[i].filter.state(), second.tracks()[i].filter.state());
  }
}

TEST(MultiTargetTrackerTest, ResetClearsTracksAndIds)
{
  MultiTargetTracker tracker;
  FeedStationary(tracker, 3, 20.0, 0.0);
  ASSERT_FALSE(tracker.tracks().empty());

  tracker.Reset();
  EXPECT_TRUE(tracker.tracks().empty());

  // Ids restart, so a replayed scenario is byte-for-byte reproducible.
  tracker.Process({Lidar(20.0, 0.0, 0.1)}, 0.1);
  ASSERT_EQ(1U, tracker.tracks().size());
  EXPECT_EQ(1U, tracker.tracks().front().id);
}

TEST(MultiTargetTrackerTest, TracksAConstantVelocityTarget)
{
  MultiTargetTracker tracker;
  // A target approaching at 20 m/s along the ego x-axis, sampled at 20 Hz.
  double timestamp = 0.0;
  double range = 60.0;
  for (int step = 0; step < 40; ++step) {
    timestamp += 0.05;
    range -= 1.0;  // 20 m/s
    tracker.Process({Lidar(range, 0.0, timestamp)}, timestamp);
  }

  const std::vector<Track> confirmed = tracker.ConfirmedTracks();
  ASSERT_FALSE(confirmed.empty());
  // ClosingSpeed is positive when approaching; the estimate must be in the
  // right ballpark (within 20% of 20 m/s).
  EXPECT_NEAR(20.0, confirmed.front().filter.ClosingSpeed(), 4.0);
}

TEST(MultiTargetTrackerTest, CloselySpacedTargetsStaySeparate)
{
  MultiTargetTracker tracker;
  double timestamp = 0.0;
  for (int step = 0; step < 6; ++step) {
    timestamp += 0.1;
    tracker.Process({Lidar(30.0, 0.0, timestamp), Lidar(60.0, 0.0, timestamp)},
      timestamp);
  }

  EXPECT_EQ(2U, tracker.tracks().size());
  EXPECT_EQ(2U, tracker.ConfirmedTracks().size());
}

TEST(MultiTargetTrackerTest, StatsReportRejections)
{
  MultiTargetTracker tracker;
  // Seed a track, then present a wildly inconsistent LiDAR observation. It will
  // fail the association gate, so it becomes a birth rather than a rejection.
  FeedStationary(tracker, 3, 20.0, 0.0);
  const TrackerCycleStats stats = tracker.Process({Lidar(20.0, 0.0, 0.4)}, 0.4);

  EXPECT_EQ(1U, stats.updates);
  EXPECT_EQ(0U, stats.rejected);
  EXPECT_EQ(1U, stats.active_tracks);
}

}  // namespace
}  // namespace sil_sensor_fusion
