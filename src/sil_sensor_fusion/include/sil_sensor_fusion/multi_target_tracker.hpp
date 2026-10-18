// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_SENSOR_FUSION__MULTI_TARGET_TRACKER_HPP_
#define SIL_SENSOR_FUSION__MULTI_TARGET_TRACKER_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

#include "sil_sensor_fusion/ekf_tracker.hpp"

namespace sil_sensor_fusion
{

/// A polar observation handed to the tracker.
///
/// LiDAR clusters provide a range and bearing only. Camera detections provide a
/// bearing (from the pixel-to-bearing projection) and are fused as bearing-only
/// observations against a range predicted from the vision bounding box height.
struct PolarObservation
{
  double range{0.0};
  double bearing{0.0};
  double timestamp{0.0};
  /// Source modality; used for the multimodal confirmation flag.
  enum class Modality : std::uint8_t { kLidar, kCamera };
  Modality modality{Modality::kLidar};
  /// Range 1-sigma override [m]; <= 0 uses the filter default.
  double range_sigma{0.0};
  /// Bearing 1-sigma override [rad]; <= 0 uses the filter default.
  double bearing_sigma{0.0};
};

/// Lifecycle state of a track.
enum class TrackStatus : std::uint8_t
{
  kTentative = 0,
  kConfirmed = 1,
  kDeleted = 2,
};

/// One tracked object with its EKF and lifecycle counters.
struct Track
{
  std::uint32_t id{0};
  EkfTracker filter;
  TrackStatus status{TrackStatus::kTentative};
  std::uint32_t hits{0};
  std::uint32_t misses{0};
  bool seen_by_lidar{false};
  bool seen_by_camera{false};
  double last_update_time{0.0};
  double mahalanobis{0.0};
  std::string class_label{"unknown"};
};

/// Tuning for track birth, confirmation and deletion.
struct TrackerConfig
{
  /// Hits required before a tentative track is published.
  std::uint32_t confirm_hits{3};
  /// Consecutive misses tolerated before a confirmed track is deleted.
  std::uint32_t max_coasted_cycles{4};
  /// Association gate in squared Mahalanobis distance.
  double association_gate{9.2103};
  /// Positional gate [m] applied to tracks with no cross-modality support.
  double max_association_distance{6.0};
};

/// Statistics for one tracker cycle, useful for SIL instrumentation.
struct TrackerCycleStats
{
  std::uint32_t births{0};
  std::uint32_t updates{0};
  std::uint32_t rejected{0};
  std::uint32_t deletions{0};
  std::uint32_t active_tracks{0};
};

/// Greedy nearest-neighbour multi-target tracker over constant-velocity EKFs.
///
/// Association is deliberately simple and deterministic: cost is the squared
/// Mahalanobis distance, ties break on the lower track id, and each
/// observation/track pair is used at most once. This keeps SIL replays
/// bit-reproducible without a Hungarian solver dependency.
class MultiTargetTracker
{
public:
  explicit MultiTargetTracker(const TrackerConfig & config = TrackerConfig{},
    const EkfConfig & ekf_config = EkfConfig{});

  /// Runs one full predict/associate/update/delete cycle.
  ///
  /// @param observations all polar observations for this cycle (LiDAR and camera)
  /// @param timestamp    cycle time [s], must be monotonic
  /// @return per-cycle counters and the number of live tracks
  TrackerCycleStats Process(
    const std::vector<PolarObservation> & observations, double timestamp);

  /// Live (non-deleted) tracks after the last Process() call.
  const std::vector<Track> & tracks() const {return tracks_;}

  /// Only the confirmed tracks; this is what the fusion node publishes.
  std::vector<Track> ConfirmedTracks() const;

  /// Drops every track and resets the id counter (used between SIL scenarios).
  void Reset();

private:
  /// Creates a new track from an unassociated observation.
  Track CreateTrack(const PolarObservation & observation);

  /// Misses tolerated before a track in @p status is deleted. Zero for a
  /// tentative track, @ref TrackerConfig::max_coasted_cycles once confirmed.
  std::uint32_t CoastLimit(TrackStatus status) const;

  /// Builds the measurement covariance for @p observation, honouring any
  /// per-observation sigma overrides.
  MeasurementCovariance MakeMeasurementNoise(const PolarObservation & observation) const;

  TrackerConfig config_;
  EkfConfig ekf_config_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_{1};
};

}  // namespace sil_sensor_fusion

#endif  // SIL_SENSOR_FUSION__MULTI_TARGET_TRACKER_HPP_
