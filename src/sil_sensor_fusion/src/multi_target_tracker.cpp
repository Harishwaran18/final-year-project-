// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_sensor_fusion/multi_target_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sil_sensor_fusion
{

MultiTargetTracker::MultiTargetTracker(
  const TrackerConfig & config, const EkfConfig & ekf_config)
: config_(config), ekf_config_(ekf_config) {}

void MultiTargetTracker::Reset()
{
  tracks_.clear();
  next_id_ = 1;
}

Track MultiTargetTracker::CreateTrack(const PolarObservation & observation)
{
  Track track;
  track.id = next_id_++;
  track.filter = EkfTracker(ekf_config_);
  track.filter.Initialize(MeasurementVector(observation.range, observation.bearing),
    observation.timestamp);
  track.status = TrackStatus::kTentative;
  track.hits = 1;
  track.misses = 0;
  track.seen_by_lidar = observation.modality == PolarObservation::Modality::kLidar;
  track.seen_by_camera = observation.modality == PolarObservation::Modality::kCamera;
  track.last_update_time = observation.timestamp;
  return track;
}

std::uint32_t MultiTargetTracker::CoastLimit(TrackStatus status) const
{
  // A tentative track has not earned the right to coast: one miss is enough to
  // reject it as a spurious detection. A confirmed track survives the
  // configured number of misses before being deleted.
  return status == TrackStatus::kConfirmed ? config_.max_coasted_cycles : 0U;
}

MeasurementCovariance MultiTargetTracker::MakeMeasurementNoise(
  const PolarObservation & observation) const
{
  const double range_sigma = observation.range_sigma > 0.0 ?
    observation.range_sigma : ekf_config_.range_noise_std;
  const double bearing_sigma = observation.bearing_sigma > 0.0 ?
    observation.bearing_sigma : ekf_config_.bearing_noise_std;

  MeasurementCovariance noise = MeasurementCovariance::Zero();
  noise(0, 0) = range_sigma * range_sigma;
  noise(1, 1) = bearing_sigma * bearing_sigma;
  return noise;
}

TrackerCycleStats MultiTargetTracker::Process(
  const std::vector<PolarObservation> & observations, double timestamp)
{
  TrackerCycleStats stats;

  // --- 1. Predict every live track to the cycle timestamp. -------------------
  for (auto & track : tracks_) {
    if (track.status == TrackStatus::kDeleted) {
      continue;
    }
    track.filter.Predict(timestamp);
  }

  // --- 2. Build the association cost matrix. --------------------------------
  const std::size_t num_tracks = tracks_.size();
  const std::size_t num_obs = observations.size();

  struct Candidate
  {
    std::size_t track_index;
    std::size_t obs_index;
    double cost;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(num_tracks * num_obs);

  for (std::size_t t = 0; t < num_tracks; ++t) {
    if (tracks_[t].status == TrackStatus::kDeleted) {
      continue;
    }
    for (std::size_t o = 0; o < num_obs; ++o) {
      const PolarObservation & observation = observations[o];
      const MeasurementVector z(observation.range, observation.bearing);
      const MeasurementCovariance noise = MakeMeasurementNoise(observation);
      const double d2 = tracks_[t].filter.MahalanobisDistance(z, &noise);
      if (!std::isfinite(d2)) {
        continue;
      }
      const double positional =
        (tracks_[t].filter.state().head<2>() -
        Eigen::Vector2d(z(0) * std::cos(z(1)), z(0) * std::sin(z(1)))).norm();
      if (d2 > config_.association_gate && positional > config_.max_association_distance) {
        continue;
      }
      candidates.push_back(Candidate{t, o, d2});
    }
  }

  // Deterministic greedy assignment: cheapest cost first, id then index as tie
  // breakers so that identical inputs always produce identical track ids.
  std::sort(candidates.begin(), candidates.end(), [this](const Candidate & a,
    const Candidate & b) {
      if (a.cost != b.cost) {
        return a.cost < b.cost;
      }
      if (tracks_[a.track_index].id != tracks_[b.track_index].id) {
        return tracks_[a.track_index].id < tracks_[b.track_index].id;
      }
      return a.obs_index < b.obs_index;
    });

  std::vector<bool> track_used(num_tracks, false);
  std::vector<bool> obs_used(num_obs, false);

  for (const auto & candidate : candidates) {
    if (track_used[candidate.track_index] || obs_used[candidate.obs_index]) {
      continue;
    }
    Track & track = tracks_[candidate.track_index];
    const PolarObservation & observation = observations[candidate.obs_index];

    const MeasurementVector z(observation.range, observation.bearing);
    const MeasurementCovariance noise = MakeMeasurementNoise(observation);
    const EkfTracker::UpdateResult result = track.filter.Update(z, &noise);
    if (!result.accepted) {
      ++stats.rejected;
      continue;
    }

    track_used[candidate.track_index] = true;
    obs_used[candidate.obs_index] = true;

    track.hits++;
    track.misses = 0;
    track.last_update_time = observation.timestamp;
    track.mahalanobis = result.mahalanobis;
    track.seen_by_lidar |= observation.modality == PolarObservation::Modality::kLidar;
    track.seen_by_camera |= observation.modality == PolarObservation::Modality::kCamera;

    if (track.status == TrackStatus::kTentative && track.hits >= config_.confirm_hits) {
      track.status = TrackStatus::kConfirmed;
    }
    ++stats.updates;
  }

  // --- 3. Coast unmatched tracks and age them out. --------------------------
  for (std::size_t t = 0; t < num_tracks; ++t) {
    if (tracks_[t].status == TrackStatus::kDeleted || track_used[t]) {
      continue;
    }
    tracks_[t].misses++;
    if (tracks_[t].misses > CoastLimit(tracks_[t].status)) {
      tracks_[t].status = TrackStatus::kDeleted;
      ++stats.deletions;
    }
  }

  // --- 4. Birth new tracks from unmatched observations. --------------------
  for (std::size_t o = 0; o < num_obs; ++o) {
    if (obs_used[o]) {
      continue;
    }
    tracks_.push_back(CreateTrack(observations[o]));
    ++stats.births;
  }

  // --- 5. Compact the track list. ------------------------------------------
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(), [](const Track & track) {
      return track.status == TrackStatus::kDeleted;
    }),
    tracks_.end());

  stats.active_tracks = static_cast<std::uint32_t>(tracks_.size());
  return stats;
}

std::vector<Track> MultiTargetTracker::ConfirmedTracks() const
{
  std::vector<Track> confirmed;
  for (const auto & track : tracks_) {
    if (track.status == TrackStatus::kConfirmed) {
      confirmed.push_back(track);
    }
  }
  return confirmed;
}

}  // namespace sil_sensor_fusion
