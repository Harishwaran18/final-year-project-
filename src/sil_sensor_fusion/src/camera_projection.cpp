// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_sensor_fusion/camera_projection.hpp"

#include <algorithm>
#include <cmath>

namespace sil_sensor_fusion
{
namespace
{
/// Height of the edge of the image is where the pinhole model loses accuracy.
constexpr double kEdgeSigmaGrowth = 1.8;
}  // namespace

double PixelToBearing(double u_pixel, const CameraIntrinsics & intrinsics)
{
  if (!intrinsics.valid()) {
    return 0.0;
  }
  return std::atan((u_pixel - intrinsics.cx) / intrinsics.fx);
}

double ClassHeight(const std::string & class_label)
{
  // Approximate real-world heights of the classes emitted by the CARLA hook.
  if (class_label == "car") {
    return 1.5;
  }
  if (class_label == "truck" || class_label == "bus") {
    return 3.2;
  }
  if (class_label == "motorcycle" || class_label == "bicycle") {
    return 1.2;
  }
  if (class_label == "pedestrian") {
    return 1.75;
  }
  if (class_label == "traffic_light" || class_label == "traffic_sign") {
    return 0.8;
  }
  return 0.0;
}

double EstimateRangeFromHeight(
  const BoundingBox2D & box, const CameraIntrinsics & intrinsics,
  double min_range, double max_range)
{
  if (!intrinsics.valid() || box.height == 0) {
    return 0.0;
  }
  const double object_height = ClassHeight(box.class_label);
  if (object_height <= 0.0) {
    return 0.0;
  }
  const double range =
    (intrinsics.fy * object_height) / static_cast<double>(box.height);
  return std::clamp(range, min_range, max_range);
}

CameraObservation MakeCameraObservation(
  const BoundingBox2D & box, const CameraIntrinsics & intrinsics,
  double base_bearing_sigma)
{
  CameraObservation observation;
  observation.class_label = box.class_label;
  observation.bearing = PixelToBearing(box.Center().x(), intrinsics);
  observation.bearing_sigma = base_bearing_sigma;

  if (intrinsics.valid() && intrinsics.image_width > 0) {
    // Detections far from the principal point have larger bearing error.
    const double half_width = static_cast<double>(intrinsics.image_width) / 2.0;
    const double offset_ratio = std::abs(box.Center().x() - intrinsics.cx) /
      std::max(1.0, half_width);
    observation.bearing_sigma = base_bearing_sigma *
      (1.0 + kEdgeSigmaGrowth * std::clamp(offset_ratio, 0.0, 1.0));
  }

  observation.range = EstimateRangeFromHeight(box, intrinsics);
  observation.range_valid = observation.range > 0.0;
  observation.range_sigma = observation.range_valid ?
    std::max(0.5, observation.range * 0.15) : 0.0;
  return observation;
}

}  // namespace sil_sensor_fusion
