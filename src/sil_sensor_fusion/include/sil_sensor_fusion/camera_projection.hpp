// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_SENSOR_FUSION__CAMERA_PROJECTION_HPP_
#define SIL_SENSOR_FUSION__CAMERA_PROJECTION_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <string>

namespace sil_sensor_fusion
{

/// Pinhole intrinsics for the forward-facing camera.
struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::uint32_t image_width{0};
  std::uint32_t image_height{0};

  /// True when the focal lengths are usable for projection.
  bool valid() const {return fx > 1e-6 && fy > 1e-6;}
};

/// Axis-aligned 2D bounding box in pixel coordinates.
struct BoundingBox2D
{
  std::uint32_t x_offset{0};
  std::uint32_t y_offset{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string class_label{"unknown"};

  /// Bounding box centre in pixels.
  Eigen::Vector2d Center() const
  {
    return Eigen::Vector2d(
      static_cast<double>(x_offset) + static_cast<double>(width) / 2.0,
      static_cast<double>(y_offset) + static_cast<double>(height) / 2.0);
  }
};

/// Bearing/range observation derived from a 2D detection.
struct CameraObservation
{
  double bearing{0.0};
  /// Range implied by the pinhole model and the known object height [m];
  /// <= 0 when the class height is unknown and no range can be inferred.
  double range{0.0};
  double bearing_sigma{0.0};
  double range_sigma{0.0};
  std::string class_label{"unknown"};
  bool range_valid{false};
};

/// Converts a pixel column into a bearing around the optical axis [rad].
///
/// Uses the pinhole model with the principal point removed:
///   bearing = atan((u - cx) / fx)
double PixelToBearing(double u_pixel, const CameraIntrinsics & intrinsics);

/// Known physical height of an object class [m]; 0 when unknown.
double ClassHeight(const std::string & class_label);

/// Estimates range from the apparent size of a detection.
///
/// Range is recovered from the pinhole projection of a known object height:
///   Z = (f_y * H_object) / h_pixels
/// The result is clamped to @p min_range .. @p max_range.
double EstimateRangeFromHeight(
  const BoundingBox2D & box, const CameraIntrinsics & intrinsics,
  double min_range = 2.0, double max_range = 120.0);

/// Builds a full camera observation for @p box.
///
/// The bearing sigma grows with the angular offset from the optical axis to
/// reflect the weaker calibration at the image edges.
CameraObservation MakeCameraObservation(
  const BoundingBox2D & box, const CameraIntrinsics & intrinsics,
  double base_bearing_sigma = 0.02);

}  // namespace sil_sensor_fusion

#endif  // SIL_SENSOR_FUSION__CAMERA_PROJECTION_HPP_
