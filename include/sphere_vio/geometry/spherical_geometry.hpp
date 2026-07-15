#pragma once

#include <Eigen/Core>

namespace sphere_vio {

// Wraps any finite longitude to [-pi, pi). In particular, +pi maps to -pi.
bool wrapLongitude(double longitude, double* wrapped_longitude);

// Returns wrap(longitude_a - longitude_b) in [-pi, pi).
bool wrappedLongitudeDifference(double longitude_a, double longitude_b,
                                double* difference);

bool bearingToLongitudeLatitude(
    const Eigen::Vector3d& bearing_b,
    Eigen::Vector2d* longitude_latitude);
bool longitudeLatitudeToBearing(
    const Eigen::Vector2d& longitude_latitude,
    Eigen::Vector3d* bearing_b);

// ERP coordinates are continuous geometric coordinates, not integer image
// indices. Horizontal coordinates are periodic in [0, width), while vertical
// coordinates include both pole boundaries in [0, height].
bool bearingToEquirectangular(const Eigen::Vector3d& bearing_b, int width,
                              int height, Eigen::Vector2d* erp_coordinate);
bool equirectangularToBearing(const Eigen::Vector2d& erp_coordinate,
                              int width, int height,
                              Eigen::Vector3d* bearing_b);

bool angularDistance(const Eigen::Vector3d& bearing_a,
                     const Eigen::Vector3d& bearing_b, double* angle);

}  // namespace sphere_vio
