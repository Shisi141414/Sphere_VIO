#pragma once

#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sphere_vio/camera/camera_rig.hpp"
#include "sphere_vio/common/types.hpp"
#include "sphere_vio/frontend/landmark_track.hpp"

namespace sphere_vio {

struct MsckfOptions {
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  // Scalar gravity magnitude mirrors `gravity`; configuration files use this
  // value and the loader writes it into gravity.z().
  double gravity_magnitude = 9.81;
  double gyroscope_noise = 1.0e-2;
  double accelerometer_noise = 1.0e-1;
  double gyroscope_bias_noise = 1.0e-4;
  double accelerometer_bias_noise = 1.0e-3;
  double pixel_noise = 1.5;
  double feature_chi_square_probability = 0.95;
  double time_offset_cam_imu = 0.0;
  Eigen::Vector3d extrinsic_rotation_perturbation =
      Eigen::Vector3d::Zero();
  Eigen::Vector3d extrinsic_translation_perturbation =
      Eigen::Vector3d::Zero();
  double initialization_duration = 1.0;
  std::size_t minimum_initialization_samples = 20U;
  int maximum_clones = 20;
  int maximum_landmarks = 40;
  std::size_t maximum_feature_observations = 40U;
  std::uint64_t maximum_frames_without_observation = 5U;
  Eigen::Matrix<double, 15, 15> initial_covariance =
      Eigen::Matrix<double, 15, 15>::Identity();
};

struct MsckfCurrentState {
  Timestamp timestamp = 0.0;
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_wb = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_wb = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
};

struct MsckfClone {
  Timestamp timestamp = 0.0;
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_wb = Eigen::Vector3d::Zero();
  int index = 0;
};

struct MsckfObservation {
  Timestamp timestamp = 0.0;
  CameraId camera_id = 0U;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
};

struct MsckfFeature {
  std::uint64_t id = 0U;
  std::uint64_t persistent_id = 0U;
  std::vector<MsckfObservation> observations;
};

// Accumulates current-frame pixel observations for every active temporal
// feature. The runner uses this to feed monocular and multi-camera features to
// MSCKF instead of relying only on cross-camera stereo LandmarkTracks.
class MsckfFeatureAccumulator {
 public:
  explicit MsckfFeatureAccumulator(
      std::size_t maximum_observations = 40U);

  void add(CameraId camera_id, FeatureId feature_id, Timestamp timestamp,
           const Eigen::Vector2d& pixel, std::uint64_t frame_index);
  void prune(std::uint64_t current_frame_index,
             std::uint64_t maximum_frames_without_observation);
  const std::vector<MsckfObservation>* observations(
      CameraId camera_id, FeatureId feature_id) const;
  std::vector<TemporalFeatureKey> keys() const;

 private:
  struct TrackState {
    std::uint64_t last_frame_index = 0U;
    std::vector<MsckfObservation> observations;
  };

  std::size_t maximum_observations_;
  std::map<TemporalFeatureKey, TrackState> tracks_;
};

// Sliding-window MSCKF backend. The current IMU state occupies the first 15
// covariance entries and each clone adds six error-state entries
// [theta, position].
class Msckf {
 public:
  explicit Msckf(MsckfOptions options = {});

  bool initialized() const { return initialized_; }
  const MsckfCurrentState& state() const { return state_; }
  const Eigen::MatrixXd& covariance() const { return covariance_; }
  std::size_t cloneCount() const { return clones_.size(); }
  std::size_t landmarkCount() const { return landmark_positions_.size(); }

  bool initialize(const ImuMeasurement& measurement);
  // Accumulates an IMU window and initializes gravity direction, gyro bias,
  // and accelerometer bias once the window is long enough. Returns false until
  // initialization completes.
  bool initialize(const std::vector<ImuMeasurement>& measurements,
                  Timestamp end_time);
  bool propagate(const std::vector<ImuMeasurement>& measurements,
                 Timestamp end_time);
  bool augmentClone(Timestamp timestamp);
  bool update(const std::vector<MsckfFeature>& features,
              const CameraRig& camera_rig);
  bool estimateTimeOffset(const std::vector<MsckfFeature>& features,
                          const CameraRig& camera_rig);
  bool estimateExtrinsicPerturbation(
      const std::vector<MsckfFeature>& features,
      const CameraRig& camera_rig);
  bool augmentLandmark(std::uint64_t persistent_id,
                       const Eigen::Vector3d& point_w);
  void marginalizeOldestClone();

 private:
  struct FeatureMeasurement {
    Timestamp timestamp = 0.0;
    CameraId camera_id = 0U;
    Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
    Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
    int clone_index = 0;
  };

  bool buildFeatureMeasurements(
      const MsckfFeature& feature, const CameraRig& camera_rig,
      Eigen::Vector3d* point_w,
      std::vector<FeatureMeasurement>* measurements);
  bool estimateFeaturePosition(
      const std::vector<FeatureMeasurement>& measurements,
      const CameraRig& camera_rig, const Eigen::Vector3d* prior,
      Eigen::Vector3d* point_w) const;
  void propagateSegment(const ImuMeasurement& minus,
                        const ImuMeasurement& plus);
  bool applyErrorState(const Eigen::VectorXd& correction);
  bool initializeStatic(Timestamp end_time);
  bool computeProjectionAndJacobians(
      const CameraRig& rig, const MsckfClone& clone,
      const FeatureMeasurement& measurement, Eigen::Vector2d* predicted,
      Eigen::Matrix<double, 2, 3>* jacobian_feature,
      Eigen::Matrix<double, 2, 6>* jacobian_clone) const;
  double featureReprojectionResidual(const MsckfFeature& feature,
                                     double time_offset,
                                     const CameraRig& camera_rig) const;

  MsckfOptions options_;
  MsckfCurrentState state_;
  Eigen::MatrixXd covariance_;
  std::map<Timestamp, MsckfClone> clones_;
  std::map<std::uint64_t, Eigen::Vector3d> feature_positions_;
  std::map<std::uint64_t, int> landmark_indices_;
  std::map<std::uint64_t, Eigen::Vector3d> landmark_positions_;
  std::vector<ImuMeasurement> initialization_buffer_;
  bool initialized_ = false;
};

}  // namespace sphere_vio
