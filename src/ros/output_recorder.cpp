#include "sphere_vio/ros/output_recorder.hpp"

#include <cerrno>
#include <cmath>
#include <sys/stat.h>
#include <sys/types.h>

#include <iomanip>

namespace sphere_vio {

namespace {

bool ensureDirectory(const std::string& path) {
  return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

}  // namespace

namespace {

EskfState interpolateState(const EskfState& first, const EskfState& second,
                           double ratio) {
  EskfState result = first;
  result.timestamp = first.timestamp + ratio * (second.timestamp - first.timestamp);
  result.p_wb = first.p_wb + ratio * (second.p_wb - first.p_wb);
  result.v_wb = first.v_wb + ratio * (second.v_wb - first.v_wb);
  result.bias_gyro = first.bias_gyro + ratio * (second.bias_gyro - first.bias_gyro);
  result.bias_accel = first.bias_accel + ratio * (second.bias_accel - first.bias_accel);
  result.q_wb = first.q_wb.slerp(ratio, second.q_wb).normalized();
  return result;
}

}  // namespace

bool OutputRecorder::open(const std::string& directory, double output_period) {
  close();
  if (directory.empty() || !ensureDirectory(directory) ||
      !std::isfinite(output_period) || output_period < 0.0) return false;
  output_period_ = output_period;

  odometry_file_.open(directory + "/odometry.csv");
  trajectory_file_.open(directory + "/trajectory.csv");
  landmarks_file_.open(directory + "/landmarks.csv");
  if (!odometry_file_.is_open() || !trajectory_file_.is_open() ||
      !landmarks_file_.is_open()) {
    close();
    return false;
  }

  odometry_file_ << "timestamp_camera,timestamp_imu,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,"
                    "bias_gyro_x,bias_gyro_y,bias_gyro_z,"
                    "bias_accel_x,bias_accel_y,bias_accel_z\n";
  trajectory_file_ << "timestamp,x,y,z,qx,qy,qz,qw\n";
  landmarks_file_ << "landmark_id,x,y,z\n";
  return true;
}

void OutputRecorder::record(
    Timestamp camera_timestamp, const EskfState& state,
    const std::vector<BackendLandmark>& landmarks) {
  if (!odometry_file_.is_open() || !std::isfinite(camera_timestamp) ||
      !std::isfinite(state.timestamp) || !state.p_wb.allFinite() ||
      !state.v_wb.allFinite() || !state.q_wb.coeffs().allFinite()) return;

  const auto write_pose = [this, &landmarks](Timestamp output_timestamp,
                                              const EskfState& output_state,
                                              bool write_landmarks) {
    odometry_file_ << std::fixed << std::setprecision(9)
                   << output_timestamp << ',' << output_state.timestamp << ','
                   << output_state.p_wb.x() << ',' << output_state.p_wb.y() << ','
                   << output_state.p_wb.z() << ','
                   << output_state.q_wb.w() << ',' << output_state.q_wb.x() << ','
                   << output_state.q_wb.y() << ',' << output_state.q_wb.z() << ','
                   << output_state.v_wb.x() << ',' << output_state.v_wb.y() << ','
                   << output_state.v_wb.z() << ','
                   << output_state.bias_gyro.x() << ',' << output_state.bias_gyro.y() << ','
                   << output_state.bias_gyro.z() << ','
                   << output_state.bias_accel.x() << ',' << output_state.bias_accel.y() << ','
                   << output_state.bias_accel.z() << '\n';

    if (trajectory_file_.is_open()) {
      trajectory_file_ << std::fixed << std::setprecision(9)
                       << output_timestamp << ','
                       << output_state.p_wb.x() << ',' << output_state.p_wb.y() << ','
                       << output_state.p_wb.z() << ','
                       << output_state.q_wb.x() << ',' << output_state.q_wb.y() << ','
                       << output_state.q_wb.z() << ',' << output_state.q_wb.w() << '\n';
    }
    if (write_landmarks && landmarks_file_.is_open()) {
      for (const BackendLandmark& landmark : landmarks) {
        landmarks_file_ << std::fixed << std::setprecision(9)
                        << landmark.track_id.value << ','
                        << landmark.point_w.x() << ',' << landmark.point_w.y()
                        << ',' << landmark.point_w.z() << '\n';
      }
    }
  };

  if (has_previous_state_ && camera_timestamp <= previous_camera_timestamp_) {
    return;
  }
  if (has_previous_state_ && output_period_ > 0.0) {
    const double camera_duration = camera_timestamp - previous_camera_timestamp_;
    while (next_regular_timestamp_ < camera_timestamp - 1e-9) {
      const double ratio = (next_regular_timestamp_ - previous_camera_timestamp_) /
                           camera_duration;
      write_pose(next_regular_timestamp_,
                 interpolateState(previous_state_, state, ratio), false);
      next_regular_timestamp_ += output_period_;
    }
  }
  write_pose(camera_timestamp, state, true);
  if (!has_previous_state_ && output_period_ > 0.0) {
    next_regular_timestamp_ = camera_timestamp + output_period_;
  }
  previous_camera_timestamp_ = camera_timestamp;
  previous_state_ = state;
  has_previous_state_ = true;
}

void OutputRecorder::close() {
  if (odometry_file_.is_open()) odometry_file_.close();
  if (trajectory_file_.is_open()) trajectory_file_.close();
  if (landmarks_file_.is_open()) landmarks_file_.close();
  output_period_ = 0.0;
  has_previous_state_ = false;
  previous_camera_timestamp_ = 0.0;
  previous_state_ = EskfState{};
  next_regular_timestamp_ = 0.0;
}

}  // namespace sphere_vio
