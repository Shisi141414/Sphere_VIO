#include "sphere_vio/ros/output_recorder.hpp"

#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>

#include <iomanip>

namespace sphere_vio {

namespace {

bool ensureDirectory(const std::string& path) {
  return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

}  // namespace

bool OutputRecorder::open(const std::string& directory) {
  close();
  if (directory.empty() || !ensureDirectory(directory)) return false;

  odometry_file_.open(directory + "/odometry.csv");
  landmarks_file_.open(directory + "/landmarks.csv");
  if (!odometry_file_.is_open() || !landmarks_file_.is_open()) {
    close();
    return false;
  }

  odometry_file_ << "timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,"
                    "bias_gyro_x,bias_gyro_y,bias_gyro_z,"
                    "bias_accel_x,bias_accel_y,bias_accel_z\n";
  landmarks_file_ << "landmark_id,x,y,z\n";
  return true;
}

void OutputRecorder::record(
    const EskfState& state,
    const std::vector<BackendLandmark>& landmarks) {
  if (!odometry_file_.is_open()) return;
  odometry_file_ << std::fixed << std::setprecision(9)
                 << state.timestamp << ','
                 << state.p_wb.x() << ',' << state.p_wb.y() << ','
                 << state.p_wb.z() << ','
                 << state.q_wb.w() << ',' << state.q_wb.x() << ','
                 << state.q_wb.y() << ',' << state.q_wb.z() << ','
                 << state.v_wb.x() << ',' << state.v_wb.y() << ','
                 << state.v_wb.z() << ','
                 << state.bias_gyro.x() << ',' << state.bias_gyro.y() << ','
                 << state.bias_gyro.z() << ','
                 << state.bias_accel.x() << ',' << state.bias_accel.y() << ','
                 << state.bias_accel.z() << '\n';

  if (landmarks_file_.is_open()) {
    for (const BackendLandmark& landmark : landmarks) {
      landmarks_file_ << std::fixed << std::setprecision(9)
                      << landmark.track_id.value << ','
                      << landmark.point_w.x() << ',' << landmark.point_w.y()
                      << ',' << landmark.point_w.z() << '\n';
    }
  }
}

void OutputRecorder::close() {
  if (odometry_file_.is_open()) odometry_file_.close();
  if (landmarks_file_.is_open()) landmarks_file_.close();
}

}  // namespace sphere_vio
