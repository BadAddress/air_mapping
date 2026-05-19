#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/air_mapping/system/common/eigen_types.h"
#include "modules/air_mapping/system/common/gps_data.h"
#include "modules/air_mapping/system/common/keyframe.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

struct Stage1GpsKeyframeObservation {
  unsigned long keyframe_id = 0;
  double timestamp = 0.0;
  bool has_gps = false;
  lightning::Vec3d utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d std_dev = lightning::Vec3d::Zero();
  double heading_deg = 0.0;
  double heading_std_deg = 0.0;
  uint32_t sol_type = 0;
};

struct Stage1GpsRawKeyframeObservation {
  unsigned long keyframe_id = 0;
  double timestamp = 0.0;
  bool has_stage1_gps = false;
  bool has_raw_gps = false;
  double interp_before_time = 0.0;
  double interp_after_time = 0.0;
  double interp_alpha = 0.0;
  double interp_gap_before_s = 0.0;
  double interp_gap_after_s = 0.0;
  lightning::Vec3d antenna_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d stage1_imu_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d gps_heading_imu_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d lio_raw_yaw_imu_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d lio_opt_yaw_imu_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d configured_lever_arm = lightning::Vec3d::Zero();
  lightning::SE3 lio_raw_pose = lightning::SE3();
  lightning::SE3 lio_opt_pose = lightning::SE3();
  double lio_raw_roll_rad = 0.0;
  double lio_raw_pitch_rad = 0.0;
  double lio_raw_yaw_rad = 0.0;
  double lio_opt_roll_rad = 0.0;
  double lio_opt_pitch_rad = 0.0;
  double lio_opt_yaw_rad = 0.0;
  double gnss_heading_rad = 0.0;
  double gnss_pitch_rad = 0.0;
  double heading_std_deg = 0.0;
  double pitch_std_deg = 0.0;
  lightning::Vec3d std_dev = lightning::Vec3d::Zero();
  uint32_t sol_status = 0;
  uint32_t sol_type = 0;
  uint32_t satellite_tracked = 0;
};

struct Stage1Dataset {
  std::vector<lightning::Keyframe::Ptr> keyframes;
  std::vector<std::string> keyframe_cloud_paths;
  std::vector<std::string> keyframe_covariance_paths;
  std::vector<Stage1GpsKeyframeObservation> gps_keyframe_observations;
  std::vector<Stage1GpsRawKeyframeObservation> gps_raw_keyframe_observations;
  std::vector<lightning::GpsFullObservation> gps_full_history;
};

struct AlignmentAnchor {
  size_t keyframe_index = 0;
  unsigned long keyframe_id = 0;
  double timestamp = 0.0;
  int segment_id = -1;
  lightning::Vec3d lio_position = lightning::Vec3d::Zero();
  lightning::Vec3d gps_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d smoothed_gps_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d gps_std_dev = lightning::Vec3d::Zero();
  double weight = 1.0;
  double residual_before_m = 0.0;
  double residual_after_m = 0.0;
};

struct LeverArmCalibrationSampleDiagnostic {
  unsigned long keyframe_id = 0;
  double timestamp = 0.0;
  bool selected = false;
  std::string reject_reason;
  lightning::Vec3d antenna_utm_position = lightning::Vec3d::Zero();
  lightning::Vec3d gps_std_dev = lightning::Vec3d::Zero();
  double max_interp_gap_s = 0.0;
  lightning::Vec3d lio_position = lightning::Vec3d::Zero();
  lightning::Vec3d predicted_antenna_initial = lightning::Vec3d::Zero();
  lightning::Vec3d predicted_antenna_optimized = lightning::Vec3d::Zero();
  lightning::Vec3d residual_initial = lightning::Vec3d::Zero();
  lightning::Vec3d residual_optimized = lightning::Vec3d::Zero();
  double residual_initial_xy_m = 0.0;
  double residual_optimized_xy_m = 0.0;
  double lio_raw_yaw_rad = 0.0;
  double lio_opt_yaw_rad = 0.0;
  double gnss_heading_rad = 0.0;
  double gnss_pitch_rad = 0.0;
  double gnss_lio_yaw_diff_deg = 0.0;
  double heading_std_deg = 0.0;
  double pitch_std_deg = 0.0;
  uint32_t sol_status = 0;
  uint32_t sol_type = 0;
  uint32_t satellite_tracked = 0;
  double weight = 1.0;
};

struct LeverArmCalibrationResult {
  bool enabled = false;
  bool success = false;
  bool use_full_lio_orientation = false;
  bool estimate_heading_bias = false;
  size_t candidate_count = 0;
  size_t selected_count = 0;
  int iterations = 0;
  lightning::Vec3d initial_lever_arm = lightning::Vec3d::Zero();
  lightning::Vec3d correction = lightning::Vec3d::Zero();
  lightning::Vec3d optimized_lever_arm = lightning::Vec3d::Zero();
  double heading_bias_rad = 0.0;
  double mean_residual_initial_xy_m = 0.0;
  double mean_residual_optimized_xy_m = 0.0;
  double max_residual_initial_xy_m = 0.0;
  double max_residual_optimized_xy_m = 0.0;
  double correction_norm_m = 0.0;
  double weighted_cost_initial = 0.0;
  double weighted_cost_optimized = 0.0;
  std::string status_message;
  std::vector<LeverArmCalibrationSampleDiagnostic> samples;
};

struct GpsSegmentSummary {
  int segment_id = -1;
  size_t first_anchor_index = 0;
  size_t last_anchor_index = 0;
  unsigned long first_keyframe_id = 0;
  unsigned long last_keyframe_id = 0;
  size_t anchor_count = 0;
  double length_xy_m = 0.0;
  double mean_abs_z_smoothing_delta_m = 0.0;
  double max_abs_z_smoothing_delta_m = 0.0;
};

struct Stage2AlignmentResult {
  bool success = false;
  bool yaw_only = false;
  lightning::SE3 lio_to_utm_local = lightning::SE3();
  lightning::Vec3d utm_origin = lightning::Vec3d::Zero();
  size_t anchor_count = 0;
  size_t segment_count = 0;
  double mean_residual_before_m = 0.0;
  double max_residual_before_m = 0.0;
  double mean_residual_after_m = 0.0;
  double max_residual_after_m = 0.0;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
};

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
