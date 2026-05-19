#pragma once

#include <cstdint>
#include <string>

namespace apollo {
namespace air_mapping {
namespace stage2 {

struct LeverArmCalibrationConfig {
  bool enable = true;
  double max_gps_std_xy_m = 0.03;
  double max_interp_gap_s = 0.75;
  bool require_stage1_gps_anchor = false;
  bool require_rtk_fixed = true;
  uint32_t required_sol_status = 0;
  uint32_t required_sol_type = 50;
  int min_satellite_tracked = 0;
  double max_heading_std_deg = 1.0;
  double max_lio_gnss_yaw_diff_deg = 2.0;
  int min_samples = 20;
  int max_iterations = 5;
  double prior_sigma_xy_m = 0.30;
  double prior_sigma_z_m = 0.05;
  bool estimate_heading_bias = true;
  double prior_sigma_heading_bias_deg = 1.0;
  double max_heading_bias_deg = 3.0;
  double huber_delta_xy_m = 0.15;
  double max_correction_norm_m = 0.50;
  bool estimate_z = false;
  // Keep the calibration model aligned with Stage 1's yaw-only anchor
  // synthesis by default. Full roll/pitch can be enabled for experiments.
  bool use_full_lio_orientation = false;
};

struct AlignmentConfig {
  double gps_segment_break_distance_m = 30.0;
  bool smooth_gps_height = true;
  double gps_height_smoothing_lambda = 200.0;
  double min_gps_std_xy = 0.01;
  double min_gps_std_z = 0.05;
  int min_alignment_anchors = 3;
  bool constrain_to_yaw_only = false;
  double robust_huber_delta_m = 2.0;
  int robust_max_iterations = 5;
};

struct Stage2OutputConfig {
  bool save_preview_map = true;
  float preview_voxel_size = 0.2f;
  int preview_keyframe_step = 1;
};

struct Stage2Config {
  std::string input_dir =
      "/apollo_workspace/modules/air_mapping/data/stage1_lio";
  std::string output_dir =
      "/apollo_workspace/modules/air_mapping/data/stage2_graph_opt";
  std::string source_config_path =
      "/apollo_workspace/modules/air_mapping/conf/stage1_lio.yaml";
  std::string map_name = "stage2_graph_opt";
  LeverArmCalibrationConfig lever_arm_calibration;
  AlignmentConfig alignment;
  Stage2OutputConfig output;
};

bool LoadStage2Config(const std::string& config_path, Stage2Config* config);

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
