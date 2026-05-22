#pragma once

#include <string>

namespace apollo {
namespace air_mapping {
namespace stage3 {

struct GraphRefineConfig {
  double lio_translation_sigma_m = 0.08;
  double lio_rotation_sigma_deg = 1.0;
  double gps_weight_scale = 1.0;
  double min_gps_std_xy = 0.05;
  double min_gps_std_z = 0.05;
  double max_gps_info_xy = 400.0;
  double max_gps_info_z = 40.0;
  bool use_gps_z = false;
  double max_fused_gps_std_xy = 0.03;
  double gps_support_max_anchor_gap_m = 20.0;
  double gps_boundary_ramp_distance_m = 15.0;
  double first_pose_prior_translation_sigma_m = 5.0;
  double first_pose_prior_rotation_sigma_deg = 10.0;
  double lio_huber_delta = 1.0;
  double gps_huber_delta = 2.0;
  double first_pose_prior_huber_delta = 1.0;

  bool enable_outage_blocks = true;
  int outage_min_keyframes = 8;
  double outage_min_length_m = 20.0;
  double outage_max_block_length_m = 0.0;
  int outage_support_overlap_keyframes = 5;
  double outage_support_radius_m = 180.0;
  double outage_source_voxel_size_m = 0.3;
  double outage_target_voxel_size_m = 0.3;
  int outage_icp_max_iterations = 50;
  double outage_icp_max_corr_dist_m = 2.0;
  double outage_icp_fitness_threshold = 1.0;
  double outage_icp_translation_sigma_m = 0.5;
  double outage_icp_rotation_sigma_deg = 3.0;
  double outage_icp_huber_delta = 1.0;

  int max_iterations = 80;
  bool verbose = false;
};

struct Stage3OutputConfig {
  bool save_preview_map = true;
  float preview_voxel_size = 0.2f;
  int preview_keyframe_step = 1;
};

struct Stage3Config {
  std::string run_config_path;
  std::string vehicle_config_path;
  std::string vehicle_name = "unknown";
  std::string module_root = "/apollo_workspace/modules/air_mapping";
  std::string data_root = "/apollo_workspace/modules/air_mapping/data";
  std::string debug_root = "/apollo_workspace/modules/air_mapping/data/debug";
  std::string input_dir;
  std::string output_dir;
  std::string map_name = "stage3_graph_refine";
  GraphRefineConfig graph;
  Stage3OutputConfig output;
};

bool LoadStage3Config(const std::string& config_path, Stage3Config* config);

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
