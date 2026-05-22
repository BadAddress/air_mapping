#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace apollo {
namespace air_mapping {
namespace stage1 {

struct ChannelConfig {
  std::string lidar = "/apollo/sensor/rslidar/up/PointCloud2";
  std::string imu = "/apollo/sensor/gnss/corrected_imu";
  std::string imu_type = "corrected";
  std::string heading = "/apollo/sensor/gnss/heading";
  std::string gnss_best_pose = "/apollo/sensor/gnss/best_pose";
  std::string ins_stat = "/apollo/sensor/gnss/ins_stat";
  std::string gps_odom = "/apollo/sensor/gnss/odometry";
};

struct DualLidarConfig {
  bool enable = false;
  std::string config_path;
  std::string primary_channel;
  std::string secondary_channel;
  bool allow_primary_only = true;
};

struct GpsGateConfig {
  bool enable_ins_gate = true;
  uint32_t ins_gate_status = 2;
  uint32_t ins_gate_pos_type = 56;
  double heading_std_threshold = 1.0;
  bool enable_gps_heading_init = true;
};

struct OutputConfig {
  std::string directory;
  bool save_keyframe_clouds = true;
  bool save_preview_map = true;
  float preview_voxel_size = 0.2f;
};

struct GpsZLevelingConfig {
  bool enable = false;
  double max_gps_std_xy_m = 0.03;
  double max_gps_std_z_m = 0.20;
  bool require_rtk_fixed = true;
  uint32_t required_sol_type = 50;
  int min_samples = 10;
  double max_abs_z_offset_m = 20.0;
};

struct LoopClosureConfig {
  bool enable = false;
  double search_radius = 15.0;
  int min_keyframe_gap = 50;
  int loop_kf_gap = 10;
  int min_id_interval = 20;
  int closest_id_threshold = 50;
  int history_submap_half_range = 40;
  int history_submap_step = 4;
  int max_candidates_per_query = 3;
  int ndt_max_iterations = 40;
  double ndt_score_threshold = 0.3;
  std::vector<double> ndt_resolutions = {10.0, 5.0, 2.0, 1.0};
  double ndt_voxel_ratio = 0.1;
  bool use_icp_refine = false;
  int icp_max_iterations = 8;
  double icp_max_corr_dist = 1.0;
  double icp_fitness_threshold = 0.3;
  double icp_max_translation_delta = 0.20;
  double icp_max_rotation_delta_deg = 3.0;
  double lio_translation_sigma_m = 0.08;
  double lio_rotation_sigma_deg = 1.0;
  double loop_translation_sigma_m = 0.25;
  double loop_rotation_sigma_deg = 3.0;
  double loop_info_scale = 1.0;
  double lio_huber_delta = 1.0;
  double loop_cauchy_delta = 1.0;
  int max_iterations = 50;
  bool verbose = false;
};

struct Stage1Config {
  std::string run_config_path;
  std::string vehicle_config_path;
  std::string vehicle_name = "unknown";
  std::string module_root = "/apollo_workspace/modules/air_mapping";
  std::string data_root = "/apollo_workspace/modules/air_mapping/data";
  std::string debug_root = "/apollo_workspace/modules/air_mapping/data/debug";
  std::vector<std::string> dataset_sources;
  std::vector<std::string> records;
  std::string map_name = "stage1_lio";
  std::string algorithm_config_path;
  ChannelConfig channels;
  DualLidarConfig dual_lidar;
  GpsGateConfig gps_gate;
  GpsZLevelingConfig gps_z_leveling;
  OutputConfig output;
  LoopClosureConfig loop_closure;
};

bool LoadStage1Config(const std::string& config_path, Stage1Config* config);

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
