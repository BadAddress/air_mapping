#pragma once

#include <string>

namespace apollo {
namespace air_mapping {
namespace stage4 {

struct FinalMapConfig {
  double voxel_size_m = 0.2;
  int keyframe_step = 1;
  bool remove_invalid_points = true;
  bool enable_height_crop = true;
  double min_z_m = -30.0;
  double max_z_m = 30.0;
  bool enable_statistical_outlier_removal = true;
  int sor_mean_k = 20;
  double sor_stddev_mul_thresh = 1.0;
};

struct Stage4Config {
  std::string run_config_path;
  std::string vehicle_config_path;
  std::string vehicle_name = "unknown";
  std::string module_root = "/apollo_workspace/modules/air_mapping";
  std::string data_root = "/apollo_workspace/modules/air_mapping/data";
  std::string debug_root = "/apollo_workspace/modules/air_mapping/data/debug";
  std::string input_dir;
  std::string output_dir;
  std::string source_config_path;
  std::string map_name = "stage4_map_export";
  FinalMapConfig final_map;
};

bool LoadStage4Config(const std::string& config_path, Stage4Config* config);

}  // namespace stage4
}  // namespace air_mapping
}  // namespace apollo
