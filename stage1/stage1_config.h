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

struct GpsGateConfig {
  bool enable_ins_gate = true;
  uint32_t ins_gate_status = 2;
  uint32_t ins_gate_pos_type = 56;
  double heading_std_threshold = 1.0;
  bool enable_gps_heading_init = true;
};

struct OutputConfig {
  std::string directory = "/apollo_workspace/modules/air_mapping/data/stage1_lio";
  bool save_keyframe_clouds = true;
  bool save_preview_map = true;
  float preview_voxel_size = 0.2f;
};

struct Stage1Config {
  std::vector<std::string> records;
  std::string map_name = "stage1_lio";
  std::string algorithm_config_path =
      "/apollo_workspace/modules/air_mapping/conf/stage1_lio.yaml";
  ChannelConfig channels;
  GpsGateConfig gps_gate;
  OutputConfig output;
};

bool LoadStage1Config(const std::string& config_path, Stage1Config* config);

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo

