#include "modules/air_mapping/stage1/stage1_config.h"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <system_error>

#include "cyber/common/log.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

namespace {

std::vector<std::string> LoadRecordList(const YAML::Node& records_node) {
  std::vector<std::string> records;
  if (!records_node) {
    return records;
  }
  if (records_node.IsSequence()) {
    for (const auto& record : records_node) {
      records.push_back(record.as<std::string>());
    }
  } else {
    records.push_back(records_node.as<std::string>());
  }
  return records;
}

void ExpandRecordPath(const std::string& path, std::vector<std::string>* records) {
  if (records == nullptr) {
    return;
  }
  std::error_code error;
  const std::filesystem::path input(path);
  if (std::filesystem::is_directory(input, error)) {
    std::vector<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(input, error)) {
      if (error) {
        AERROR << "Failed to read record directory: " << path
               << ", error: " << error.message();
        return;
      }
      if (entry.is_regular_file(error)) {
        if (error) {
          AERROR << "Failed to inspect record path: " << entry.path().string()
                 << ", error: " << error.message();
          error.clear();
          continue;
        }
        files.push_back(entry.path().string());
      }
    }
    std::sort(files.begin(), files.end());
    records->insert(records->end(), files.begin(), files.end());
    return;
  }
  records->push_back(path);
}

std::vector<std::string> ExpandRecordPaths(const std::vector<std::string>& inputs) {
  std::vector<std::string> records;
  for (const auto& input : inputs) {
    ExpandRecordPath(input, &records);
  }
  return records;
}

}  // namespace

bool LoadStage1Config(const std::string& config_path, Stage1Config* config) {
  if (config == nullptr) {
    return false;
  }

  try {
    YAML::Node yaml = YAML::LoadFile(config_path);
    config->algorithm_config_path = config_path;

    if (yaml["stage1"]) {
      const auto& stage = yaml["stage1"];
      if (stage["records"]) {
        config->records = LoadRecordList(stage["records"]);
      } else if (stage["record"]) {
        config->records = LoadRecordList(stage["record"]);
      }
      if (stage["map_name"]) {
        config->map_name = stage["map_name"].as<std::string>();
      }
      if (stage["output_dir"]) {
        config->output.directory = stage["output_dir"].as<std::string>();
      }
      if (stage["save_keyframe_clouds"]) {
        config->output.save_keyframe_clouds =
            stage["save_keyframe_clouds"].as<bool>();
      }
      if (stage["save_preview_map"]) {
        config->output.save_preview_map = stage["save_preview_map"].as<bool>();
      }
      if (stage["preview_voxel_size"]) {
        config->output.preview_voxel_size =
            stage["preview_voxel_size"].as<float>();
      }
    }

    if (yaml["channels"]) {
      const auto& channels = yaml["channels"];
      if (channels["lidar"]) {
        config->channels.lidar = channels["lidar"].as<std::string>();
      }
      if (channels["imu"]) {
        config->channels.imu = channels["imu"].as<std::string>();
      }
      if (channels["imu_type"]) {
        config->channels.imu_type = channels["imu_type"].as<std::string>();
      }
      if (channels["heading"]) {
        config->channels.heading = channels["heading"].as<std::string>();
      }
      if (channels["gnss_best_pose"]) {
        config->channels.gnss_best_pose =
            channels["gnss_best_pose"].as<std::string>();
      }
      if (channels["ins_stat"]) {
        config->channels.ins_stat = channels["ins_stat"].as<std::string>();
      }
      if (channels["gps_odom"]) {
        config->channels.gps_odom = channels["gps_odom"].as<std::string>();
      }
    }

    if (yaml["gps_heading_init"] && yaml["gps_heading_init"]["enable"]) {
      config->gps_gate.enable_gps_heading_init =
          yaml["gps_heading_init"]["enable"].as<bool>();
    }
    if (yaml["gps_fusion"]) {
      const auto& gps = yaml["gps_fusion"];
      if (gps["heading_channel"]) {
        config->channels.heading = gps["heading_channel"].as<std::string>();
      }
      if (gps["gps_channel"]) {
        config->channels.gnss_best_pose =
            gps["gps_channel"].as<std::string>();
      }
      if (gps["heading_std_threshold"]) {
        config->gps_gate.heading_std_threshold =
            gps["heading_std_threshold"].as<double>();
      }
      if (gps["enable_ins_gate"]) {
        config->gps_gate.enable_ins_gate = gps["enable_ins_gate"].as<bool>();
      }
      if (gps["ins_gate_status"]) {
        config->gps_gate.ins_gate_status =
            gps["ins_gate_status"].as<uint32_t>();
      }
      if (gps["ins_gate_pos_type"]) {
        config->gps_gate.ins_gate_pos_type =
            gps["ins_gate_pos_type"].as<uint32_t>();
      }
    }
  } catch (const std::exception& e) {
    AERROR << "Failed to load stage1 config: " << config_path
           << ", error: " << e.what();
    return false;
  }

  if (config->records.empty()) {
    AERROR << "No input records configured in " << config_path;
    return false;
  }
  config->records = ExpandRecordPaths(config->records);
  if (config->records.empty()) {
    AERROR << "No record files found from configured inputs in " << config_path;
    return false;
  }
  return true;
}

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
