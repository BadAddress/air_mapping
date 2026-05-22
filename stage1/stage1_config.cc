#include "modules/air_mapping/stage1/stage1_config.h"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <system_error>

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/run_config.h"
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

std::string ReadString(const YAML::Node& node, const std::string& key,
                       const std::string& default_value) {
  return node && node[key] ? node[key].as<std::string>() : default_value;
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

std::string ReadProfileVehicleName(const YAML::Node& yaml,
                                   const std::string& fallback) {
  if (yaml["profile"] && yaml["profile"]["vehicle_name"]) {
    return yaml["profile"]["vehicle_name"].as<std::string>();
  }
  if (yaml["vehicle"] && yaml["vehicle"]["name"]) {
    return yaml["vehicle"]["name"].as<std::string>();
  }
  return fallback;
}

std::string VehicleStageDir(const std::string& data_root,
                            const std::string& vehicle,
                            const std::string& stage_dir) {
  return (std::filesystem::path(data_root) / vehicle / stage_dir).string();
}

std::string WriteRuntimeStage1Yaml(const YAML::Node& yaml,
                                   const Stage1Config& config) {
  const std::filesystem::path runtime_dir =
      std::filesystem::path(config.data_root) / config.vehicle_name /
      "runtime_config";
  std::error_code error;
  std::filesystem::create_directories(runtime_dir, error);
  if (error) {
    AERROR << "Failed to create runtime config directory: "
           << runtime_dir.string() << ", error: " << error.message();
    return "";
  }
  const auto runtime_path = runtime_dir / "stage1_resolved_config.yaml";
  std::ofstream file(runtime_path);
  if (!file.is_open()) {
    AERROR << "Failed to write runtime stage1 config: "
           << runtime_path.string();
    return "";
  }
  file << yaml;
  return runtime_path.string();
}

YAML::Node ResolveStage1Yaml(const std::string& config_path,
                             Stage1Config* config) {
  YAML::Node yaml = YAML::LoadFile(config_path);
  if (!IsTopLevelRunConfig(yaml)) {
    config->algorithm_config_path = config_path;
    return yaml;
  }

  AirMappingRunConfig run_config;
  if (!LoadAirMappingRunConfig(config_path, &run_config)) {
    return YAML::Node();
  }
  config->run_config_path = config_path;
  config->vehicle_config_path = run_config.resolved_vehicle_config_path;
  config->vehicle_name =
      ReadProfileVehicleName(run_config.vehicle_yaml, run_config.active_vehicle);
  config->module_root = run_config.module_root;
  config->data_root = run_config.data_root;
  config->debug_root = run_config.debug_root;
  config->algorithm_config_path = run_config.resolved_vehicle_config_path;
  config->map_name = config->vehicle_name + "_stage1_lio";
  config->output.directory =
      VehicleStageDir(config->data_root, config->vehicle_name, "stage1_lio");
  if (run_config.vehicle_yaml["stage1"]) {
    run_config.vehicle_yaml["stage1"]["output_dir"] = config->output.directory;
    if (!run_config.vehicle_yaml["stage1"]["map_name"]) {
      run_config.vehicle_yaml["stage1"]["map_name"] = config->map_name;
    }
  }
  const std::string runtime_config =
      WriteRuntimeStage1Yaml(run_config.vehicle_yaml, *config);
  if (runtime_config.empty()) {
    return YAML::Node();
  }
  config->algorithm_config_path = runtime_config;
  return run_config.vehicle_yaml;
}

void LoadDualLidarConfig(const YAML::Node& yaml, Stage1Config* config) {
  if (config == nullptr) {
    return;
  }
  if (yaml["dual_lidar"] && yaml["dual_lidar"]["enable"]) {
    config->dual_lidar.enable = yaml["dual_lidar"]["enable"].as<bool>();
  }
  if (yaml["dual_lidar_fusion"] && yaml["dual_lidar_fusion"]["enable"]) {
    config->dual_lidar.enable =
        yaml["dual_lidar_fusion"]["enable"].as<bool>();
  }
  if (!config->dual_lidar.enable) {
    return;
  }

  config->dual_lidar.config_path = config->algorithm_config_path;
  const auto& dual = yaml["dual_lidar"];
  if (dual && dual["sync"] && dual["sync"]["allow_primary_only"]) {
    config->dual_lidar.allow_primary_only =
        dual["sync"]["allow_primary_only"].as<bool>();
  }
  if (dual && dual["channels"]) {
    const std::string primary =
        ReadString(dual, "primary_lidar", std::string("right"));
    const std::string secondary =
        ReadString(dual, "secondary_lidar", std::string("left"));
    const auto& channels = dual["channels"];
    if (channels[primary]) {
      config->dual_lidar.primary_channel = channels[primary].as<std::string>();
      config->channels.lidar = config->dual_lidar.primary_channel;
    }
    if (channels[secondary]) {
      config->dual_lidar.secondary_channel =
          channels[secondary].as<std::string>();
    }
  }
}

}  // namespace

bool LoadStage1Config(const std::string& config_path, Stage1Config* config) {
  if (config == nullptr) {
    return false;
  }

  try {
    YAML::Node yaml = ResolveStage1Yaml(config_path, config);
    if (!yaml) {
      return false;
    }
    const bool use_top_level_paths = !config->run_config_path.empty();

    if (yaml["stage1"]) {
      const auto& stage = yaml["stage1"];
      if (!use_top_level_paths && stage["records"]) {
        config->records = LoadRecordList(stage["records"]);
        config->dataset_sources = config->records;
      } else if (!use_top_level_paths && stage["record"]) {
        config->records = LoadRecordList(stage["record"]);
        config->dataset_sources = config->records;
      }
      if (!use_top_level_paths && stage["map_name"]) {
        config->map_name = stage["map_name"].as<std::string>();
      }
      if (!use_top_level_paths && stage["output_dir"]) {
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
    if (config->records.empty() && yaml["dataset"]) {
      const auto& dataset = yaml["dataset"];
      if (dataset["records"]) {
        config->records = LoadRecordList(dataset["records"]);
      } else if (dataset["record"]) {
        config->records = LoadRecordList(dataset["record"]);
      }
      config->dataset_sources = config->records;
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

    LoadDualLidarConfig(yaml, config);

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

    if (yaml["zleveling"]) {
      const auto& leveling = yaml["zleveling"];
      if (leveling["enable"]) {
        config->zleveling.enable = leveling["enable"].as<bool>();
      }
      if (leveling["height_noise_m"]) {
        config->zleveling.height_noise_m =
            leveling["height_noise_m"].as<double>();
      }
    }

    if (yaml["loop_closure"]) {
      const auto& loop = yaml["loop_closure"];
      auto& config_loop = config->loop_closure;
      if (loop["enable"]) {
        config_loop.enable = loop["enable"].as<bool>();
      }
      if (loop["search_radius"]) {
        config_loop.search_radius = loop["search_radius"].as<double>();
      }
      if (loop["min_keyframe_gap"]) {
        config_loop.min_keyframe_gap = loop["min_keyframe_gap"].as<int>();
      }
      if (loop["loop_kf_gap"]) {
        config_loop.loop_kf_gap = loop["loop_kf_gap"].as<int>();
      }
      if (loop["min_id_interval"]) {
        config_loop.min_id_interval = loop["min_id_interval"].as<int>();
      }
      if (loop["closest_id_th"]) {
        config_loop.closest_id_threshold = loop["closest_id_th"].as<int>();
      }
      if (loop["closest_id_threshold"]) {
        config_loop.closest_id_threshold =
            loop["closest_id_threshold"].as<int>();
      }
      if (loop["history_submap_half_range"]) {
        config_loop.history_submap_half_range =
            loop["history_submap_half_range"].as<int>();
      }
      if (loop["history_submap_step"]) {
        config_loop.history_submap_step = loop["history_submap_step"].as<int>();
      }
      if (loop["max_candidates_per_query"]) {
        config_loop.max_candidates_per_query =
            loop["max_candidates_per_query"].as<int>();
      }
      if (loop["ndt_max_iterations"]) {
        config_loop.ndt_max_iterations = loop["ndt_max_iterations"].as<int>();
      } else if (loop["icp_max_iterations"]) {
        config_loop.ndt_max_iterations = loop["icp_max_iterations"].as<int>();
      }
      if (loop["ndt_score_threshold"]) {
        config_loop.ndt_score_threshold =
            loop["ndt_score_threshold"].as<double>();
      } else if (loop["icp_fitness_threshold"]) {
        config_loop.ndt_score_threshold =
            loop["icp_fitness_threshold"].as<double>();
      }
      if (loop["ndt_resolutions"]) {
        config_loop.ndt_resolutions =
            loop["ndt_resolutions"].as<std::vector<double>>();
      }
      if (loop["ndt_voxel_ratio"]) {
        config_loop.ndt_voxel_ratio = loop["ndt_voxel_ratio"].as<double>();
      }
      if (loop["use_icp_refine"]) {
        config_loop.use_icp_refine = loop["use_icp_refine"].as<bool>();
      }
      if (loop["icp_max_iterations"]) {
        config_loop.icp_max_iterations = loop["icp_max_iterations"].as<int>();
      }
      if (loop["icp_max_corr_dist"]) {
        config_loop.icp_max_corr_dist = loop["icp_max_corr_dist"].as<double>();
      }
      if (loop["icp_fitness_threshold"]) {
        config_loop.icp_fitness_threshold =
            loop["icp_fitness_threshold"].as<double>();
      }
      if (loop["icp_max_translation_delta"]) {
        config_loop.icp_max_translation_delta =
            loop["icp_max_translation_delta"].as<double>();
      }
      if (loop["icp_max_rotation_delta_deg"]) {
        config_loop.icp_max_rotation_delta_deg =
            loop["icp_max_rotation_delta_deg"].as<double>();
      }
      if (loop["lio_translation_sigma_m"]) {
        config_loop.lio_translation_sigma_m =
            loop["lio_translation_sigma_m"].as<double>();
      }
      if (loop["lio_rotation_sigma_deg"]) {
        config_loop.lio_rotation_sigma_deg =
            loop["lio_rotation_sigma_deg"].as<double>();
      }
      if (loop["loop_translation_sigma_m"]) {
        config_loop.loop_translation_sigma_m =
            loop["loop_translation_sigma_m"].as<double>();
      }
      if (loop["loop_rotation_sigma_deg"]) {
        config_loop.loop_rotation_sigma_deg =
            loop["loop_rotation_sigma_deg"].as<double>();
      }
      if (loop["loop_info_scale"]) {
        config_loop.loop_info_scale = loop["loop_info_scale"].as<double>();
      } else if (loop["info_scale"]) {
        config_loop.loop_info_scale = loop["info_scale"].as<double>() / 100.0;
      }
      if (loop["lio_huber_delta"]) {
        config_loop.lio_huber_delta = loop["lio_huber_delta"].as<double>();
      }
      if (loop["loop_cauchy_delta"]) {
        config_loop.loop_cauchy_delta = loop["loop_cauchy_delta"].as<double>();
      }
      if (loop["enable_loop_outlier_rejection"]) {
        config_loop.enable_loop_outlier_rejection =
            loop["enable_loop_outlier_rejection"].as<bool>();
      }
      if (loop["loop_outlier_chi2_threshold"]) {
        config_loop.loop_outlier_chi2_threshold =
            loop["loop_outlier_chi2_threshold"].as<double>();
      }
      if (loop["max_iterations"]) {
        config_loop.max_iterations = loop["max_iterations"].as<int>();
      }
      if (loop["verbose"]) {
        config_loop.verbose = loop["verbose"].as<bool>();
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
  if (config->output.directory.empty()) {
    AERROR << "Stage1 output directory is empty in " << config_path;
    return false;
  }
  auto& z_leveling = config->zleveling;
  z_leveling.height_noise_m = std::max(z_leveling.height_noise_m, 1e-4);
  auto& loop = config->loop_closure;
  loop.loop_kf_gap = std::max(loop.loop_kf_gap, 1);
  loop.min_keyframe_gap = std::max(loop.min_keyframe_gap, 1);
  loop.closest_id_threshold =
      std::max(loop.closest_id_threshold, loop.min_keyframe_gap);
  loop.history_submap_half_range = std::max(loop.history_submap_half_range, 1);
  loop.history_submap_step = std::max(loop.history_submap_step, 1);
  loop.max_candidates_per_query = std::max(loop.max_candidates_per_query, 1);
  loop.ndt_max_iterations = std::max(loop.ndt_max_iterations, 1);
  loop.icp_max_iterations = std::max(loop.icp_max_iterations, 1);
  loop.loop_outlier_chi2_threshold =
      std::max(loop.loop_outlier_chi2_threshold, 1e-6);
  loop.max_iterations = std::max(loop.max_iterations, 1);
  if (loop.ndt_resolutions.empty()) {
    loop.ndt_resolutions = {10.0, 5.0, 2.0, 1.0};
  }
  return true;
}

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
