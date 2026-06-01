#include "modules/air_mapping/stage4/stage4_config.h"

#include <algorithm>
#include <exception>
#include <filesystem>

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/run_config.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace air_mapping {
namespace stage4 {

namespace {

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

void LoadFinalMapConfig(const YAML::Node& node, FinalMapConfig* config) {
  if (!node || config == nullptr) {
    return;
  }
  if (node["voxel_size_m"]) {
    config->voxel_size_m = node["voxel_size_m"].as<double>();
  }
  if (node["keyframe_step"]) {
    config->keyframe_step = node["keyframe_step"].as<int>();
  }
  if (node["remove_invalid_points"]) {
    config->remove_invalid_points = node["remove_invalid_points"].as<bool>();
  }
  if (node["enable_height_crop"]) {
    config->enable_height_crop = node["enable_height_crop"].as<bool>();
  }
  if (node["min_z_m"]) {
    config->min_z_m = node["min_z_m"].as<double>();
  }
  if (node["max_z_m"]) {
    config->max_z_m = node["max_z_m"].as<double>();
  }
  if (node["enable_statistical_outlier_removal"]) {
    config->enable_statistical_outlier_removal =
        node["enable_statistical_outlier_removal"].as<bool>();
  }
  if (node["sor_mean_k"]) {
    config->sor_mean_k = node["sor_mean_k"].as<int>();
  }
  if (node["sor_stddev_mul_thresh"]) {
    config->sor_stddev_mul_thresh =
        node["sor_stddev_mul_thresh"].as<double>();
  }
}

YAML::Node ResolveStage4Yaml(const std::string& config_path,
                             Stage4Config* config) {
  YAML::Node yaml = YAML::LoadFile(config_path);
  if (!IsTopLevelRunConfig(yaml)) {
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
  config->input_dir =
      VehicleStageDir(config->data_root, config->vehicle_name,
                      "stage3_graph_refine");
  config->output_dir =
      VehicleStageDir(config->data_root, config->vehicle_name,
                      "stage4_map_export");
  config->source_config_path = run_config.resolved_vehicle_config_path;
  config->map_name = config->vehicle_name + "_stage4_map_export";
  if (run_config.vehicle_yaml["stage4"]) {
    run_config.vehicle_yaml["stage4"]["input_dir"] = config->input_dir;
    run_config.vehicle_yaml["stage4"]["output_dir"] = config->output_dir;
    run_config.vehicle_yaml["stage4"]["source_config_path"] =
        config->source_config_path;
    if (!run_config.vehicle_yaml["stage4"]["map_name"]) {
      run_config.vehicle_yaml["stage4"]["map_name"] = config->map_name;
    }
  }
  return run_config.vehicle_yaml;
}

void NormalizeFinalMapConfig(FinalMapConfig* config) {
  if (config == nullptr) {
    return;
  }
  config->voxel_size_m = std::max(config->voxel_size_m, 0.01);
  config->keyframe_step = std::max(config->keyframe_step, 1);
  if (config->min_z_m > config->max_z_m) {
    std::swap(config->min_z_m, config->max_z_m);
  }
  config->sor_mean_k = std::max(config->sor_mean_k, 5);
  config->sor_stddev_mul_thresh =
      std::max(config->sor_stddev_mul_thresh, 0.1);
}

}  // namespace

bool LoadStage4Config(const std::string& config_path, Stage4Config* config) {
  if (config == nullptr) {
    return false;
  }

  try {
    YAML::Node yaml = ResolveStage4Yaml(config_path, config);
    if (!yaml) {
      return false;
    }
    const bool use_top_level_paths = !config->run_config_path.empty();

    if (yaml["map_save_filter"]) {
      LoadFinalMapConfig(yaml["map_save_filter"], &config->final_map);
    }
    if (yaml["final_map"]) {
      LoadFinalMapConfig(yaml["final_map"], &config->final_map);
    }
    if (yaml["stage4"]) {
      const auto& stage = yaml["stage4"];
      if (!use_top_level_paths && stage["input_dir"]) {
        config->input_dir = stage["input_dir"].as<std::string>();
      }
      if (!use_top_level_paths && stage["output_dir"]) {
        config->output_dir = stage["output_dir"].as<std::string>();
      }
      if (!use_top_level_paths && stage["source_config_path"]) {
        config->source_config_path =
            stage["source_config_path"].as<std::string>();
      }
      if (!use_top_level_paths && stage["map_name"]) {
        config->map_name = stage["map_name"].as<std::string>();
      }
      LoadFinalMapConfig(stage, &config->final_map);
      if (stage["final_map"]) {
        LoadFinalMapConfig(stage["final_map"], &config->final_map);
      }
    }

    NormalizeFinalMapConfig(&config->final_map);
    AINFO << "[Stage4Config] vehicle=" << config->vehicle_name
          << ", input=" << config->input_dir
          << ", output=" << config->output_dir
          << ", voxel=" << config->final_map.voxel_size_m
          << "m, keyframe_step=" << config->final_map.keyframe_step;
  } catch (const std::exception& e) {
    AERROR << "Failed to load stage4 config: " << config_path
           << ", error: " << e.what();
    return false;
  }
  return true;
}

}  // namespace stage4
}  // namespace air_mapping
}  // namespace apollo
