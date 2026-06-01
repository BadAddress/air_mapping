#include "modules/air_mapping/stage2/stage2_config.h"

#include <algorithm>
#include <exception>
#include <filesystem>

#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/run_config.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

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

YAML::Node ResolveStage2Yaml(const std::string& config_path,
                             Stage2Config* config) {
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
      VehicleStageDir(config->data_root, config->vehicle_name, "stage1_lio");
  config->output_dir =
      VehicleStageDir(config->data_root, config->vehicle_name, "stage2_graph_opt");
  config->source_config_path = run_config.resolved_vehicle_config_path;
  config->map_name = config->vehicle_name + "_stage2_graph_opt";
  if (run_config.vehicle_yaml["stage2"]) {
    run_config.vehicle_yaml["stage2"]["input_dir"] = config->input_dir;
    run_config.vehicle_yaml["stage2"]["output_dir"] = config->output_dir;
    run_config.vehicle_yaml["stage2"]["source_config_path"] =
        config->source_config_path;
    if (!run_config.vehicle_yaml["stage2"]["map_name"]) {
      run_config.vehicle_yaml["stage2"]["map_name"] = config->map_name;
    }
  }
  return run_config.vehicle_yaml;
}

}  // namespace

bool LoadStage2Config(const std::string& config_path, Stage2Config* config) {
  if (config == nullptr) {
    return false;
  }

  try {
    YAML::Node yaml = ResolveStage2Yaml(config_path, config);
    if (!yaml) {
      return false;
    }
    const bool use_top_level_paths = !config->run_config_path.empty();
    if (yaml["stage2"]) {
      const auto& stage = yaml["stage2"];
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
    }

    if (yaml["alignment"]) {
      const auto& alignment = yaml["alignment"];
      if (alignment["gps_segment_break_distance_m"]) {
        config->alignment.gps_segment_break_distance_m =
            alignment["gps_segment_break_distance_m"].as<double>();
      }
      if (alignment["smooth_gps_height"]) {
        config->alignment.smooth_gps_height =
            alignment["smooth_gps_height"].as<bool>();
      }
      if (alignment["gps_height_smoothing_lambda"]) {
        config->alignment.gps_height_smoothing_lambda =
            alignment["gps_height_smoothing_lambda"].as<double>();
      }
      if (alignment["min_gps_std_xy"]) {
        config->alignment.min_gps_std_xy =
            alignment["min_gps_std_xy"].as<double>();
      }
      if (alignment["min_gps_std_z"]) {
        config->alignment.min_gps_std_z =
            alignment["min_gps_std_z"].as<double>();
      }
      if (alignment["min_alignment_anchors"]) {
        config->alignment.min_alignment_anchors =
            alignment["min_alignment_anchors"].as<int>();
      }
      if (alignment["constrain_to_yaw_only"]) {
        config->alignment.constrain_to_yaw_only =
            alignment["constrain_to_yaw_only"].as<bool>();
      }
      if (alignment["robust_huber_delta_m"]) {
        config->alignment.robust_huber_delta_m =
            alignment["robust_huber_delta_m"].as<double>();
      }
      if (alignment["robust_max_iterations"]) {
        config->alignment.robust_max_iterations =
            alignment["robust_max_iterations"].as<int>();
      }
    }

    if (yaml["lever_arm_calibration"]) {
      const auto& lever = yaml["lever_arm_calibration"];
      if (lever["enable"]) {
        config->lever_arm_calibration.enable = lever["enable"].as<bool>();
      }
      if (lever["max_gps_std_xy_m"]) {
        config->lever_arm_calibration.max_gps_std_xy_m =
            lever["max_gps_std_xy_m"].as<double>();
      }
      if (lever["max_interp_gap_s"]) {
        config->lever_arm_calibration.max_interp_gap_s =
            lever["max_interp_gap_s"].as<double>();
      }
      if (lever["require_stage1_gps_anchor"]) {
        config->lever_arm_calibration.require_stage1_gps_anchor =
            lever["require_stage1_gps_anchor"].as<bool>();
      }
      if (lever["require_rtk_fixed"]) {
        config->lever_arm_calibration.require_rtk_fixed =
            lever["require_rtk_fixed"].as<bool>();
      }
      if (lever["required_sol_status"]) {
        config->lever_arm_calibration.required_sol_status =
            lever["required_sol_status"].as<uint32_t>();
      }
      if (lever["required_sol_type"]) {
        config->lever_arm_calibration.required_sol_type =
            lever["required_sol_type"].as<uint32_t>();
      }
      if (lever["max_heading_std_deg"]) {
        config->lever_arm_calibration.max_heading_std_deg =
            lever["max_heading_std_deg"].as<double>();
      }
      if (lever["min_samples"]) {
        config->lever_arm_calibration.min_samples =
            lever["min_samples"].as<int>();
      }
      if (lever["max_iterations"]) {
        config->lever_arm_calibration.max_iterations =
            lever["max_iterations"].as<int>();
      }
      if (lever["prior_sigma_xy_m"]) {
        config->lever_arm_calibration.prior_sigma_xy_m =
            lever["prior_sigma_xy_m"].as<double>();
      }
      if (lever["prior_sigma_z_m"]) {
        config->lever_arm_calibration.prior_sigma_z_m =
            lever["prior_sigma_z_m"].as<double>();
      }
      if (lever["estimate_heading_bias"]) {
        config->lever_arm_calibration.estimate_heading_bias =
            lever["estimate_heading_bias"].as<bool>();
      }
      if (lever["prior_sigma_heading_bias_deg"]) {
        config->lever_arm_calibration.prior_sigma_heading_bias_deg =
            lever["prior_sigma_heading_bias_deg"].as<double>();
      }
      if (lever["max_heading_bias_deg"]) {
        config->lever_arm_calibration.max_heading_bias_deg =
            lever["max_heading_bias_deg"].as<double>();
      }
      if (lever["huber_delta_xy_m"]) {
        config->lever_arm_calibration.huber_delta_xy_m =
            lever["huber_delta_xy_m"].as<double>();
      }
      if (lever["max_correction_norm_m"]) {
        config->lever_arm_calibration.max_correction_norm_m =
            lever["max_correction_norm_m"].as<double>();
      }
      if (lever["estimate_z"]) {
        config->lever_arm_calibration.estimate_z =
            lever["estimate_z"].as<bool>();
      }
    }

    if (yaml["output"]) {
      const auto& output = yaml["output"];
      if (output["save_preview_map"]) {
        config->output.save_preview_map = output["save_preview_map"].as<bool>();
      }
      if (output["preview_voxel_size"]) {
        config->output.preview_voxel_size =
            output["preview_voxel_size"].as<float>();
      }
      if (output["preview_keyframe_step"]) {
        config->output.preview_keyframe_step =
            output["preview_keyframe_step"].as<int>();
      }
    }
  } catch (const std::exception& e) {
    AERROR << "Failed to load stage2 config: " << config_path
           << ", error: " << e.what();
    return false;
  }

  config->alignment.gps_segment_break_distance_m =
      std::max(config->alignment.gps_segment_break_distance_m, 1.0);
  config->alignment.gps_height_smoothing_lambda =
      std::max(config->alignment.gps_height_smoothing_lambda, 0.0);
  config->alignment.min_gps_std_xy =
      std::max(config->alignment.min_gps_std_xy, 1e-3);
  config->alignment.min_gps_std_z =
      std::max(config->alignment.min_gps_std_z, 1e-3);
  config->alignment.min_alignment_anchors =
      std::max(config->alignment.min_alignment_anchors, 3);
  config->alignment.robust_huber_delta_m =
      std::max(config->alignment.robust_huber_delta_m, 0.0);
  config->alignment.robust_max_iterations =
      std::max(config->alignment.robust_max_iterations, 1);
  config->lever_arm_calibration.max_gps_std_xy_m =
      std::max(config->lever_arm_calibration.max_gps_std_xy_m, 1e-4);
  config->lever_arm_calibration.max_interp_gap_s =
      std::max(config->lever_arm_calibration.max_interp_gap_s, 0.0);
  config->lever_arm_calibration.max_heading_std_deg =
      std::max(config->lever_arm_calibration.max_heading_std_deg, 0.0);
  config->lever_arm_calibration.min_samples =
      std::max(config->lever_arm_calibration.min_samples, 3);
  config->lever_arm_calibration.max_iterations =
      std::max(config->lever_arm_calibration.max_iterations, 1);
  config->lever_arm_calibration.prior_sigma_xy_m =
      std::max(config->lever_arm_calibration.prior_sigma_xy_m, 1e-3);
  config->lever_arm_calibration.prior_sigma_z_m =
      std::max(config->lever_arm_calibration.prior_sigma_z_m, 1e-3);
  config->lever_arm_calibration.prior_sigma_heading_bias_deg =
      std::max(config->lever_arm_calibration.prior_sigma_heading_bias_deg,
               1e-3);
  config->lever_arm_calibration.max_heading_bias_deg =
      std::max(config->lever_arm_calibration.max_heading_bias_deg, 0.0);
  config->lever_arm_calibration.huber_delta_xy_m =
      std::max(config->lever_arm_calibration.huber_delta_xy_m, 1e-3);
  config->lever_arm_calibration.max_correction_norm_m =
      std::max(config->lever_arm_calibration.max_correction_norm_m, 0.0);
  config->output.preview_voxel_size =
      std::max(config->output.preview_voxel_size, 0.01f);
  config->output.preview_keyframe_step =
      std::max(config->output.preview_keyframe_step, 1);
  if (config->input_dir.empty() || config->output_dir.empty() ||
      config->source_config_path.empty()) {
    AERROR << "Stage2 derived paths are empty in " << config_path;
    return false;
  }

  return true;
}

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
