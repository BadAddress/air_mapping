#include "modules/air_mapping/stage2/stage2_config.h"

#include <algorithm>
#include <exception>

#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

bool LoadStage2Config(const std::string& config_path, Stage2Config* config) {
  if (config == nullptr) {
    return false;
  }

  try {
    YAML::Node yaml = YAML::LoadFile(config_path);
    if (yaml["stage2"]) {
      const auto& stage = yaml["stage2"];
      if (stage["input_dir"]) {
        config->input_dir = stage["input_dir"].as<std::string>();
      }
      if (stage["output_dir"]) {
        config->output_dir = stage["output_dir"].as<std::string>();
      }
      if (stage["source_config_path"]) {
        config->source_config_path =
            stage["source_config_path"].as<std::string>();
      }
      if (stage["map_name"]) {
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
      if (lever["min_satellite_tracked"]) {
        config->lever_arm_calibration.min_satellite_tracked =
            lever["min_satellite_tracked"].as<int>();
      }
      if (lever["max_heading_std_deg"]) {
        config->lever_arm_calibration.max_heading_std_deg =
            lever["max_heading_std_deg"].as<double>();
      }
      if (lever["max_lio_gnss_yaw_diff_deg"]) {
        config->lever_arm_calibration.max_lio_gnss_yaw_diff_deg =
            lever["max_lio_gnss_yaw_diff_deg"].as<double>();
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
      if (lever["use_full_lio_orientation"]) {
        config->lever_arm_calibration.use_full_lio_orientation =
            lever["use_full_lio_orientation"].as<bool>();
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
  config->lever_arm_calibration.min_satellite_tracked =
      std::max(config->lever_arm_calibration.min_satellite_tracked, 0);
  config->lever_arm_calibration.max_heading_std_deg =
      std::max(config->lever_arm_calibration.max_heading_std_deg, 0.0);
  config->lever_arm_calibration.max_lio_gnss_yaw_diff_deg =
      std::max(config->lever_arm_calibration.max_lio_gnss_yaw_diff_deg, 0.0);
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

  return true;
}

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
