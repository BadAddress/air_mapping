#include "modules/air_mapping/stage3/stage3_config.h"

#include <algorithm>
#include <exception>
#include <filesystem>

#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/run_config.h"

namespace apollo {
namespace air_mapping {
namespace stage3 {

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

YAML::Node ResolveStage3Yaml(const std::string& config_path,
                             Stage3Config* config) {
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
      VehicleStageDir(config->data_root, config->vehicle_name, "stage2_graph_opt");
  config->output_dir =
      VehicleStageDir(config->data_root, config->vehicle_name,
                      "stage3_graph_refine");
  config->map_name = config->vehicle_name + "_stage3_graph_refine";
  if (run_config.vehicle_yaml["stage3"]) {
    run_config.vehicle_yaml["stage3"]["input_dir"] = config->input_dir;
    run_config.vehicle_yaml["stage3"]["output_dir"] = config->output_dir;
    if (!run_config.vehicle_yaml["stage3"]["map_name"]) {
      run_config.vehicle_yaml["stage3"]["map_name"] = config->map_name;
    }
  }
  return run_config.vehicle_yaml;
}

}  // namespace

bool LoadStage3Config(const std::string& config_path, Stage3Config* config) {
  if (config == nullptr) {
    return false;
  }

  try {
    YAML::Node yaml = ResolveStage3Yaml(config_path, config);
    if (!yaml) {
      return false;
    }
    const bool use_top_level_paths = !config->run_config_path.empty();
    if (yaml["stage3"]) {
      const auto& stage = yaml["stage3"];
      if (!use_top_level_paths && stage["input_dir"]) {
        config->input_dir = stage["input_dir"].as<std::string>();
      }
      if (!use_top_level_paths && stage["output_dir"]) {
        config->output_dir = stage["output_dir"].as<std::string>();
      }
      if (!use_top_level_paths && stage["map_name"]) {
        config->map_name = stage["map_name"].as<std::string>();
      }
    }

    if (yaml["graph_refine"]) {
      const auto& graph = yaml["graph_refine"];
      if (graph["trust_all_quality_fields"]) {
        config->graph.trust_all_quality_fields =
            graph["trust_all_quality_fields"].as<bool>();
      }
      if (graph["lio_translation_sigma_m"]) {
        config->graph.lio_translation_sigma_m =
            graph["lio_translation_sigma_m"].as<double>();
      }
      if (graph["lio_rotation_sigma_deg"]) {
        config->graph.lio_rotation_sigma_deg =
            graph["lio_rotation_sigma_deg"].as<double>();
      }
      if (graph["gps_weight_scale"]) {
        config->graph.gps_weight_scale = graph["gps_weight_scale"].as<double>();
      }
      if (graph["min_gps_std_xy"]) {
        config->graph.min_gps_std_xy = graph["min_gps_std_xy"].as<double>();
      }
      if (graph["min_gps_std_z"]) {
        config->graph.min_gps_std_z = graph["min_gps_std_z"].as<double>();
      }
      if (graph["max_gps_info_xy"]) {
        config->graph.max_gps_info_xy = graph["max_gps_info_xy"].as<double>();
      }
      if (graph["max_gps_info_z"]) {
        config->graph.max_gps_info_z = graph["max_gps_info_z"].as<double>();
      }
      if (graph["use_gps_z"]) {
        config->graph.use_gps_z = graph["use_gps_z"].as<bool>();
      }
      if (graph["max_fused_gps_std_xy"]) {
        config->graph.max_fused_gps_std_xy =
            graph["max_fused_gps_std_xy"].as<double>();
      }
      if (graph["gps_support_max_anchor_gap_m"]) {
        config->graph.gps_support_max_anchor_gap_m =
            graph["gps_support_max_anchor_gap_m"].as<double>();
      }
      if (graph["gps_boundary_ramp_distance_m"]) {
        config->graph.gps_boundary_ramp_distance_m =
            graph["gps_boundary_ramp_distance_m"].as<double>();
      }
      if (graph["first_pose_prior_translation_sigma_m"]) {
        config->graph.first_pose_prior_translation_sigma_m =
            graph["first_pose_prior_translation_sigma_m"].as<double>();
      }
      if (graph["first_pose_prior_rotation_sigma_deg"]) {
        config->graph.first_pose_prior_rotation_sigma_deg =
            graph["first_pose_prior_rotation_sigma_deg"].as<double>();
      }
      if (graph["z_prior_sigma_m"]) {
        config->graph.z_prior_sigma_m = graph["z_prior_sigma_m"].as<double>();
      }
      if (graph["lio_huber_delta"]) {
        config->graph.lio_huber_delta = graph["lio_huber_delta"].as<double>();
      }
      if (graph["gps_huber_delta"]) {
        config->graph.gps_huber_delta = graph["gps_huber_delta"].as<double>();
      }
      if (graph["z_prior_huber_delta"]) {
        config->graph.z_prior_huber_delta =
            graph["z_prior_huber_delta"].as<double>();
      }
      if (graph["first_pose_prior_huber_delta"]) {
        config->graph.first_pose_prior_huber_delta =
            graph["first_pose_prior_huber_delta"].as<double>();
      }
      if (graph["enable_outage_blocks"]) {
        config->graph.enable_outage_blocks =
            graph["enable_outage_blocks"].as<bool>();
      }
      if (graph["outage_min_keyframes"]) {
        config->graph.outage_min_keyframes =
            graph["outage_min_keyframes"].as<int>();
      }
      if (graph["outage_min_length_m"]) {
        config->graph.outage_min_length_m =
            graph["outage_min_length_m"].as<double>();
      }
      if (graph["outage_max_block_length_m"]) {
        config->graph.outage_max_block_length_m =
            graph["outage_max_block_length_m"].as<double>();
      }
      if (graph["outage_support_overlap_keyframes"]) {
        config->graph.outage_support_overlap_keyframes =
            graph["outage_support_overlap_keyframes"].as<int>();
      }
      if (graph["outage_support_radius_m"]) {
        config->graph.outage_support_radius_m =
            graph["outage_support_radius_m"].as<double>();
      }
      if (graph["outage_source_voxel_size_m"]) {
        config->graph.outage_source_voxel_size_m =
            graph["outage_source_voxel_size_m"].as<double>();
      }
      if (graph["outage_target_voxel_size_m"]) {
        config->graph.outage_target_voxel_size_m =
            graph["outage_target_voxel_size_m"].as<double>();
      }
      if (graph["outage_icp_max_iterations"]) {
        config->graph.outage_icp_max_iterations =
            graph["outage_icp_max_iterations"].as<int>();
      }
      if (graph["outage_icp_max_corr_dist_m"]) {
        config->graph.outage_icp_max_corr_dist_m =
            graph["outage_icp_max_corr_dist_m"].as<double>();
      }
      if (graph["outage_icp_fitness_threshold"]) {
        config->graph.outage_icp_fitness_threshold =
            graph["outage_icp_fitness_threshold"].as<double>();
      }
      if (graph["outage_icp_translation_sigma_m"]) {
        config->graph.outage_icp_translation_sigma_m =
            graph["outage_icp_translation_sigma_m"].as<double>();
      }
      if (graph["outage_icp_rotation_sigma_deg"]) {
        config->graph.outage_icp_rotation_sigma_deg =
            graph["outage_icp_rotation_sigma_deg"].as<double>();
      }
      if (graph["outage_icp_huber_delta"]) {
        config->graph.outage_icp_huber_delta =
            graph["outage_icp_huber_delta"].as<double>();
      }
      if (graph["max_iterations"]) {
        config->graph.max_iterations = graph["max_iterations"].as<int>();
      }
      if (graph["verbose"]) {
        config->graph.verbose = graph["verbose"].as<bool>();
      }
    }

    if (yaml["gps_fusion"] &&
        yaml["gps_fusion"]["trust_all_quality_fields"]) {
      config->graph.trust_all_quality_fields =
          yaml["gps_fusion"]["trust_all_quality_fields"].as<bool>();
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
    AERROR << "Failed to load stage3 config: " << config_path
           << ", error: " << e.what();
    return false;
  }

  auto& graph = config->graph;
  graph.lio_translation_sigma_m = std::max(graph.lio_translation_sigma_m, 1e-3);
  graph.lio_rotation_sigma_deg = std::max(graph.lio_rotation_sigma_deg, 1e-3);
  graph.gps_weight_scale = std::max(graph.gps_weight_scale, 1e-6);
  graph.min_gps_std_xy = std::max(graph.min_gps_std_xy, 1e-3);
  graph.min_gps_std_z = std::max(graph.min_gps_std_z, 1e-3);
  graph.max_gps_info_xy = std::max(graph.max_gps_info_xy, 1e-6);
  graph.max_gps_info_z = std::max(graph.max_gps_info_z, 1e-6);
  graph.max_fused_gps_std_xy = std::max(graph.max_fused_gps_std_xy, 0.0);
  graph.gps_support_max_anchor_gap_m =
      std::max(graph.gps_support_max_anchor_gap_m, 0.0);
  graph.gps_boundary_ramp_distance_m =
      std::max(graph.gps_boundary_ramp_distance_m, 0.0);
  graph.first_pose_prior_translation_sigma_m =
      std::max(graph.first_pose_prior_translation_sigma_m, 1e-3);
  graph.first_pose_prior_rotation_sigma_deg =
      std::max(graph.first_pose_prior_rotation_sigma_deg, 1e-3);
  graph.z_prior_sigma_m = std::max(graph.z_prior_sigma_m, 1e-4);
  graph.lio_huber_delta = std::max(graph.lio_huber_delta, 1e-6);
  graph.gps_huber_delta = std::max(graph.gps_huber_delta, 1e-6);
  graph.z_prior_huber_delta = std::max(graph.z_prior_huber_delta, 1e-4);
  graph.first_pose_prior_huber_delta =
      std::max(graph.first_pose_prior_huber_delta, 1e-6);
  graph.outage_min_keyframes = std::max(graph.outage_min_keyframes, 1);
  graph.outage_min_length_m = std::max(graph.outage_min_length_m, 0.0);
  graph.outage_max_block_length_m =
      std::max(graph.outage_max_block_length_m, 0.0);
  graph.outage_support_overlap_keyframes =
      std::max(graph.outage_support_overlap_keyframes, 0);
  graph.outage_support_radius_m = std::max(graph.outage_support_radius_m, 1.0);
  graph.outage_source_voxel_size_m =
      std::max(graph.outage_source_voxel_size_m, 0.01);
  graph.outage_target_voxel_size_m =
      std::max(graph.outage_target_voxel_size_m, 0.01);
  graph.outage_icp_max_iterations =
      std::max(graph.outage_icp_max_iterations, 1);
  graph.outage_icp_max_corr_dist_m =
      std::max(graph.outage_icp_max_corr_dist_m, 0.1);
  graph.outage_icp_fitness_threshold =
      std::max(graph.outage_icp_fitness_threshold, 1e-6);
  graph.outage_icp_translation_sigma_m =
      std::max(graph.outage_icp_translation_sigma_m, 1e-3);
  graph.outage_icp_rotation_sigma_deg =
      std::max(graph.outage_icp_rotation_sigma_deg, 1e-3);
  graph.outage_icp_huber_delta = std::max(graph.outage_icp_huber_delta, 1e-6);
  graph.max_iterations = std::max(graph.max_iterations, 1);

  config->output.preview_voxel_size =
      std::max(config->output.preview_voxel_size, 0.01f);
  config->output.preview_keyframe_step =
      std::max(config->output.preview_keyframe_step, 1);
  if (config->input_dir.empty() || config->output_dir.empty()) {
    AERROR << "Stage3 derived paths are empty in " << config_path;
    return false;
  }
  return true;
}

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
