//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/maps/tiled_map.h"
#include "core/gps_fusion/gps_fusion_optimizer.h"
#include "common/debug_utils.h"
#include "common/utm_converter.h"
// #include "ui/pangolin_window.h"
// #include "wrapper/ros_utils.h"

#include "cyber/common/log.h" // Use Apollo log
#include <iostream>
#include <deque>
#include <algorithm>
#include <mutex>

#include <yaml-cpp/yaml.h>
#include <filesystem>
// #include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <thread>
#include <chrono>
namespace lightning {

namespace {

struct MapSaveFilterStats {
    size_t input_points = 0;
    size_t after_invalid = 0;
    size_t after_height = 0;
    size_t after_voxel = 0;
    size_t after_outlier = 0;
};

double GetBestPoseTimestampSec(
    const apollo::drivers::gnss::GnssBestPose& gps_msg) {
    if (gps_msg.has_measurement_time() && gps_msg.measurement_time() > 0.0) {
        return gps_msg.measurement_time();
    }
    if (gps_msg.has_header()) {
        return gps_msg.header().timestamp_sec();
    }
    return 0.0;
}

CloudPtr FilterMapForSaving(CloudPtr input,
                            const MapSaveFilterOptions& options,
                            MapSaveFilterStats* stats) {
    if (!input) {
        return CloudPtr(new PointCloudType());
    }

    MapSaveFilterStats local_stats;
    local_stats.input_points = input->size();

    CloudPtr current(new PointCloudType(*input));
    if (options.remove_invalid_points) {
        std::vector<int> indices;
        CloudPtr filtered(new PointCloudType());
        pcl::removeNaNFromPointCloud(*current, *filtered, indices);
        current = filtered;
    }
    local_stats.after_invalid = current->size();

    if (options.enable_height_crop && current && !current->empty()) {
        pcl::PassThrough<PointType> pass;
        pass.setInputCloud(current);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(static_cast<float>(options.min_z_m), static_cast<float>(options.max_z_m));
        CloudPtr filtered(new PointCloudType());
        pass.filter(*filtered);
        if (!filtered->empty()) {
            current = filtered;
        } else {
            AWARN << "[MAP_SAVE] Height crop removed all points, keep pre-height-filter cloud. range=["
                  << options.min_z_m << ", " << options.max_z_m << "]";
        }
    }
    local_stats.after_height = current->size();

    if (options.voxel_size_m > 1e-6 && current && !current->empty()) {
        current = math::VoxelGrid(current, static_cast<float>(options.voxel_size_m));
    }
    local_stats.after_voxel = current->size();

    if (options.enable_statistical_outlier_removal && current && current->size() >= static_cast<size_t>(std::max(5, options.sor_mean_k))) {
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(current);
        sor.setMeanK(std::max(5, options.sor_mean_k));
        sor.setStddevMulThresh(std::max(0.1, options.sor_stddev_mul_thresh));
        CloudPtr filtered(new PointCloudType());
        sor.filter(*filtered);
        if (!filtered->empty()) {
            current = filtered;
        } else {
            AWARN << "[MAP_SAVE] Statistical outlier removal removed all points, keep pre-outlier cloud";
        }
    }
    local_stats.after_outlier = current->size();

    if (current) {
        current->is_dense = false;
        current->height = 1;
        current->width = current->size();
    }

    if (stats) {
        *stats = local_stats;
    }
    return current;
}

}  // namespace

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    // signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        AERROR << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();
    
    if (yaml["system"]["map_path"]) {
        options_.map_path_ = yaml["system"]["map_path"].as<std::string>();
    }

    if (yaml["map_save_filter"]) {
        const auto& map_filter = yaml["map_save_filter"];
        if (map_filter["enable"]) {
            map_save_filter_opts_.enable = map_filter["enable"].as<bool>();
        }
        if (map_filter["voxel_size_m"]) {
            map_save_filter_opts_.voxel_size_m = map_filter["voxel_size_m"].as<double>();
        }
        if (map_filter["chunk_voxel_size_m"]) {
            map_save_filter_opts_.chunk_voxel_size_m = map_filter["chunk_voxel_size_m"].as<double>();
        } else {
            map_save_filter_opts_.chunk_voxel_size_m = map_save_filter_opts_.voxel_size_m;
        }
        if (map_filter["remove_invalid_points"]) {
            map_save_filter_opts_.remove_invalid_points = map_filter["remove_invalid_points"].as<bool>();
        }
        if (map_filter["enable_height_crop"]) {
            map_save_filter_opts_.enable_height_crop = map_filter["enable_height_crop"].as<bool>();
        }
        if (map_filter["min_z_m"]) {
            map_save_filter_opts_.min_z_m = map_filter["min_z_m"].as<double>();
        }
        if (map_filter["max_z_m"]) {
            map_save_filter_opts_.max_z_m = map_filter["max_z_m"].as<double>();
        }
        if (map_filter["enable_statistical_outlier_removal"]) {
            map_save_filter_opts_.enable_statistical_outlier_removal =
                map_filter["enable_statistical_outlier_removal"].as<bool>();
        }
        if (map_filter["sor_mean_k"]) {
            map_save_filter_opts_.sor_mean_k = map_filter["sor_mean_k"].as<int>();
        }
        if (map_filter["sor_stddev_mul_thresh"]) {
            map_save_filter_opts_.sor_stddev_mul_thresh = map_filter["sor_stddev_mul_thresh"].as<double>();
        }
    } else {
        map_save_filter_opts_.chunk_voxel_size_m = map_save_filter_opts_.voxel_size_m;
    }

    AINFO << "[MAP_SAVE] Filter config: enable=" << (map_save_filter_opts_.enable ? "true" : "false")
          << ", voxel=" << map_save_filter_opts_.voxel_size_m << "m"
          << ", chunk_voxel=" << map_save_filter_opts_.chunk_voxel_size_m << "m"
          << ", remove_invalid=" << (map_save_filter_opts_.remove_invalid_points ? "true" : "false")
          << ", height_crop=" << (map_save_filter_opts_.enable_height_crop ? "true" : "false")
          << " [" << map_save_filter_opts_.min_z_m << ", " << map_save_filter_opts_.max_z_m << "]"
          << ", sor=" << (map_save_filter_opts_.enable_statistical_outlier_removal ? "true" : "false")
          << " (mean_k=" << map_save_filter_opts_.sor_mean_k
          << ", std=" << map_save_filter_opts_.sor_stddev_mul_thresh << ")";
    
    // Load GPS lever arm (antenna to IMU offset)
    if (yaml["gps_heading_init"] && yaml["gps_heading_init"]["gps_to_imu_translation"]) {
        auto lever_arm_vec = yaml["gps_heading_init"]["gps_to_imu_translation"].as<std::vector<double>>();
        if (lever_arm_vec.size() == 3) {
            gps_lever_arm_ = Vec3d(lever_arm_vec[0], lever_arm_vec[1], lever_arm_vec[2]);
            AINFO << "[GPS_OFFSET] Loaded GPS lever arm: [" << gps_lever_arm_.x() 
                  << ", " << gps_lever_arm_.y() << ", " << gps_lever_arm_.z() << "] (IMU frame)";
        }
    }
    
    AINFO << "[GPS_HEADING] Host GNSS driver outputs Apollo convention heading (East=0, CCW positive)";
    AINFO << "[GPS_HEADING] Heading value is directly used as ENU yaw (no offset conversion)";
    
    // Load time synchronization offset
    if (yaml["time_sync"] && yaml["time_sync"]["gnss_lidar_time_offset"]) {
        gnss_lidar_time_offset_ = yaml["time_sync"]["gnss_lidar_time_offset"].as<double>();
        AINFO << "[TIME_SYNC] Loaded GNSS-LiDAR time offset: " << gnss_lidar_time_offset_ << " s";
        AINFO << "[TIME_SYNC] Alignment formula: t_lidar = t_gnss - " << gnss_lidar_time_offset_;
    } else {
        AINFO << "[TIME_SYNC] No time offset configured, using 0.0 s";
    }

    // Load GPS attachment sparsity control
    if (yaml["gps_fusion"]) {
        // Prefer new key name, keep legacy key for compatibility
        if (yaml["gps_fusion"]["min_kf_distance_for_gps_attach"]) {
            gps_attach_min_kf_distance_ = yaml["gps_fusion"]["min_kf_distance_for_gps_attach"].as<double>();
        } else if (yaml["gps_fusion"]["min_keyframe_distance"]) {
            gps_attach_min_kf_distance_ = yaml["gps_fusion"]["min_keyframe_distance"].as<double>();
        }
    }
    if (gps_attach_min_kf_distance_ < 0.0) {
        gps_attach_min_kf_distance_ = 0.0;
    }
    AINFO << "[GPS_ATTACH] Minimum keyframe distance for GPS attachment: "
          << gps_attach_min_kf_distance_ << " m";
    
    // 加载 Debug 配置
    std::string common_conf_path = "/apollo_workspace/modules/air_mapping/conf/common_conf.yaml";
    debug_config_ = DebugUtils::LoadConfig(common_conf_path);
    if (debug_config_.enabled) {
        AINFO << "Debug mode enabled for SLAM, output path: " << debug_config_.output_path;
    }

    // pure LIO mode: no loop closing or pose-graph optimization
    /*
    if (options_.with_visualization_) {
        AINFO << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }
    */

    if (options_.with_gridmap_) {
        AINFO << "slam with 2D grid map (g2p5)";
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);
    }
    
    // Initialize GPS fusion optimizer if GPS fusion is enabled
    if (yaml["gps_fusion"] && yaml["gps_fusion"]["enable"].as<bool>(false)) {
        AINFO << "slam with GPS fusion backend optimizer";
        
        GpsFusionOptimizerOptions gps_opts;
        
        // Load constraint weights
        if (yaml["gps_fusion"]["weight_lio_relative"]) {
            gps_opts.weight_lio_relative = yaml["gps_fusion"]["weight_lio_relative"].as<double>();
        }
        if (yaml["gps_fusion"]["weight_height_smooth"]) {
            gps_opts.weight_height_smooth = yaml["gps_fusion"]["weight_height_smooth"].as<double>();
        }
        if (yaml["gps_fusion"]["weight_gps_position"]) {
            gps_opts.weight_gps_position = yaml["gps_fusion"]["weight_gps_position"].as<double>();
        }
        if (yaml["gps_fusion"]["weight_gps_heading"]) {
            gps_opts.weight_gps_heading = yaml["gps_fusion"]["weight_gps_heading"].as<double>();
        }
        
        // Load constraint enable flags
        if (yaml["gps_fusion"]["enable_gps_position"]) {
            gps_opts.enable_gps_position = yaml["gps_fusion"]["enable_gps_position"].as<bool>();
        }
        if (yaml["gps_fusion"]["enable_gps_heading"]) {
            gps_opts.enable_gps_heading = yaml["gps_fusion"]["enable_gps_heading"].as<bool>();
        }
        if (yaml["gps_fusion"]["enable_gps_height"]) {
            gps_opts.enable_gps_height = yaml["gps_fusion"]["enable_gps_height"].as<bool>();
        }
        if (yaml["gps_fusion"]["weight_gps_height"]) {
            gps_opts.weight_gps_height = yaml["gps_fusion"]["weight_gps_height"].as<double>();
        }
        if (yaml["gps_fusion"]["min_gps_std_xy"]) {
            gps_opts.min_gps_std_xy = yaml["gps_fusion"]["min_gps_std_xy"].as<double>();
        }
        if (yaml["gps_fusion"]["max_gps_info_xy"]) {
            gps_opts.max_gps_info_xy = yaml["gps_fusion"]["max_gps_info_xy"].as<double>();
        }
        if (yaml["gps_fusion"]["gps_anchor_min_spacing_m"]) {
            gps_opts.gps_anchor_min_spacing_m = yaml["gps_fusion"]["gps_anchor_min_spacing_m"].as<double>();
        }
        if (yaml["gps_fusion"]["gps_segment_break_distance_m"]) {
            gps_opts.gps_segment_break_distance_m = yaml["gps_fusion"]["gps_segment_break_distance_m"].as<double>();
        }
        if (yaml["gps_fusion"]["gps_boundary_ramp_distance_m"]) {
            gps_opts.gps_boundary_ramp_distance_m = yaml["gps_fusion"]["gps_boundary_ramp_distance_m"].as<double>();
        }
        if (yaml["gps_fusion"]["gps_segment_min_points"]) {
            gps_opts.gps_segment_min_points = yaml["gps_fusion"]["gps_segment_min_points"].as<int>();
        }
        if (yaml["gps_fusion"]["gps_height_trend_anchor_spacing_m"]) {
            gps_opts.gps_height_trend_anchor_spacing_m =
                yaml["gps_fusion"]["gps_height_trend_anchor_spacing_m"].as<double>();
        }
        if (yaml["gps_fusion"]["gps_height_trend_min_points"]) {
            gps_opts.gps_height_trend_min_points =
                yaml["gps_fusion"]["gps_height_trend_min_points"].as<int>();
        }
        if (yaml["gps_fusion"]["gps_height_trend_smoothing_lambda"]) {
            gps_opts.gps_height_trend_smoothing_lambda =
                yaml["gps_fusion"]["gps_height_trend_smoothing_lambda"].as<double>();
        }
        if (yaml["gps_fusion"]["gps_height_trend_min_std_z"]) {
            gps_opts.gps_height_trend_min_std_z =
                yaml["gps_fusion"]["gps_height_trend_min_std_z"].as<double>();
        }
        if (yaml["gps_fusion"]["enable_outage_rigid_segments"]) {
            gps_opts.enable_outage_rigid_segments = yaml["gps_fusion"]["enable_outage_rigid_segments"].as<bool>();
        }
        if (yaml["gps_fusion"]["enable_outage_block_scan_to_map"]) {
            gps_opts.enable_outage_block_scan_to_map =
                yaml["gps_fusion"]["enable_outage_block_scan_to_map"].as<bool>();
        }
        if (yaml["gps_fusion"]["outage_rigid_min_keyframes"]) {
            gps_opts.outage_rigid_min_keyframes = yaml["gps_fusion"]["outage_rigid_min_keyframes"].as<int>();
        }
        if (yaml["gps_fusion"]["outage_rigid_min_length_m"]) {
            gps_opts.outage_rigid_min_length_m = yaml["gps_fusion"]["outage_rigid_min_length_m"].as<double>();
        }
        if (yaml["gps_fusion"]["max_outage_rigid_block_length_m"]) {
            gps_opts.max_outage_rigid_block_length_m = yaml["gps_fusion"]["max_outage_rigid_block_length_m"].as<double>();
        }
        if (yaml["gps_fusion"]["outage_overlap_keyframes"]) {
            gps_opts.outage_overlap_keyframes = yaml["gps_fusion"]["outage_overlap_keyframes"].as<int>();
        }
        if (yaml["gps_fusion"]["outage_block_target_radius_m"]) {
            gps_opts.outage_block_target_radius_m = yaml["gps_fusion"]["outage_block_target_radius_m"].as<double>();
        }
        if (yaml["gps_fusion"]["outage_block_source_window_radius_m"]) {
            gps_opts.outage_block_source_window_radius_m =
                yaml["gps_fusion"]["outage_block_source_window_radius_m"].as<double>();
        }
        if (yaml["gps_fusion"]["outage_block_window_spacing_m"]) {
            gps_opts.outage_block_window_spacing_m =
                yaml["gps_fusion"]["outage_block_window_spacing_m"].as<double>();
        }
        if (yaml["gps_fusion"]["outage_block_min_source_keyframes"]) {
            gps_opts.outage_block_min_source_keyframes =
                yaml["gps_fusion"]["outage_block_min_source_keyframes"].as<int>();
        }
        if (yaml["gps_fusion"]["outage_block_target_sample_step"]) {
            gps_opts.outage_block_target_sample_step =
                yaml["gps_fusion"]["outage_block_target_sample_step"].as<int>();
        }
        if (yaml["gps_fusion"]["outage_block_max_coarse_z_search_m"]) {
            gps_opts.outage_block_max_coarse_z_search_m =
                yaml["gps_fusion"]["outage_block_max_coarse_z_search_m"].as<double>();
        }
        if (yaml["gps_fusion"]["outage_block_coarse_z_step_m"]) {
            gps_opts.outage_block_coarse_z_step_m =
                yaml["gps_fusion"]["outage_block_coarse_z_step_m"].as<double>();
        }
        if (yaml["gps_fusion"]["outage_block_prior_translation_sigma_m"]) {
            gps_opts.outage_block_prior_translation_sigma_m =
                yaml["gps_fusion"]["outage_block_prior_translation_sigma_m"].as<double>();
        }
        if (yaml["gps_fusion"]["outage_block_prior_rotation_sigma_deg"]) {
            gps_opts.outage_block_prior_rotation_sigma_deg =
                yaml["gps_fusion"]["outage_block_prior_rotation_sigma_deg"].as<double>();
        }
        if (yaml["gps_fusion"]["disable_gps_height_on_long_outage"]) {
            gps_opts.disable_gps_height_on_long_outage =
                yaml["gps_fusion"]["disable_gps_height_on_long_outage"].as<bool>();
        }
        if (yaml["gps_fusion"]["long_outage_threshold_m"]) {
            gps_opts.long_outage_threshold_m = yaml["gps_fusion"]["long_outage_threshold_m"].as<double>();
        }
        
        // Load robust kernel parameters
        if (yaml["gps_fusion"]["huber_lio_delta"]) {
            gps_opts.huber_lio_delta = yaml["gps_fusion"]["huber_lio_delta"].as<double>();
        }
        if (yaml["gps_fusion"]["huber_gps_pos_delta"]) {
            gps_opts.huber_gps_pos_delta = yaml["gps_fusion"]["huber_gps_pos_delta"].as<double>();
        }
        if (yaml["gps_fusion"]["cauchy_heading_delta"]) {
            gps_opts.cauchy_heading_delta = yaml["gps_fusion"]["cauchy_heading_delta"].as<double>();
        }
        
        // Load solver parameters
        if (yaml["gps_fusion"]["max_iterations"]) {
            gps_opts.max_iterations = yaml["gps_fusion"]["max_iterations"].as<int>();
        }
        if (yaml["gps_fusion"]["verbose"]) {
            gps_opts.verbose = yaml["gps_fusion"]["verbose"].as<bool>();
        }
        
        gps_optimizer_ = std::make_shared<GpsFusionOptimizer>(gps_opts);
        
        // Load loop closure options
        LoopClosureOptions loop_opts;
        if (yaml["loop_closure"]) {
            auto lc = yaml["loop_closure"];
            if (lc["enable"]) loop_opts.enable = lc["enable"].as<bool>();
            if (lc["search_radius"]) loop_opts.search_radius = lc["search_radius"].as<double>();
            if (lc["min_keyframe_gap"]) loop_opts.min_keyframe_gap = lc["min_keyframe_gap"].as<int>();
            if (lc["loop_kf_gap"]) loop_opts.loop_kf_gap = lc["loop_kf_gap"].as<int>();
            if (lc["min_id_interval"]) loop_opts.min_id_interval = lc["min_id_interval"].as<int>();
            if (lc["closest_id_th"]) loop_opts.closest_id_threshold = lc["closest_id_th"].as<int>();
            if (lc["history_submap_half_range"]) {
                loop_opts.history_submap_half_range = lc["history_submap_half_range"].as<int>();
            }
            if (lc["history_submap_step"]) loop_opts.history_submap_step = lc["history_submap_step"].as<int>();
            if (lc["icp_max_iterations"]) loop_opts.icp_max_iterations = lc["icp_max_iterations"].as<int>();
            if (lc["icp_max_corr_dist"]) loop_opts.icp_max_corr_dist = lc["icp_max_corr_dist"].as<double>();
            if (lc["icp_fitness_threshold"]) loop_opts.icp_fitness_threshold = lc["icp_fitness_threshold"].as<double>();
            if (lc["info_scale"]) loop_opts.info_scale = lc["info_scale"].as<double>();
        }
        gps_optimizer_->SetLoopClosureOptions(loop_opts);
        
        AINFO << "GPS Fusion Optimizer initialized with:";
        AINFO << "  - Weights: LIO=" << gps_opts.weight_lio_relative 
              << ", GPS_pos=" << gps_opts.weight_gps_position
              << ", GPS_yaw=" << gps_opts.weight_gps_heading;
        AINFO << "  - GPS safeguards: min_std_xy=" << gps_opts.min_gps_std_xy
              << ", max_info_xy=" << gps_opts.max_gps_info_xy
              << ", anchor_spacing=" << gps_opts.gps_anchor_min_spacing_m
              << ", segment_break=" << gps_opts.gps_segment_break_distance_m
              << ", boundary_ramp=" << gps_opts.gps_boundary_ramp_distance_m
              << ", z_trend_spacing=" << gps_opts.gps_height_trend_anchor_spacing_m
              << ", z_trend_lambda=" << gps_opts.gps_height_trend_smoothing_lambda
              << ", outage_rigid=" << (gps_opts.enable_outage_rigid_segments ? "on" : "off")
              << ", outage_block_s2m=" << (gps_opts.enable_outage_block_scan_to_map ? "on" : "off")
              << ", outage_min_kf=" << gps_opts.outage_rigid_min_keyframes
              << ", outage_min_len=" << gps_opts.outage_rigid_min_length_m
              << ", outage_block_max=" << gps_opts.max_outage_rigid_block_length_m
              << ", outage_overlap_kf=" << gps_opts.outage_overlap_keyframes
              << ", outage_block_target_r=" << gps_opts.outage_block_target_radius_m
              << ", outage_block_source_r=" << gps_opts.outage_block_source_window_radius_m
              << ", outage_block_spacing=" << gps_opts.outage_block_window_spacing_m
              << ", outage_block_coarse_z=" << gps_opts.outage_block_max_coarse_z_search_m
              << ", disable_gps_z_long_gap=" << (gps_opts.disable_gps_height_on_long_outage ? "on" : "off")
              << ", long_gap_th=" << gps_opts.long_outage_threshold_m;
        AINFO << "  - Loop closure: " << (loop_opts.enable ? "enabled" : "disabled")
              << ", radius=" << loop_opts.search_radius
              << "m, gap=" << loop_opts.min_keyframe_gap
              << ", loop_kf_gap=" << loop_opts.loop_kf_gap
              << ", min_id_interval=" << loop_opts.min_id_interval
              << ", closest_id_th=" << loop_opts.closest_id_threshold
              << ", hist_half_range=" << loop_opts.history_submap_half_range
              << ", hist_step=" << loop_opts.history_submap_step;
    }

    return true;
}

SlamSystem::~SlamSystem() {
    /*
    if (ui_) {
        ui_->Quit();
    }
    */
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

std::vector<Keyframe::Ptr> SlamSystem::GetAllKeyframes() const {
    if (!lio_) {
        return {};
    }
    return lio_->GetAllKeyframes();
}

std::vector<GpsFullObservation> SlamSystem::GetGpsFullHistory() const {
    std::lock_guard<std::mutex> lock(mtx_gps_pair_);
    return std::vector<GpsFullObservation>(gps_full_history_.begin(), gps_full_history_.end());
}

CloudPtr SlamSystem::GetGlobalMapFromKeyframes(const std::vector<Keyframe::Ptr>& keyframes,
                                               bool use_voxel,
                                               float res) const {
    if (!lio_) {
        return CloudPtr(new PointCloudType());
    }
    return lio_->GetGlobalMapFromKeyframes(keyframes, use_voxel, res);
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        if (!options_.map_path_.empty()) {
            save_path = options_.map_path_;
        } else {
            save_path = "./data/" + map_name_ + "/";
        }
    }

    AINFO << "slam map saving to " << save_path;
    
    // Run final GPS fusion optimization if enabled
    if (gps_optimizer_) {
        AINFO << "Running final GPS fusion optimization before saving map...";
        std::vector<GpsFullObservation> gps_history_snapshot;
        {
            std::lock_guard<std::mutex> lock(mtx_gps_pair_);
            gps_history_snapshot.assign(gps_full_history_.begin(), gps_full_history_.end());
        }
        gps_optimizer_->SetRawGpsHistory(gps_history_snapshot);
        AINFO << "[GPS_HEIGHT] Raw GPS history snapshot passed to optimizer: "
              << gps_history_snapshot.size() << " observations";
        gps_optimizer_->FinalOptimize();
    }

    // pure LIO map saving

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    std::vector<Keyframe::Ptr> all_keyframes = lio_->GetAllKeyframes();
    AINFO << "Using LIO keyframes for map saving: " << all_keyframes.size() << " keyframes";
    
    if (all_keyframes.empty()) {
        AWARN << "No keyframes to save!";
        return;
    }
    
    auto global_map = lio_->GetGlobalMapFromKeyframes(all_keyframes);
    if (map_save_filter_opts_.enable) {
        MapSaveFilterStats filter_stats;
        auto filtered_map = FilterMapForSaving(global_map, map_save_filter_opts_, &filter_stats);
        if (filtered_map && !filtered_map->empty()) {
            global_map = filtered_map;
        } else {
            AWARN << "[MAP_SAVE] Filtered map is empty, fallback to original global map";
        }

        AINFO << "[MAP_SAVE] Filter stats:"
              << " input=" << filter_stats.input_points
              << ", after_invalid=" << filter_stats.after_invalid
              << ", after_height=" << filter_stats.after_height
              << ", after_voxel=" << filter_stats.after_voxel
              << ", after_outlier=" << filter_stats.after_outlier;
    }

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;
    tm_options.voxel_size_in_chunk_ = static_cast<float>(
        std::max(1e-3, map_save_filter_opts_.chunk_voxel_size_m));

    TiledMap tm(tm_options);
    SE3 start_pose = all_keyframes.front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    std::string global_pcd_path = save_path + "/global.pcd";
    if (pcl::io::savePCDFileBinaryCompressed(global_pcd_path, *global_map) < 0) {
        AWARN << "Compressed PCD save failed (file too large), using uncompressed format";
        pcl::io::savePCDFileBinary(global_pcd_path, *global_map);
    }

    // ========== 保存位姿数据用于评估 (仅 DEBUG 模式) ==========
    // 仅在 Debug 模式开启时保存详细的位姿数据，避免影响性能
    if (debug_config_.enabled && !all_keyframes.empty()) {
        std::string debug_path = DebugUtils::GetOrCreateDebugOutputDir(debug_config_.output_path);
        
        AINFO << "[DEBUG] Debug mode enabled, saving pose data for evaluation...";
        
        // 获取当前时间作为实验标识
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_time_t);
        
        // 1. 保存优化后的位姿 (slam_poses_opt.txt)
        std::string opt_poses_file = debug_path + "/slam_poses_opt.txt";
        {
            std::ofstream ofs(opt_poses_file);
            if (ofs.is_open()) {
                ofs << "# ============================================================\n";
                ofs << "# SLAM Optimized Poses (GPS Fusion Backend)\n";
                ofs << "# ============================================================\n";
                ofs << "# Experiment time: " << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "\n";
                ofs << "# Total keyframes: " << all_keyframes.size() << "\n";
                ofs << "# \n";
                ofs << "# Optimization: GPS XY + GPS Z + Height smooth + LIO (Z@1%)\n";
                ofs << "# \n";
                ofs << "# Format: TUM trajectory format\n";
                ofs << "# timestamp tx ty tz qx qy qz qw\n";
                ofs << "# Coordinate system: LIO frame (aligned with UTM via GPS heading init)\n";
                ofs << "# ============================================================\n";
                ofs << "#\n";
                
                for (const auto& kf : all_keyframes) {
                    if (!kf) continue;
                    SE3 pose = kf->GetOptPose();
                    ofs << DebugUtils::SE3ToTUMString(kf->GetTimestamp(), pose) << "\n";
                }
                ofs.close();
                AINFO << "[DEBUG] Saved optimized poses to: " << opt_poses_file;
            }
        }
        
        // 2. 保存 LIO 原始位姿 (slam_poses_lio.txt)
        std::string lio_poses_file = debug_path + "/slam_poses_lio.txt";
        {
            std::ofstream ofs(lio_poses_file);
            if (ofs.is_open()) {
                ofs << "# ============================================================\n";
                ofs << "# SLAM LIO Frontend Poses (Before Optimization)\n";
                ofs << "# ============================================================\n";
                ofs << "# Experiment time: " << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "\n";
                ofs << "# Total keyframes: " << all_keyframes.size() << "\n";
                ofs << "# \n";
                ofs << "# This file contains raw LIO poses before GPS fusion optimization.\n";
                ofs << "# Use this to compare with optimized poses and GPS ground truth.\n";
                ofs << "# \n";
                ofs << "# Format: TUM trajectory format\n";
                ofs << "# timestamp tx ty tz qx qy qz qw\n";
                ofs << "# Coordinate system: LIO frame\n";
                ofs << "# ============================================================\n";
                ofs << "#\n";
                
                for (const auto& kf : all_keyframes) {
                    if (!kf) continue;
                    SE3 pose = kf->GetLIOPose();
                    ofs << DebugUtils::SE3ToTUMString(kf->GetTimestamp(), pose) << "\n";
                }
                ofs.close();
                AINFO << "[DEBUG] Saved LIO poses to: " << lio_poses_file;
            }
        }
        
        // 3. 保存兼容性文件 (slam_poses_ba.txt - 保持向后兼容)
        std::string ba_poses_file = debug_path + "/slam_poses_ba.txt";
        if (DebugUtils::SaveKeyframesToTUM(all_keyframes, ba_poses_file, true)) {
            AINFO << "[DEBUG] Saved BA poses to: " << ba_poses_file;
        }
        
        AINFO << "[DEBUG] ========== Pose files saved for evaluation ==========";
        AINFO << "[DEBUG] Optimized poses: " << opt_poses_file;
        AINFO << "[DEBUG] LIO poses:       " << lio_poses_file;
        AINFO << "[DEBUG] BA poses:        " << ba_poses_file;
        AINFO << "[DEBUG] GPS ground truth: (recorded separately by GpsOdomRecorder)";
    } else if (!all_keyframes.empty()) {
        AINFO << "[DEBUG] Debug mode DISABLED, skipping pose data export (set debug.enabled=true in common_conf.yaml to enable)";
    }

    // 保存 2D 栅格地图 (ROS 导航兼容格式)
    if (options_.with_gridmap_ && g2p5_) {
        // 使用所有关键帧重新生成带射线清除的地图
        // 这里的关键帧位姿已经是经过 PGO 优化的
        AINFO << "Generating 2D grid map from all keyframes (with ray clearing)...";
        g2p5_->GenerateMapFromKeyframes(all_keyframes);

        auto map = g2p5_->GetNewestMap();
        if (map) {
            // 保存 PGM 图像
            std::string pgm_path = save_path + "/map.pgm";
            if (map->SavePGM(pgm_path)) {
                AINFO << "2D grid map saved to: " << pgm_path;
                
                // 保存 map.yaml (ROS 导航标准格式)
                std::string yaml_path = save_path + "/map.yaml";
                std::ofstream yaml_file(yaml_path);
                if (yaml_file.is_open()) {
                    float origin_x, origin_y;
                    map->GetOrigin(origin_x, origin_y);
                    float resolution = map->GetGridResolution();
                    int width = map->GetImageWidth();
                    int height = map->GetImageHeight();
                    
                    yaml_file << "image: map.pgm\n";
                    yaml_file << "mode: trinary\n";
                    yaml_file << "resolution: " << resolution << "\n";
                    yaml_file << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
                    yaml_file << "negate: 0\n";
                    yaml_file << "occupied_thresh: 0.65\n";
                    yaml_file << "free_thresh: 0.25\n";
                    yaml_file << "# Map size: " << width << " x " << height << "\n";
                    yaml_file.close();
                    
                    AINFO << "Map metadata saved to: " << yaml_path;
                } else {
                    AWARN << "Failed to write map.yaml";
                }
            } else {
                AWARN << "Failed to save 2D grid map";
            }
        } else {
            AWARN << "G2P5 map is null, skip saving 2D grid map.";
        }
    }
    
    // Save UTM alignment file for visualization
    // This file contains the transformation from LIO frame to UTM frame
    // Since GPS heading is applied at LIO initialization, the frames are already parallel (no rotation needed)
    // Only translation offset is stored for HD map visualization
    std::string alignment_file = save_path + "/utm_alignment.txt";
    std::ofstream ofs(alignment_file);
    if (ofs.is_open()) {
        ofs << "# UTM Alignment File\n";
        ofs << "# This file defines the transformation from LIO coordinate frame to UTM coordinate frame\n";
        ofs << "# LIO frame is already rotated to align with UTM at initialization (via GPS heading)\n";
        ofs << "# Format: offset_x offset_y offset_z yaw_deg\n";
        ofs << "# - offset: Translation from LIO origin to UTM (meters)\n";
        ofs << "# - yaw_deg: Rotation already applied at init, always 0.0 here\n";
        ofs << "\n";
        
        // Get GPS heading information if available
        double yaw_rad = 0.0;
        bool has_heading = lio_->GetGPSHeadingInfo(yaw_rad);
        
        if (has_heading) {
            ofs << "# GPS heading was applied during LIO initialization\n";
            ofs << "# Applied yaw (ENU, radians): " << yaw_rad << "\n";
            ofs << "# Applied yaw (ENU, degrees): " << (yaw_rad * 180.0 / M_PI) << "\n";
        } else {
            ofs << "# No GPS heading was applied (manual alignment needed)\n";
        }
        
        // Compute offset as the average across ALL GPS keyframes (robust)
        // Old approach used only the first GPS keyframe, making the offset
        // sensitive to GPS fusion optimization moving that single keyframe.
        Vec3d offset = Vec3d::Zero();
        
        if (first_gps_received_ && !all_keyframes.empty()) {
            Vec3d offset_sum = Vec3d::Zero();
            int gps_count = 0;
            for (const auto& kf : all_keyframes) {
                auto gps_data = kf->GetGpsData();
                if (!gps_data.has_gps) continue;
                Vec3d kf_opt_pos = kf->GetOptPose().translation();
                Vec3d kf_offset = gps_data.gps_utm_position - kf_opt_pos;
                if (kf_offset.allFinite()) {
                    offset_sum += kf_offset;
                    gps_count++;
                }
            }
            
            if (gps_count > 0) {
                offset = offset_sum / gps_count;
                AINFO << "[GPS_OFFSET] UTM offset computed from " << gps_count
                      << " GPS keyframes (global average):";
                AINFO << "  Offset: [" << offset.x() << ", " 
                      << offset.y() << ", " << offset.z() << "]";
            } else {
                Vec3d first_kf_pos = all_keyframes[0]->GetOptPose().translation();
                offset = first_gps_utm_ - first_kf_pos;
                AINFO << "[GPS_OFFSET] No GPS keyframes found, fallback to first GPS pair:";
                AINFO << "  Offset: [" << offset.x() << ", " 
                      << offset.y() << ", " << offset.z() << "]";
            }
        } else {
            AWARN << "[GPS_OFFSET] No GPS data received, offset set to zero";
            AWARN << "  You need to manually set offset in utm_alignment.txt";
        }
        
        ofs << "\noffset_x: " << std::fixed << std::setprecision(6) << offset.x() << "\n";
        ofs << "offset_y: " << std::fixed << std::setprecision(6) << offset.y() << "\n";
        ofs << "offset_z: " << std::fixed << std::setprecision(6) << offset.z() << "\n";
        ofs << "yaw_deg: 0.0\n";  // Rotation already applied at init
        ofs << "\n";
        ofs << "# Note: For HD map visualization, apply ONLY translation offset\n";
        ofs << "# p_viz = p_hdmap_utm - offset\n";
        ofs << "# No rotation needed since LIO frame is already parallel to UTM\n";
        
        ofs.close();
        AINFO << "UTM alignment file saved to: " << alignment_file;
        
        if (has_heading) {
            AINFO << "  - GPS heading applied: " << (yaw_rad * 180.0 / M_PI) << " degrees";
        }
        AINFO << "  - Offset: [" << offset.x() << ", " << offset.y() << ", " << offset.z() << "]";
    } else {
        AWARN << "Failed to create UTM alignment file: " << alignment_file;
    }
    
    AINFO << "map saved";
}

void SlamSystem::RequestShutdown() {
    AINFO << "[SHUTDOWN] Graceful shutdown requested (CTRL+C)";
    shutdown_requested_.store(true);
    
    // Wait for any ongoing optimization to complete
    if (gps_optimizer_) {
        if (gps_optimizer_->IsOptimizing()) {
            AINFO << "[SHUTDOWN] Waiting for ongoing GPS optimization to complete...";
            while (gps_optimizer_->IsOptimizing()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            AINFO << "[SHUTDOWN] GPS optimization completed";
        }
    }
    
    AINFO << "[SHUTDOWN] Performing final optimization and saving map...";
    SaveMap();  // This will trigger FinalOptimize() inside
    
    AINFO << "[SHUTDOWN] Graceful shutdown complete";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const std::shared_ptr<apollo::drivers::PointCloud>& cloud) {
    if (running_ == false) {
        return;
    }

    // lio_->ProcessPointCloud2 now needs to accept Apollo PointCloud
    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    // Attach interpolated GPS observation to the new keyframe (if available)
    AttachGpsToKeyframe(cur_kf_);
    
    // Notify backend optimizer (if enabled)
    OnKeyframeCreated(cur_kf_);

    if (options_.with_gridmap_ && g2p5_) {
        g2p5_->PushKeyframe(cur_kf_);
    }
}

void SlamSystem::ProcessHeading(const HeadingObservation& heading) {
    if (running_ == false) {
        return;
    }
    
    if (!lio_) {
        AWARN << "[GPS_HEADING_INIT] LIO module not initialized, cannot process heading";
        return;
    }
    
    AINFO << "[GPS_DEBUG] Received Heading: timestamp=" << std::fixed << std::setprecision(6) 
          << heading.timestamp << ", heading=" << heading.heading << " deg";
    
    // Pass heading to LIO for initialization (NO correction applied)
    lio_->ProcessHeadingForInit(heading);
    
    // Also add to pairing cache for GPS full observation
    std::lock_guard<std::mutex> lock(mtx_gps_pair_);
    
    // Try to find matching Best Pose by timestamp
    bool found_match = false;
    for (auto& candidate : gps_pair_cache_) {
        if (std::abs(candidate.timestamp - heading.timestamp) < GPS_TIME_MATCH_TOLERANCE) {
            // Found matching timestamp, add heading data
            candidate.heading_rad = heading.GetYawRad();
            candidate.pitch_rad = heading.pitch * M_PI / 180.0;
            candidate.heading_std_dev = heading.heading_std_dev;
            candidate.pitch_std_dev = heading.pitch_std_dev;
            candidate.satellite_tracked = heading.satellite_tracked;
            candidate.has_heading = true;
            
            // If pair is complete, process it
            if (candidate.IsComplete()) {
                ProcessGpsPair(candidate);
                // Remove from cache
                gps_pair_cache_.erase(std::remove_if(gps_pair_cache_.begin(), gps_pair_cache_.end(),
                    [&](const GpsPairCandidate& c) { 
                        return std::abs(c.timestamp - candidate.timestamp) < 1e-6; 
                    }), gps_pair_cache_.end());
            }
            
            found_match = true;
            break;
        }
    }
    
    // If no match found, create new candidate
    if (!found_match) {
        GpsPairCandidate new_candidate;
        new_candidate.timestamp = heading.timestamp;
        new_candidate.heading_rad = heading.GetYawRad();
        new_candidate.pitch_rad = heading.pitch * M_PI / 180.0;
        new_candidate.heading_std_dev = heading.heading_std_dev;
        new_candidate.pitch_std_dev = heading.pitch_std_dev;
        new_candidate.satellite_tracked = heading.satellite_tracked;
        new_candidate.has_heading = true;
        gps_pair_cache_.push_back(new_candidate);
    }
    
    // Clean up old unpaired data (timeout > 1s)
    double current_time = heading.timestamp;
    gps_pair_cache_.erase(std::remove_if(gps_pair_cache_.begin(), gps_pair_cache_.end(),
        [&](const GpsPairCandidate& c) { 
            return (current_time - c.timestamp) > GPS_PAIR_TIMEOUT; 
        }), gps_pair_cache_.end());
    
    // Limit cache size
    while (gps_pair_cache_.size() > GPS_PAIR_CACHE_MAX_SIZE) {
        gps_pair_cache_.pop_front();
    }
}

void SlamSystem::ProcessGPS(const std::shared_ptr<apollo::drivers::gnss::GnssBestPose>& gps) {
    if (!running_) {
        return;
    }
    if (!gps) {
        return;
    }
    
    if (!gps->has_latitude() || !gps->has_longitude() || !gps->has_height_msl()) {
        return;
    }
    
    // Prefer measurement_time as sensor time, but fall back to header timestamp
    // for compatibility with older driver outputs.
    double raw_measurement_time = GetBestPoseTimestampSec(*gps);
    if (raw_measurement_time <= 0.0) {
        ADEBUG << "[GPS_DEBUG] Reject Best Pose: missing usable timestamp";
        return;
    }
    
    AINFO << "[GPS_DEBUG] Received Best Pose: lat=" << gps->latitude() 
          << ", lon=" << gps->longitude() << ", height=" << gps->height_msl()
          << ", measurement_time=" << std::fixed << std::setprecision(6) << raw_measurement_time;
    
    // Convert GPS antenna position to UTM
    Vec3d antenna_utm = UTMConverter::LatLonToVec3d(
        gps->latitude(), gps->longitude(), gps->height_msl());
    
    // Extract quality information
    Vec3d position_std_dev = Vec3d::Zero();
    if (gps->has_latitude_std_dev() && gps->has_longitude_std_dev() && gps->has_height_std_dev()) {
        position_std_dev = Vec3d(gps->latitude_std_dev(), gps->longitude_std_dev(), gps->height_std_dev());
    }
    
    uint32_t sol_status = gps->has_sol_status() ? gps->sol_status() : 0;
    uint32_t sol_type = gps->has_sol_type() ? gps->sol_type() : 0;
    double timestamp = raw_measurement_time - gnss_lidar_time_offset_;
    
    // Add to pairing cache
    std::lock_guard<std::mutex> lock(mtx_gps_pair_);
    
    // Try to find matching Heading by timestamp
    bool found_match = false;
    for (auto& candidate : gps_pair_cache_) {
        if (std::abs(candidate.timestamp - timestamp) < GPS_TIME_MATCH_TOLERANCE) {
            // Found matching timestamp, add Best Pose data
            candidate.antenna_utm = antenna_utm;
            candidate.position_std_dev = position_std_dev;
            candidate.sol_status = sol_status;
            candidate.sol_type = sol_type;
            candidate.has_position = true;
            
            // If pair is complete, process it
            if (candidate.IsComplete()) {
                ProcessGpsPair(candidate);
                // Remove from cache
                gps_pair_cache_.erase(std::remove_if(gps_pair_cache_.begin(), gps_pair_cache_.end(),
                    [&](const GpsPairCandidate& c) { 
                        return std::abs(c.timestamp - candidate.timestamp) < 1e-6; 
                    }), gps_pair_cache_.end());
            }
            
            found_match = true;
            break;
        }
    }
    
    // If no match found, create new candidate
    if (!found_match) {
        GpsPairCandidate new_candidate;
        new_candidate.timestamp = timestamp;
        new_candidate.antenna_utm = antenna_utm;
        new_candidate.position_std_dev = position_std_dev;
        new_candidate.sol_status = sol_status;
        new_candidate.sol_type = sol_type;
        new_candidate.has_position = true;
        gps_pair_cache_.push_back(new_candidate);
    }
    
    // Clean up old unpaired data (timeout > 1s)
    gps_pair_cache_.erase(std::remove_if(gps_pair_cache_.begin(), gps_pair_cache_.end(),
        [&](const GpsPairCandidate& c) { 
            return (timestamp - c.timestamp) > GPS_PAIR_TIMEOUT; 
        }), gps_pair_cache_.end());
    
    // Limit cache size
    while (gps_pair_cache_.size() > GPS_PAIR_CACHE_MAX_SIZE) {
        gps_pair_cache_.pop_front();
    }
}

void SlamSystem::Spin() {
    // removed ros spin
}

void SlamSystem::ProcessGpsPair(const GpsPairCandidate& pair) {
    // Apply lever arm compensation to get IMU position
    // P_imu_utm = P_antenna_utm - R_enu_imu(heading) * lever_arm_imu
    // where R_enu_imu is rotation from IMU frame to ENU/UTM frame
    
    // Construct rotation matrix from heading
    // Heading is already in ENU definition (0=West, 90=North)
    // IMU frame: X-right, Y-forward, Z-up
    // ENU frame: X-East, Y-North, Z-Up
    SO3 R_enu_imu = SO3::rotZ(pair.heading_rad);
    
    // Transform lever arm from IMU frame to ENU/UTM frame
    Vec3d lever_arm_utm = R_enu_imu * gps_lever_arm_;
    
    // Compute IMU position in UTM
    Vec3d imu_utm = pair.antenna_utm - lever_arm_utm;
    
    // Create GpsFullObservation
    GpsFullObservation full_obs;
    full_obs.timestamp = pair.timestamp;
    full_obs.antenna_utm = pair.antenna_utm;
    full_obs.imu_utm = imu_utm;
    full_obs.heading_rad = pair.heading_rad;
    full_obs.pitch_rad = pair.pitch_rad;
    full_obs.position_std_dev = pair.position_std_dev;
    full_obs.sol_status = pair.sol_status;
    full_obs.sol_type = pair.sol_type;
    full_obs.heading_std_dev = pair.heading_std_dev;
    full_obs.pitch_std_dev = pair.pitch_std_dev;
    full_obs.satellite_tracked = pair.satellite_tracked;
    full_obs.has_position = true;
    full_obs.has_heading = true;
    full_obs.is_valid = true;
    
    // Update latest GPS full observation
    latest_gps_full_ = full_obs;
    has_valid_gps_full_ = true;
    
    // Append to GPS history for interpolation at keyframe timestamps
    gps_full_history_.push_back(full_obs);
    while (options_.online_mode_ &&
           gps_full_history_.size() > GPS_HISTORY_ONLINE_MAX_SIZE) {
        gps_full_history_.pop_front();
    }
    
    // Print paired GPS data in reference format
    AINFO << "[GPS_PAIRED] ==================== GPS Full Observation ====================";
    AINFO << "[GPS_PAIRED] Timestamp: " << std::fixed << std::setprecision(6) << pair.timestamp;
    AINFO << "[GPS_PAIRED] ";
    AINFO << "[GPS_PAIRED] --- Antenna (Mushroom Head) Position (UTM) ---";
    AINFO << "[GPS_PAIRED] antenna_utm_x: " << std::fixed << std::setprecision(6) << pair.antenna_utm.x();
    AINFO << "[GPS_PAIRED] antenna_utm_y: " << std::fixed << std::setprecision(6) << pair.antenna_utm.y();
    AINFO << "[GPS_PAIRED] antenna_utm_z: " << std::fixed << std::setprecision(6) << pair.antenna_utm.z();
    AINFO << "[GPS_PAIRED] ";
    AINFO << "[GPS_PAIRED] --- IMU Position (UTM, Lever-Arm Compensated) ---";
    AINFO << "[GPS_PAIRED] imu_utm_x: " << std::fixed << std::setprecision(6) << imu_utm.x();
    AINFO << "[GPS_PAIRED] imu_utm_y: " << std::fixed << std::setprecision(6) << imu_utm.y();
    AINFO << "[GPS_PAIRED] imu_utm_z: " << std::fixed << std::setprecision(6) << imu_utm.z();
    AINFO << "[GPS_PAIRED] ";
    AINFO << "[GPS_PAIRED] --- Orientation ---";
    AINFO << "[GPS_PAIRED] heading_rad: " << std::fixed << std::setprecision(6) << pair.heading_rad 
          << " (" << (pair.heading_rad * 180.0 / M_PI) << " deg)";
    AINFO << "[GPS_PAIRED] pitch_rad: " << std::fixed << std::setprecision(6) << pair.pitch_rad 
          << " (" << (pair.pitch_rad * 180.0 / M_PI) << " deg)";
    AINFO << "[GPS_PAIRED] ";
    AINFO << "[GPS_PAIRED] --- Lever Arm ---";
    AINFO << "[GPS_PAIRED] lever_arm_imu: [" << gps_lever_arm_.x() << ", " 
          << gps_lever_arm_.y() << ", " << gps_lever_arm_.z() << "]";
    AINFO << "[GPS_PAIRED] lever_arm_utm: [" << std::fixed << std::setprecision(6) 
          << lever_arm_utm.x() << ", " << lever_arm_utm.y() << ", " << lever_arm_utm.z() << "]";
    AINFO << "[GPS_PAIRED] ";
    AINFO << "[GPS_PAIRED] --- Quality Metrics ---";
    AINFO << "[GPS_PAIRED] sol_status: " << pair.sol_status << ", sol_type: " << pair.sol_type;
    AINFO << "[GPS_PAIRED] satellite_tracked: " << pair.satellite_tracked;
    AINFO << "[GPS_PAIRED] heading_std_dev: " << pair.heading_std_dev << " deg";
    AINFO << "[GPS_PAIRED] position_std_dev: [" << pair.position_std_dev.x() << ", " 
          << pair.position_std_dev.y() << ", " << pair.position_std_dev.z() << "]";
    AINFO << "[GPS_PAIRED] ================================================================";
    AINFO << " ";
    
    // Capture first GPS IMU position for offset calculation
    if (!first_gps_received_) {
        first_gps_utm_ = imu_utm;
        first_gps_received_ = true;
        
        AINFO << "[GPS_OFFSET] ============ First GPS Pair Captured ============";
        AINFO << "[GPS_OFFSET] This will be used for utm_alignment.txt offset calculation";
        AINFO << "[GPS_OFFSET] IMU (UTM): [" << imu_utm.x() << ", " 
              << imu_utm.y() << ", " << imu_utm.z() << "]";
        AINFO << "[GPS_OFFSET] ======================================================";
    }
}

bool SlamSystem::GetLatestGpsFullObservation(GpsFullObservation& obs) const {
    std::lock_guard<std::mutex> lock(mtx_gps_pair_);
    if (!has_valid_gps_full_) {
        return false;
    }
    obs = latest_gps_full_;
    return true;
}

bool SlamSystem::GetInterpolatedGpsAt(double timestamp, GpsFullObservation& obs) const {
    std::lock_guard<std::mutex> lock(mtx_gps_pair_);
    if (gps_full_history_.size() < 2) {
        return false;
    }

    // Find the closest observations before and after the target time
    bool has_before = false;
    bool has_after = false;
    GpsFullObservation before_obs;
    GpsFullObservation after_obs;

    for (const auto& g : gps_full_history_) {
        if (g.timestamp <= timestamp) {
            if (!has_before || g.timestamp > before_obs.timestamp) {
                before_obs = g;
                has_before = true;
            }
        }
        if (g.timestamp >= timestamp) {
            if (!has_after || g.timestamp < after_obs.timestamp) {
                after_obs = g;
                has_after = true;
            }
        }
    }

    if (!has_before || !has_after) {
        return false;
    }

    double dt = after_obs.timestamp - before_obs.timestamp;
    if (dt <= 0.0) {
        return false;
    }

    // Reject if interpolation gap is too large
    if ((timestamp - before_obs.timestamp) > GPS_INTERP_MAX_GAP ||
        (after_obs.timestamp - timestamp) > GPS_INTERP_MAX_GAP) {
        return false;
    }

    double alpha = (timestamp - before_obs.timestamp) / dt;

    // Linear interpolation for positions
    obs.timestamp = timestamp;
    obs.antenna_utm = before_obs.antenna_utm + alpha * (after_obs.antenna_utm - before_obs.antenna_utm);
    obs.imu_utm = before_obs.imu_utm + alpha * (after_obs.imu_utm - before_obs.imu_utm);

    // Heading / pitch interpolation with angle wrap-around handling
    auto interp_angle = [alpha](double a1, double a2) {
        double delta = a2 - a1;
        while (delta > M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        double a = a1 + alpha * delta;
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    obs.heading_rad = interp_angle(before_obs.heading_rad, after_obs.heading_rad);
    obs.pitch_rad = interp_angle(before_obs.pitch_rad, after_obs.pitch_rad);

    // Use the closer observation for quality metrics
    const GpsFullObservation& q = (timestamp - before_obs.timestamp <= after_obs.timestamp - timestamp)
                                      ? before_obs
                                      : after_obs;
    obs.position_std_dev = q.position_std_dev;
    obs.sol_status = q.sol_status;
    obs.sol_type = q.sol_type;
    obs.heading_std_dev = q.heading_std_dev;
    obs.pitch_std_dev = q.pitch_std_dev;
    obs.satellite_tracked = q.satellite_tracked;

    obs.has_position = q.has_position;
    obs.has_heading = q.has_heading;
    obs.is_valid = q.is_valid;

    return obs.is_valid;
}

void SlamSystem::AttachGpsToKeyframe(const Keyframe::Ptr& kf) {
    if (!kf) {
        return;
    }

    double t_kf = kf->GetTimestamp();
    SE3 lio_pose = kf->GetLIOPose();
    Vec3d lio_pos = lio_pose.translation();

    // Make GPS constraints sparser by keyframe travel distance
    if (gps_attach_min_kf_distance_ > 0.0 && has_last_gps_attach_lio_pos_) {
        double dist_since_last_gps = (lio_pos - last_gps_attach_lio_pos_).norm();
        if (dist_since_last_gps < gps_attach_min_kf_distance_) {
            ADEBUG << "[GPS_ATTACH] KF " << kf->GetID() << " skipped, dist="
                   << dist_since_last_gps << " m < min "
                   << gps_attach_min_kf_distance_ << " m";
            return;
        }
    }

    GpsFullObservation gps_obs;
    if (!GetInterpolatedGpsAt(t_kf, gps_obs)) {
        AWARN << "[GPS_ATTACH] KF " << kf->GetID() << " at t=" << std::fixed << std::setprecision(6) << t_kf 
              << " - FAILED to interpolate GPS (gap too large or insufficient data)";
        return;
    }

    if (!gps_obs.is_valid) {
        AWARN << "[GPS_ATTACH] KF " << kf->GetID() << " - GPS data is INVALID";
        return;
    }
    
    // ========== Re-compute IMU position using LIO yaw (more accurate) ==========
    // Extract yaw from LIO pose (ignore pitch/roll to avoid GPS pitch noise)
    Mat3d R_lio = lio_pose.rotationMatrix();
    double lio_yaw = std::atan2(R_lio(1, 0), R_lio(0, 0));
    
    // Construct rotation matrix using only yaw (pitch=0, roll=0)
    // R_enu_imu = Rz(yaw)
    SO3 R_enu_imu = SO3::rotZ(lio_yaw);
    
    // Transform lever arm from IMU frame to ENU/UTM frame
    Vec3d lever_arm_utm = R_enu_imu * gps_lever_arm_;
    
    // Re-compute IMU position in UTM using LIO yaw
    Vec3d imu_utm_corrected = gps_obs.antenna_utm - lever_arm_utm;
    
    AINFO << "[GPS_ATTACH] KF " << kf->GetID() << " at t=" << std::fixed << std::setprecision(6) << t_kf;
    AINFO << "[GPS_ATTACH]   Antenna UTM:       " << gps_obs.antenna_utm.transpose();
    AINFO << "[GPS_ATTACH]   IMU UTM (GPS yaw): " << gps_obs.imu_utm.transpose();
    AINFO << "[GPS_ATTACH]   IMU UTM (LIO yaw): " << imu_utm_corrected.transpose();
    AINFO << "[GPS_ATTACH]   LIO yaw (deg):     " << (lio_yaw * 180.0 / M_PI);
    AINFO << "[GPS_ATTACH]   Lever arm UTM:     " << lever_arm_utm.transpose();
    AINFO << "[GPS_ATTACH]   Correction (m):    " << (imu_utm_corrected - gps_obs.imu_utm).transpose();

    Keyframe::GpsData gps_data;
    gps_data.has_gps = true;
    gps_data.gps_utm_position = imu_utm_corrected;         // Use LIO-yaw-corrected IMU position
    gps_data.gps_std_dev = gps_obs.position_std_dev;       // std dev in meters
    gps_data.gps_heading_deg = gps_obs.heading_rad * 180.0 / M_PI;
    gps_data.heading_std_deg = gps_obs.heading_std_dev;
    gps_data.sol_type = gps_obs.sol_type;                  // Solution type (50 = NARROW_INT)

    kf->SetGpsData(gps_data);
    last_gps_attach_lio_pos_ = lio_pos;
    has_last_gps_attach_lio_pos_ = true;
}

void SlamSystem::OnKeyframeCreated(const Keyframe::Ptr& kf) {
    if (!kf || !gps_optimizer_) {
        return;
    }
    
    // Only collect keyframes during mapping; optimization deferred to FinalOptimize().
    // PeriodicOptimize was removed because it modifies pose_opt_ mid-SLAM, causing
    // inconsistency between optimization batches → point cloud ghosting in the final map.
    gps_optimizer_->AddKeyframe(kf);
}

}  // namespace lightning
