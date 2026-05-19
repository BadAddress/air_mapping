#include "modules/air_mapping/stage3/stage3_artifact_writer.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>

#include "pcl/io/pcd_io.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/debug_utils.h"

namespace apollo {
namespace air_mapping {
namespace stage3 {

namespace {

bool EnsureDirectory(const std::filesystem::path& path) {
  std::error_code error;
  std::filesystem::create_directories(path, error);
  if (error) {
    AERROR << "Failed to create directory: " << path.string()
           << ", error: " << error.message();
    return false;
  }
  return true;
}

std::filesystem::path FindModuleRoot() {
  std::error_code error;
  std::filesystem::path path = std::filesystem::current_path(error);
  while (!error && !path.empty()) {
    if (std::filesystem::exists(path / "cyberfile.xml", error) &&
        std::filesystem::exists(path / "stage1", error) &&
        std::filesystem::exists(path / "stage2", error)) {
      return path;
    }
    const auto candidate = path / "modules" / "air_mapping";
    if (std::filesystem::exists(candidate / "cyberfile.xml", error)) {
      return candidate;
    }
    if (path == path.root_path()) {
      break;
    }
    path = path.parent_path();
  }
  return std::filesystem::current_path();
}

std::filesystem::path ResolveOutputPath(const std::string& path) {
  const std::filesystem::path candidate(path);
  std::error_code error;
  const auto parent = candidate.parent_path();
  if (parent.empty() || std::filesystem::exists(parent, error)) {
    return candidate;
  }

  const std::string prefix = "/apollo_workspace/modules/air_mapping/";
  if (path.rfind(prefix, 0) == 0) {
    return FindModuleRoot() / path.substr(prefix.size());
  }
  return candidate;
}

void WritePoseCsv(std::ofstream& file, const lightning::SE3& pose) {
  const auto& translation = pose.translation();
  const auto& quaternion = pose.unit_quaternion();
  file << std::setprecision(12) << translation.x() << "," << translation.y()
       << "," << translation.z() << "," << quaternion.x() << ","
       << quaternion.y() << "," << quaternion.z() << "," << quaternion.w();
}

lightning::SE3 ToAbsoluteUtmPose(const lightning::SE3& local_pose,
                                 const lightning::Vec3d& utm_origin) {
  return lightning::SE3(local_pose.so3(),
                        local_pose.translation() + utm_origin);
}

}  // namespace

bool Stage3ArtifactWriter::Write(
    const Stage3Config& config, const Stage2Dataset& dataset,
    const Stage3RefineSummary& summary,
    const std::vector<Stage3OutageBlock>& outage_blocks,
    lightning::CloudPtr preview_map) const {
  const std::filesystem::path output_dir = ResolveOutputPath(config.output_dir);
  const std::filesystem::path keyframe_dir = output_dir / "keyframes";
  const std::filesystem::path diagnostics_dir = output_dir / "diagnostics";
  const std::filesystem::path preview_dir = output_dir / "preview";

  if (!EnsureDirectory(output_dir) || !EnsureDirectory(keyframe_dir) ||
      !EnsureDirectory(diagnostics_dir)) {
    return false;
  }
  if (config.output.save_preview_map && !EnsureDirectory(preview_dir)) {
    return false;
  }
  if (dataset.records.size() != dataset.keyframes.size()) {
    AERROR << "Invalid stage3 dataset size: records=" << dataset.records.size()
           << ", keyframes=" << dataset.keyframes.size();
    return false;
  }

  {
    std::ofstream manifest(output_dir / "manifest.yaml");
    if (!manifest.is_open()) {
      AERROR << "Failed to write stage3 manifest";
      return false;
    }
    manifest << "stage: stage3_graph_refine\n";
    manifest << "map_name: " << config.map_name << "\n";
    manifest << "source_stage2_dir: " << config.input_dir << "\n";
    manifest << "source_stage1_dir: " << dataset.source_stage1_dir << "\n";
    manifest << "source_config_path: " << dataset.source_config_path << "\n";
    manifest << "keyframe_count: " << summary.keyframe_count << "\n";
    manifest << "relative_edge_count: " << summary.relative_edge_count << "\n";
    manifest << "gps_prior_count: " << summary.gps_prior_count << "\n";
    manifest << "outage_block_count: " << summary.outage_block_count << "\n";
    manifest << "outage_icp_prior_count: " << summary.outage_icp_prior_count
             << "\n";
    manifest << "alignment_frame: utm_local\n";
    manifest << "utm_origin: [" << std::fixed << std::setprecision(9)
             << dataset.utm_origin.x() << ", " << dataset.utm_origin.y() << ", "
             << dataset.utm_origin.z() << "]\n";
    manifest << "output_format_version: 1\n";
  }

  {
    std::ofstream summary_file(diagnostics_dir / "refine_summary.csv");
    if (!summary_file.is_open()) {
      AERROR << "Failed to write stage3 refine summary";
      return false;
    }
    summary_file
        << "success,keyframe_count,relative_edge_count,gps_prior_count,"
        << "outage_block_count,outage_icp_prior_count,optimizer_iterations,"
        << "chi2_before,chi2_after,"
        << "mean_gps_residual_before_m,max_gps_residual_before_m,"
        << "mean_gps_residual_after_m,max_gps_residual_after_m,"
        << "mean_pose_delta_m,max_pose_delta_m,mean_rotation_delta_deg,"
        << "max_rotation_delta_deg\n";
    summary_file << (summary.success ? 1 : 0) << "," << summary.keyframe_count
                 << "," << summary.relative_edge_count << ","
                 << summary.gps_prior_count << "," << summary.outage_block_count
                 << "," << summary.outage_icp_prior_count << ","
                 << summary.optimizer_iterations << "," << std::fixed
                 << std::setprecision(9) << summary.chi2_before << ","
                 << summary.chi2_after << ","
                 << summary.mean_gps_residual_before_m << ","
                 << summary.max_gps_residual_before_m << ","
                 << summary.mean_gps_residual_after_m << ","
                 << summary.max_gps_residual_after_m << ","
                 << summary.mean_pose_delta_m << "," << summary.max_pose_delta_m
                 << "," << summary.mean_rotation_delta_deg << ","
                 << summary.max_rotation_delta_deg << "\n";
  }

  {
    std::ofstream block_file(diagnostics_dir / "outage_blocks.csv");
    if (!block_file.is_open()) {
      AERROR << "Failed to write stage3 outage block diagnostics";
      return false;
    }
    block_file
        << "block_id,start_keyframe_id,end_keyframe_id,representative_keyframe_"
           "id,"
        << "left_anchor_index,right_anchor_index,path_length_m,icp_valid,"
        << "icp_fitness,source_keyframes,target_keyframes,prior_x,prior_y,"
        << "prior_z,prior_qx,prior_qy,prior_qz,prior_qw\n";
    for (size_t i = 0; i < outage_blocks.size(); ++i) {
      const auto& block = outage_blocks[i];
      const auto start_id = block.start_index < dataset.keyframes.size() &&
                                    dataset.keyframes[block.start_index]
                                ? dataset.keyframes[block.start_index]->GetID()
                                : 0UL;
      const auto end_id = block.end_index < dataset.keyframes.size() &&
                                  dataset.keyframes[block.end_index]
                              ? dataset.keyframes[block.end_index]->GetID()
                              : 0UL;
      const auto representative_id =
          block.representative_index < dataset.keyframes.size() &&
                  dataset.keyframes[block.representative_index]
              ? dataset.keyframes[block.representative_index]->GetID()
              : 0UL;
      block_file << i << "," << start_id << "," << end_id << ","
                 << representative_id << "," << block.left_anchor_index << ","
                 << block.right_anchor_index << "," << std::fixed
                 << std::setprecision(9) << block.path_length_m << ","
                 << (block.icp_valid ? 1 : 0) << "," << block.icp_fitness << ","
                 << block.source_keyframes << "," << block.target_keyframes
                 << ",";
      WritePoseCsv(block_file, block.representative_prior);
      block_file << "\n";
    }
  }

  {
    std::ofstream prior_file(diagnostics_dir / "gps_priors.csv");
    if (!prior_file.is_open()) {
      AERROR << "Failed to write stage3 GPS prior diagnostics";
      return false;
    }
    prior_file
        << "keyframe_id,timestamp,segment_id,stage2_weight,gps_smooth_utm_x,"
        << "gps_smooth_utm_y,gps_smooth_utm_z,std_x,std_y,std_z,"
        << "stage3_used,stage3_info_x,stage3_info_y,stage3_info_z,"
        << "stage3_ramp_scale,"
        << "stage2_reported_residual_m,stage2_recomputed_residual_m,"
        << "stage3_residual_m\n";
    for (const auto& anchor : dataset.gps_anchors) {
      prior_file << anchor.keyframe_id << "," << std::fixed
                 << std::setprecision(9) << anchor.timestamp << ","
                 << anchor.segment_id << "," << anchor.stage2_weight << ","
                 << anchor.gps_smooth_utm_position.x() << ","
                 << anchor.gps_smooth_utm_position.y() << ","
                 << anchor.gps_smooth_utm_position.z() << ","
                 << anchor.gps_std_dev.x() << "," << anchor.gps_std_dev.y()
                 << "," << anchor.gps_std_dev.z() << ","
                 << (anchor.stage3_used ? 1 : 0) << ","
                 << anchor.stage3_information_diag.x() << ","
                 << anchor.stage3_information_diag.y() << ","
                 << anchor.stage3_information_diag.z() << ","
                 << anchor.stage3_ramp_scale << "," << anchor.residual_after_m
                 << "," << anchor.residual_stage2_recomputed_m << ","
                 << anchor.residual_stage3_m << "\n";
    }
  }

  {
    std::ofstream poses_refined(keyframe_dir / "poses_refined.tum");
    std::ofstream poses_refined_utm(keyframe_dir / "poses_refined_utm.tum");
    std::ofstream poses_stage2(keyframe_dir / "poses_stage2.tum");
    std::ofstream keyframe_csv(keyframe_dir / "keyframes_refined.csv");
    std::ofstream delta_csv(keyframe_dir / "pose_delta.csv");
    std::ofstream relative_csv(keyframe_dir / "relative_edges.csv");
    if (!poses_refined.is_open() || !poses_refined_utm.is_open() ||
        !poses_stage2.is_open() || !keyframe_csv.is_open() ||
        !delta_csv.is_open() || !relative_csv.is_open()) {
      AERROR << "Failed to write stage3 keyframe outputs";
      return false;
    }

    poses_refined << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage3 refined poses in UTM-local frame");
    poses_refined_utm << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage3 refined poses in absolute UTM frame");
    poses_stage2 << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage3 source stage2 poses");

    keyframe_csv
        << "id,timestamp,stage2_x,stage2_y,stage2_z,stage2_qx,stage2_qy,"
        << "stage2_qz,stage2_qw,refined_x,refined_y,refined_z,refined_qx,"
        << "refined_qy,refined_qz,refined_qw,utm_x,utm_y,utm_z,utm_qx,"
        << "utm_qy,utm_qz,utm_qw,source_stage1_dir,source_cloud_path,"
        << "source_covariance_path,has_gps\n";
    delta_csv << "id,timestamp,dx,dy,dz,rotation_delta_deg\n";
    relative_csv << "from_id,to_id,tx,ty,tz,qx,qy,qz,qw\n";

    for (size_t i = 0; i < dataset.keyframes.size(); ++i) {
      const auto& keyframe = dataset.keyframes[i];
      if (!keyframe) {
        continue;
      }
      const auto stage2_pose = dataset.records[i].stage2_pose;
      const auto refined_pose = keyframe->GetOptPose();
      const auto utm_pose = ToAbsoluteUtmPose(refined_pose, dataset.utm_origin);

      poses_refined << lightning::DebugUtils::SE3ToTUMString(
                           keyframe->GetTimestamp(), refined_pose)
                    << "\n";
      poses_refined_utm << lightning::DebugUtils::SE3ToTUMString(
                               keyframe->GetTimestamp(), utm_pose)
                        << "\n";
      poses_stage2 << lightning::DebugUtils::SE3ToTUMString(
                          keyframe->GetTimestamp(), stage2_pose)
                   << "\n";

      keyframe_csv << keyframe->GetID() << "," << std::fixed
                   << std::setprecision(9) << keyframe->GetTimestamp() << ",";
      WritePoseCsv(keyframe_csv, stage2_pose);
      keyframe_csv << ",";
      WritePoseCsv(keyframe_csv, refined_pose);
      keyframe_csv << ",";
      WritePoseCsv(keyframe_csv, utm_pose);
      keyframe_csv << "," << dataset.records[i].source_stage1_dir << ","
                   << dataset.records[i].source_cloud_path << ","
                   << dataset.records[i].source_covariance_path << ","
                   << (dataset.records[i].has_gps ? 1 : 0) << "\n";

      const auto pose_delta = stage2_pose.inverse() * refined_pose;
      delta_csv
          << keyframe->GetID() << "," << std::fixed << std::setprecision(9)
          << keyframe->GetTimestamp() << ","
          << (refined_pose.translation().x() - stage2_pose.translation().x())
          << ","
          << (refined_pose.translation().y() - stage2_pose.translation().y())
          << ","
          << (refined_pose.translation().z() - stage2_pose.translation().z())
          << "," << pose_delta.so3().log().norm() * 180.0 / M_PI << "\n";

      if (i > 0 && dataset.keyframes[i - 1]) {
        relative_csv << dataset.keyframes[i - 1]->GetID() << ","
                     << keyframe->GetID() << ",";
        WritePoseCsv(
            relative_csv,
            dataset.keyframes[i - 1]->GetOptPose().inverse() * refined_pose);
        relative_csv << "\n";
      }
    }
  }

  if (config.output.save_preview_map && preview_map && !preview_map->empty()) {
    const auto preview_path = preview_dir / "refined_global_preview.pcd";
    if (pcl::io::savePCDFileBinaryCompressed(preview_path.string(),
                                             *preview_map) < 0) {
      AERROR << "Failed to save refined preview map: " << preview_path.string();
      return false;
    }
  }

  AINFO << "Stage3 artifacts written to " << output_dir.string();
  return true;
}

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
