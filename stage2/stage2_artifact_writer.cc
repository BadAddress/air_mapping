#include "modules/air_mapping/stage2/stage2_artifact_writer.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "pcl/io/pcd_io.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/debug_utils.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

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

std::string FormatKeyframeDataName(unsigned long id,
                                   const std::string& extension) {
  std::ostringstream stream;
  stream << std::setw(6) << std::setfill('0') << id << extension;
  return stream.str();
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

void WriteMatrix4(std::ofstream& file, const Eigen::Matrix4d& matrix) {
  file << std::fixed << std::setprecision(12);
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (col > 0) {
        file << " ";
      }
      file << matrix(row, col);
    }
    file << "\n";
  }
}

}  // namespace

bool Stage2ArtifactWriter::Write(
    const Stage2Config& config, const Stage1Dataset& dataset,
    const std::vector<AlignmentAnchor>& anchors,
    const std::vector<GpsSegmentSummary>& segments,
    const Stage2AlignmentResult& result,
    const LeverArmCalibrationResult& lever_arm_result,
    lightning::CloudPtr preview_map) const {
  const std::filesystem::path output_dir(config.output_dir);
  const std::filesystem::path keyframe_dir = output_dir / "keyframes";
  const std::filesystem::path alignment_dir = output_dir / "alignment";
  const std::filesystem::path diagnostics_dir = output_dir / "diagnostics";
  const std::filesystem::path preview_dir = output_dir / "preview";

  if (!EnsureDirectory(output_dir) || !EnsureDirectory(keyframe_dir) ||
      !EnsureDirectory(alignment_dir) || !EnsureDirectory(diagnostics_dir)) {
    return false;
  }
  if (config.output.save_preview_map && !EnsureDirectory(preview_dir)) {
    return false;
  }

  {
    std::ofstream manifest(output_dir / "manifest.yaml");
    if (!manifest.is_open()) {
      AERROR << "Failed to write stage2 manifest";
      return false;
    }
    manifest << "stage: stage2_graph_opt\n";
    manifest << "map_name: " << config.map_name << "\n";
    manifest << "source_stage1_dir: " << config.input_dir << "\n";
    manifest << "source_config_path: " << config.source_config_path << "\n";
    manifest << "keyframe_count: " << dataset.keyframes.size() << "\n";
    manifest << "gps_anchor_count: " << result.anchor_count << "\n";
    manifest << "gps_segment_count: " << result.segment_count << "\n";
    manifest << "lever_arm_calibration_enabled: "
             << (lever_arm_result.enabled ? "true" : "false") << "\n";
    manifest << "lever_arm_calibration_success: "
             << (lever_arm_result.success ? "true" : "false") << "\n";
    manifest << "lever_arm_orientation_model: "
             << (lever_arm_result.use_full_lio_orientation ? "full_lio"
                                                           : "lio_yaw_only")
             << "\n";
    manifest << "lever_arm_heading_bias_enabled: "
             << (lever_arm_result.estimate_heading_bias ? "true" : "false")
             << "\n";
    manifest << "lever_arm_initial: [" << std::fixed << std::setprecision(9)
             << lever_arm_result.initial_lever_arm.x() << ", "
             << lever_arm_result.initial_lever_arm.y() << ", "
             << lever_arm_result.initial_lever_arm.z() << "]\n";
    manifest << "lever_arm_optimized: [" << std::fixed << std::setprecision(9)
             << lever_arm_result.optimized_lever_arm.x() << ", "
             << lever_arm_result.optimized_lever_arm.y() << ", "
             << lever_arm_result.optimized_lever_arm.z() << "]\n";
    manifest << "lever_arm_correction: [" << std::fixed << std::setprecision(9)
             << lever_arm_result.correction.x() << ", "
             << lever_arm_result.correction.y() << ", "
             << lever_arm_result.correction.z() << "]\n";
    manifest << "lever_arm_heading_bias_deg: " << std::fixed
             << std::setprecision(9)
             << lever_arm_result.heading_bias_rad * 180.0 / M_PI << "\n";
    manifest << "alignment_frame: utm_local\n";
    manifest << "alignment_model: "
             << (result.yaw_only ? "yaw_only_translation" : "se3") << "\n";
    manifest << "utm_origin: [" << std::fixed << std::setprecision(9)
             << result.utm_origin.x() << ", " << result.utm_origin.y() << ", "
             << result.utm_origin.z() << "]\n";
    manifest << "output_format_version: 1\n";
  }

  {
    std::ofstream matrix_file(alignment_dir / "se3_lio_to_utm_local.txt");
    if (!matrix_file.is_open()) {
      AERROR << "Failed to write alignment matrix";
      return false;
    }
    matrix_file << "# T_utm_local_lio, row-major 4x4 matrix\n";
    WriteMatrix4(matrix_file, result.lio_to_utm_local.matrix());
  }

  {
    std::ofstream origin_file(alignment_dir / "utm_origin.txt");
    if (!origin_file.is_open()) {
      AERROR << "Failed to write UTM origin";
      return false;
    }
    origin_file << "# utm_origin_x utm_origin_y utm_origin_z\n";
    origin_file << std::fixed << std::setprecision(12) << result.utm_origin.x()
                << " " << result.utm_origin.y() << " " << result.utm_origin.z()
                << "\n";
  }

  {
    std::ofstream lever_summary(diagnostics_dir / "lever_arm_calibration.csv");
    if (!lever_summary.is_open()) {
      AERROR << "Failed to write lever arm calibration summary";
      return false;
    }
    lever_summary
        << "enabled,success,orientation_model,status_message,candidate_count,"
        << "selected_count,iterations,initial_x,initial_y,initial_z,"
        << "correction_x,"
        << "correction_y,correction_z,optimized_x,optimized_y,optimized_z,"
        << "correction_norm_m,heading_bias_deg,weighted_cost_initial,"
        << "weighted_cost_optimized,mean_residual_initial_xy_m,"
        << "mean_residual_optimized_xy_m,max_residual_initial_xy_m,"
        << "max_residual_optimized_xy_m\n";
    lever_summary << (lever_arm_result.enabled ? 1 : 0) << ","
                  << (lever_arm_result.success ? 1 : 0) << ","
                  << (lever_arm_result.use_full_lio_orientation ? "full_lio"
                                                                : "lio_yaw_only")
                  << ","
                  << lever_arm_result.status_message << ","
                  << lever_arm_result.candidate_count << ","
                  << lever_arm_result.selected_count << ","
                  << lever_arm_result.iterations << "," << std::fixed
                  << std::setprecision(9)
                  << lever_arm_result.initial_lever_arm.x() << ","
                  << lever_arm_result.initial_lever_arm.y() << ","
                  << lever_arm_result.initial_lever_arm.z() << ","
                  << lever_arm_result.correction.x() << ","
                  << lever_arm_result.correction.y() << ","
                  << lever_arm_result.correction.z() << ","
                  << lever_arm_result.optimized_lever_arm.x() << ","
                  << lever_arm_result.optimized_lever_arm.y() << ","
                  << lever_arm_result.optimized_lever_arm.z() << ","
                  << lever_arm_result.correction_norm_m << ","
                  << lever_arm_result.heading_bias_rad * 180.0 / M_PI << ","
                  << lever_arm_result.weighted_cost_initial << ","
                  << lever_arm_result.weighted_cost_optimized << ","
                  << lever_arm_result.mean_residual_initial_xy_m << ","
                  << lever_arm_result.mean_residual_optimized_xy_m << ","
                  << lever_arm_result.max_residual_initial_xy_m << ","
                  << lever_arm_result.max_residual_optimized_xy_m << "\n";
  }

  {
    std::ofstream sample_file(diagnostics_dir / "lever_arm_samples.csv");
    if (!sample_file.is_open()) {
      AERROR << "Failed to write lever arm sample diagnostics";
      return false;
    }
    sample_file
        << "keyframe_id,timestamp,selected,reject_reason,weight,std_x,std_y,"
        << "std_z,max_interp_gap_s,antenna_x,antenna_y,antenna_z,lio_x,lio_y,"
        << "lio_z,pred_initial_x,pred_initial_y,pred_initial_z,"
        << "pred_optimized_x,pred_optimized_y,pred_optimized_z,"
        << "residual_initial_x,residual_initial_y,residual_initial_z,"
        << "residual_optimized_x,residual_optimized_y,residual_optimized_z,"
        << "residual_initial_xy_m,residual_optimized_xy_m,lio_raw_yaw_rad,"
        << "lio_opt_yaw_rad,gnss_heading_rad,gnss_pitch_rad,"
        << "gnss_lio_yaw_diff_deg,heading_std_deg,pitch_std_deg,sol_status,"
        << "sol_type,satellite_tracked\n";
    for (const auto& sample : lever_arm_result.samples) {
      sample_file << sample.keyframe_id << "," << std::fixed
                  << std::setprecision(9) << sample.timestamp << ","
                  << (sample.selected ? 1 : 0) << "," << sample.reject_reason
                  << "," << sample.weight << "," << sample.gps_std_dev.x()
                  << "," << sample.gps_std_dev.y() << ","
                  << sample.gps_std_dev.z() << "," << sample.max_interp_gap_s
                  << "," << sample.antenna_utm_position.x() << ","
                  << sample.antenna_utm_position.y() << ","
                  << sample.antenna_utm_position.z() << ","
                  << sample.lio_position.x() << "," << sample.lio_position.y()
                  << "," << sample.lio_position.z() << ","
                  << sample.predicted_antenna_initial.x() << ","
                  << sample.predicted_antenna_initial.y() << ","
                  << sample.predicted_antenna_initial.z() << ","
                  << sample.predicted_antenna_optimized.x() << ","
                  << sample.predicted_antenna_optimized.y() << ","
                  << sample.predicted_antenna_optimized.z() << ","
                  << sample.residual_initial.x() << ","
                  << sample.residual_initial.y() << ","
                  << sample.residual_initial.z() << ","
                  << sample.residual_optimized.x() << ","
                  << sample.residual_optimized.y() << ","
                  << sample.residual_optimized.z() << ","
                  << sample.residual_initial_xy_m << ","
                  << sample.residual_optimized_xy_m << ","
                  << sample.lio_raw_yaw_rad << "," << sample.lio_opt_yaw_rad
                  << "," << sample.gnss_heading_rad << ","
                  << sample.gnss_pitch_rad << ","
                  << sample.gnss_lio_yaw_diff_deg << ","
                  << sample.heading_std_deg << ","
                  << sample.pitch_std_deg << "," << sample.sol_status << ","
                  << sample.sol_type << "," << sample.satellite_tracked
                  << "\n";
    }
  }

  {
    std::ofstream summary_file(diagnostics_dir / "alignment_summary.csv");
    if (!summary_file.is_open()) {
      AERROR << "Failed to write alignment summary";
      return false;
    }
    summary_file
        << "alignment_model,anchor_count,segment_count,mean_residual_before_m,"
        << "max_residual_before_m,mean_residual_after_m,max_residual_after_m,"
        << "roll_deg,pitch_deg,yaw_deg\n";
    summary_file << (result.yaw_only ? "yaw_only_translation" : "se3") << ","
                 << result.anchor_count << "," << result.segment_count << ","
                 << std::fixed << std::setprecision(9)
                 << result.mean_residual_before_m << ","
                 << result.max_residual_before_m << ","
                 << result.mean_residual_after_m << ","
                 << result.max_residual_after_m << "," << result.roll_deg << ","
                 << result.pitch_deg << "," << result.yaw_deg << "\n";
  }

  {
    std::ofstream segment_file(diagnostics_dir / "gps_segments.csv");
    if (!segment_file.is_open()) {
      AERROR << "Failed to write GPS segment diagnostics";
      return false;
    }
    segment_file
        << "segment_id,first_keyframe_id,last_keyframe_id,anchor_count,"
        << "length_xy_m,mean_abs_z_smoothing_delta_m,"
        << "max_abs_z_smoothing_delta_m\n";
    for (const auto& segment : segments) {
      segment_file << segment.segment_id << "," << segment.first_keyframe_id
                   << "," << segment.last_keyframe_id << ","
                   << segment.anchor_count << "," << std::fixed
                   << std::setprecision(9) << segment.length_xy_m << ","
                   << segment.mean_abs_z_smoothing_delta_m << ","
                   << segment.max_abs_z_smoothing_delta_m << "\n";
    }
  }

  {
    std::ofstream anchor_file(diagnostics_dir / "alignment_anchors.csv");
    if (!anchor_file.is_open()) {
      AERROR << "Failed to write alignment anchor diagnostics";
      return false;
    }
    anchor_file << "keyframe_id,timestamp,segment_id,weight,lio_x,lio_y,lio_z,"
                << "gps_utm_x,gps_utm_y,gps_utm_z,gps_smooth_utm_x,"
                << "gps_smooth_utm_y,gps_smooth_utm_z,std_x,std_y,std_z,"
                << "residual_before_m,residual_after_m\n";
    for (const auto& anchor : anchors) {
      anchor_file << anchor.keyframe_id << "," << std::fixed
                  << std::setprecision(9) << anchor.timestamp << ","
                  << anchor.segment_id << "," << anchor.weight << ","
                  << anchor.lio_position.x() << "," << anchor.lio_position.y()
                  << "," << anchor.lio_position.z() << ","
                  << anchor.gps_utm_position.x() << ","
                  << anchor.gps_utm_position.y() << ","
                  << anchor.gps_utm_position.z() << ","
                  << anchor.smoothed_gps_utm_position.x() << ","
                  << anchor.smoothed_gps_utm_position.y() << ","
                  << anchor.smoothed_gps_utm_position.z() << ","
                  << anchor.gps_std_dev.x() << "," << anchor.gps_std_dev.y()
                  << "," << anchor.gps_std_dev.z() << ","
                  << anchor.residual_before_m << "," << anchor.residual_after_m
                  << "\n";
    }
  }

  {
    std::ofstream poses_local(keyframe_dir / "poses_opt.tum");
    std::ofstream poses_utm(keyframe_dir / "poses_opt_utm.tum");
    std::ofstream poses_lio(keyframe_dir / "poses_lio.tum");
    std::ofstream keyframe_csv(keyframe_dir / "keyframes_opt.csv");
    std::ofstream delta_csv(keyframe_dir / "pose_delta.csv");
    std::ofstream relative_csv(keyframe_dir / "relative_edges.csv");
    if (!poses_local.is_open() || !poses_utm.is_open() ||
        !poses_lio.is_open() || !keyframe_csv.is_open() ||
        !delta_csv.is_open() || !relative_csv.is_open()) {
      AERROR << "Failed to write stage2 keyframe outputs";
      return false;
    }

    poses_local << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage2 optimized poses in UTM-local frame");
    poses_utm << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage2 optimized poses in absolute UTM frame");
    poses_lio << lightning::DebugUtils::GenerateTUMHeader(
        "air_mapping stage2 source LIO poses");

    keyframe_csv
        << "id,timestamp,lio_x,lio_y,lio_z,lio_qx,lio_qy,lio_qz,lio_qw,"
        << "opt_x,opt_y,opt_z,opt_qx,opt_qy,opt_qz,opt_qw,"
        << "utm_x,utm_y,utm_z,utm_qx,utm_qy,utm_qz,utm_qw,"
        << "source_stage1_dir,source_cloud_path,source_covariance_path,"
        << "has_gps\n";
    delta_csv << "id,timestamp,dx,dy,dz,rotation_delta_deg\n";
    relative_csv << "from_id,to_id,tx,ty,tz,qx,qy,qz,qw\n";

    for (size_t keyframe_index = 0; keyframe_index < dataset.keyframes.size();
         ++keyframe_index) {
      const auto& keyframe = dataset.keyframes[keyframe_index];
      if (!keyframe) {
        continue;
      }
      const auto lio_pose = keyframe->GetLIOPose();
      const auto opt_pose = keyframe->GetOptPose();
      const auto utm_pose = ToAbsoluteUtmPose(opt_pose, result.utm_origin);
      poses_local << lightning::DebugUtils::SE3ToTUMString(
                         keyframe->GetTimestamp(), opt_pose)
                  << "\n";
      poses_utm << lightning::DebugUtils::SE3ToTUMString(
                       keyframe->GetTimestamp(), utm_pose)
                << "\n";
      poses_lio << lightning::DebugUtils::SE3ToTUMString(
                       keyframe->GetTimestamp(), lio_pose)
                << "\n";

      keyframe_csv << keyframe->GetID() << "," << std::fixed
                   << std::setprecision(9) << keyframe->GetTimestamp() << ",";
      WritePoseCsv(keyframe_csv, lio_pose);
      keyframe_csv << ",";
      WritePoseCsv(keyframe_csv, opt_pose);
      keyframe_csv << ",";
      WritePoseCsv(keyframe_csv, utm_pose);
      const std::string cloud_path =
          keyframe_index < dataset.keyframe_cloud_paths.size()
              ? dataset.keyframe_cloud_paths[keyframe_index]
              : ("keyframes/clouds/" +
                 FormatKeyframeDataName(keyframe->GetID(), ".pcd"));
      const std::string covariance_path =
          keyframe_index < dataset.keyframe_covariance_paths.size()
              ? dataset.keyframe_covariance_paths[keyframe_index]
              : ("keyframes/covariance/" +
                 FormatKeyframeDataName(keyframe->GetID(), ".txt"));
      keyframe_csv << "," << config.input_dir << "," << cloud_path << ","
                   << covariance_path << ","
                   << (keyframe->GetGpsData().has_gps ? 1 : 0) << "\n";

      const auto pose_delta = lio_pose.inverse() * opt_pose;
      delta_csv << keyframe->GetID() << "," << std::fixed
                << std::setprecision(9) << keyframe->GetTimestamp() << ","
                << (opt_pose.translation().x() - lio_pose.translation().x())
                << ","
                << (opt_pose.translation().y() - lio_pose.translation().y())
                << ","
                << (opt_pose.translation().z() - lio_pose.translation().z())
                << "," << pose_delta.so3().log().norm() * 180.0 / M_PI << "\n";

      if (keyframe_index > 0 && dataset.keyframes[keyframe_index - 1]) {
        const auto& previous = dataset.keyframes[keyframe_index - 1];
        relative_csv << previous->GetID() << "," << keyframe->GetID() << ",";
        WritePoseCsv(relative_csv,
                     previous->GetOptPose().inverse() * keyframe->GetOptPose());
        relative_csv << "\n";
      }
    }
  }

  if (config.output.save_preview_map && preview_map && !preview_map->empty()) {
    const auto preview_path = preview_dir / "optimized_global_preview.pcd";
    if (pcl::io::savePCDFileBinaryCompressed(preview_path.string(),
                                             *preview_map) < 0) {
      AERROR << "Failed to save optimized preview map: "
             << preview_path.string();
      return false;
    }
  }

  AINFO << "Stage2 artifacts written to " << output_dir.string();
  return true;
}

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
