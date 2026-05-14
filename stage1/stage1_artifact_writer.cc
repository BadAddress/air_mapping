#include "modules/air_mapping/stage1/stage1_artifact_writer.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/debug_utils.h"
#include "pcl/io/pcd_io.h"

namespace apollo {
namespace air_mapping {
namespace stage1 {

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

void WritePoseCsv(std::ofstream& file, const lightning::SE3& pose) {
  const auto& translation = pose.translation();
  const auto& quaternion = pose.unit_quaternion();
  file << std::setprecision(9) << translation.x() << ","
       << translation.y() << "," << translation.z() << ","
       << quaternion.x() << "," << quaternion.y() << ","
       << quaternion.z() << "," << quaternion.w();
}

std::string FormatKeyframeCloudName(unsigned long id) {
  std::ostringstream stream;
  stream << std::setw(6) << std::setfill('0') << id << ".pcd";
  return stream.str();
}

std::string FormatKeyframeDataName(unsigned long id, const std::string& extension) {
  std::ostringstream stream;
  stream << std::setw(6) << std::setfill('0') << id << extension;
  return stream.str();
}

bool WriteMatrix6(const std::filesystem::path& path, const lightning::Mat6d& matrix) {
  std::ofstream file(path);
  if (!file.is_open()) {
    AERROR << "Failed to open covariance file: " << path.string();
    return false;
  }
  file << std::fixed << std::setprecision(12);
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      if (col > 0) {
        file << " ";
      }
      file << matrix(row, col);
    }
    file << "\n";
  }
  return true;
}

}  // namespace

bool Stage1ArtifactWriter::Write(
    const Stage1Config& config,
    const std::vector<lightning::Keyframe::Ptr>& keyframes,
    const std::vector<lightning::GpsFullObservation>& gps_history,
    lightning::CloudPtr preview_map) const {
  const std::filesystem::path output_dir(config.output.directory);
  const std::filesystem::path keyframe_dir = output_dir / "keyframes";
  const std::filesystem::path cloud_dir = keyframe_dir / "clouds";
  const std::filesystem::path covariance_dir = keyframe_dir / "covariance";
  const std::filesystem::path gps_dir = output_dir / "gps";
  const std::filesystem::path preview_dir = output_dir / "preview";

  if (!EnsureDirectory(keyframe_dir) || !EnsureDirectory(gps_dir)) {
    return false;
  }
  if (config.output.save_keyframe_clouds && !EnsureDirectory(cloud_dir)) {
    return false;
  }
  if (!EnsureDirectory(covariance_dir)) {
    return false;
  }
  if (config.output.save_preview_map && !EnsureDirectory(preview_dir)) {
    return false;
  }

  {
    std::ofstream manifest(output_dir / "manifest.yaml");
    if (!manifest.is_open()) {
      AERROR << "Failed to write manifest";
      return false;
    }
    manifest << "stage: stage1_lio\n";
    manifest << "map_name: " << config.map_name << "\n";
    manifest << "algorithm_config_path: " << config.algorithm_config_path << "\n";
    manifest << "records:\n";
    for (const auto& record : config.records) {
      manifest << "  - " << record << "\n";
    }
    manifest << "keyframe_count: " << keyframes.size() << "\n";
    manifest << "gps_full_count: " << gps_history.size() << "\n";
    manifest << "output_format_version: 1\n";
  }

  {
    std::ofstream poses(keyframe_dir / "poses_lio.tum");
    if (!poses.is_open()) {
      AERROR << "Failed to write poses_lio.tum";
      return false;
    }
    poses << lightning::DebugUtils::GenerateTUMHeader("air_mapping stage1 LIO poses");

    std::ofstream keyframe_csv(keyframe_dir / "keyframes.csv");
    if (!keyframe_csv.is_open()) {
      AERROR << "Failed to write keyframes.csv";
      return false;
    }
    keyframe_csv << "id,timestamp,x,y,z,qx,qy,qz,qw,cloud_path,covariance_path,has_gps\n";

    std::ofstream relative_csv(keyframe_dir / "relative_edges.csv");
    if (!relative_csv.is_open()) {
      AERROR << "Failed to write relative_edges.csv";
      return false;
    }
    relative_csv << "from_id,to_id,tx,ty,tz,qx,qy,qz,qw\n";

    for (size_t index = 0; index < keyframes.size(); ++index) {
      const auto& keyframe = keyframes[index];
      if (!keyframe) {
        continue;
      }
      const std::string cloud_name = FormatKeyframeCloudName(keyframe->GetID());
      const std::string cov_name = FormatKeyframeDataName(keyframe->GetID(), ".txt");
      const std::filesystem::path cov_path = covariance_dir / cov_name;
      const lightning::SE3 pose = keyframe->GetLIOPose();
      const auto gps_data = keyframe->GetGpsData();

      poses << lightning::DebugUtils::SE3ToTUMString(keyframe->GetTimestamp(), pose) << "\n";

      keyframe_csv << keyframe->GetID() << ","
                   << std::fixed << std::setprecision(9) << keyframe->GetTimestamp() << ",";
      WritePoseCsv(keyframe_csv, pose);
      keyframe_csv << ",keyframes/clouds/" << cloud_name
                   << ",keyframes/covariance/" << cov_name
                   << "," << (gps_data.has_gps ? 1 : 0) << "\n";

      WriteMatrix6(cov_path, keyframe->GetCovariance());

      if (config.output.save_keyframe_clouds && keyframe->GetCloud()) {
        const std::filesystem::path cloud_path = cloud_dir / cloud_name;
        if (pcl::io::savePCDFileBinaryCompressed(cloud_path.string(), *keyframe->GetCloud()) < 0) {
          AERROR << "Failed to save keyframe cloud: " << cloud_path.string();
          return false;
        }
      }

      if (index > 0) {
        const auto& previous = keyframes[index - 1];
        if (previous) {
          relative_csv << previous->GetID() << "," << keyframe->GetID() << ",";
          WritePoseCsv(relative_csv, keyframe->GetRelativeMotion());
          relative_csv << "\n";
        }
      }
    }
  }

  {
    std::ofstream gps_full(gps_dir / "gps_full.csv");
    if (!gps_full.is_open()) {
      AERROR << "Failed to write gps_full.csv";
      return false;
    }
    gps_full << "timestamp,antenna_x,antenna_y,antenna_z,imu_x,imu_y,imu_z,"
             << "heading_rad,pitch_rad,std_x,std_y,std_z,sol_status,sol_type,"
             << "heading_std_deg,pitch_std_deg,satellite_tracked\n";
    for (const auto& gps : gps_history) {
      gps_full << std::fixed << std::setprecision(9)
               << gps.timestamp << ","
               << gps.antenna_utm.x() << "," << gps.antenna_utm.y() << ","
               << gps.antenna_utm.z() << ","
               << gps.imu_utm.x() << "," << gps.imu_utm.y() << ","
               << gps.imu_utm.z() << ","
               << gps.heading_rad << "," << gps.pitch_rad << ","
               << gps.position_std_dev.x() << "," << gps.position_std_dev.y() << ","
               << gps.position_std_dev.z() << ","
               << gps.sol_status << "," << gps.sol_type << ","
               << gps.heading_std_dev << "," << gps.pitch_std_dev << ","
               << gps.satellite_tracked << "\n";
    }

    std::ofstream gps_assoc(gps_dir / "gps_keyframe_assoc.csv");
    if (!gps_assoc.is_open()) {
      AERROR << "Failed to write gps_keyframe_assoc.csv";
      return false;
    }
    gps_assoc << "keyframe_id,timestamp,has_gps,utm_x,utm_y,utm_z,std_x,std_y,std_z,"
              << "heading_deg,heading_std_deg,sol_type\n";
    for (const auto& keyframe : keyframes) {
      if (!keyframe) {
        continue;
      }
      const auto gps = keyframe->GetGpsData();
      gps_assoc << keyframe->GetID() << ","
                << std::fixed << std::setprecision(9) << keyframe->GetTimestamp() << ","
                << (gps.has_gps ? 1 : 0) << ","
                << gps.gps_utm_position.x() << "," << gps.gps_utm_position.y() << ","
                << gps.gps_utm_position.z() << ","
                << gps.gps_std_dev.x() << "," << gps.gps_std_dev.y() << ","
                << gps.gps_std_dev.z() << ","
                << gps.gps_heading_deg << "," << gps.heading_std_deg << ","
                << gps.sol_type << "\n";
    }
  }

  if (config.output.save_preview_map && preview_map && !preview_map->empty()) {
    const std::filesystem::path preview_path = preview_dir / "lio_global_preview.pcd";
    if (pcl::io::savePCDFileBinaryCompressed(preview_path.string(), *preview_map) < 0) {
      AERROR << "Failed to save preview map: " << preview_path.string();
      return false;
    }
  }

  AINFO << "Stage1 artifacts written to " << output_dir.string();
  return true;
}

}  // namespace stage1
}  // namespace air_mapping
}  // namespace apollo
