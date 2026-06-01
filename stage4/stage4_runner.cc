#include "modules/air_mapping/stage4/stage4_runner.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "pcl/common/transforms.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/system/common/artifact_utils.h"
#include "modules/air_mapping/system/common/point_def.h"
#include "modules/air_mapping/system/common/run_config.h"

namespace apollo {
namespace air_mapping {
namespace stage4 {

namespace {

constexpr const char* kApolloAirMappingPrefix =
    "/apollo_workspace/modules/air_mapping/";

struct Stage4InputMetadata {
  std::string stage3_manifest_path;
  std::string stage3_generated_at;
  std::string source_stage2_dir;
  std::string source_stage2_manifest;
  std::string source_stage2_generated_at;
  std::string source_stage1_dir;
  std::string source_stage1_manifest;
  std::string source_stage1_generated_at;
  std::string source_config_path;
  std::vector<std::string> dataset_sources;
  std::vector<std::string> expanded_records;
  lightning::Vec3d utm_origin = lightning::Vec3d::Zero();
  bool has_utm_origin = false;
};

struct Stage4KeyframeRecord {
  unsigned long id = 0;
  double timestamp = 0.0;
  lightning::SE3 refined_pose;
  std::string source_stage1_dir;
  std::string source_cloud_path;
  bool has_gps = false;
  lightning::CloudPtr cloud;
};

struct Stage4MapStats {
  size_t keyframe_count = 0;
  size_t used_keyframe_count = 0;
  size_t input_points = 0;
  size_t after_keyframe_voxel_points = 0;
  size_t assembled_points = 0;
  size_t after_invalid_points = 0;
  size_t after_height_points = 0;
  size_t after_global_voxel_points = 0;
  size_t output_points = 0;
};

std::filesystem::path LocalModuleRoot(const Stage4Config& config) {
  std::error_code error;
  const std::filesystem::path configured(config.module_root);
  if (std::filesystem::exists(configured / "cyberfile.xml", error)) {
    return configured;
  }
  return FindAirMappingModuleRoot();
}

std::filesystem::path ResolveExistingPath(
    const std::string& path, const std::filesystem::path& module_root) {
  const std::filesystem::path candidate(path);
  std::error_code error;
  if (std::filesystem::exists(candidate, error)) {
    return candidate;
  }
  if (path.rfind(kApolloAirMappingPrefix, 0) == 0) {
    const auto local_candidate =
        module_root / path.substr(std::string(kApolloAirMappingPrefix).size());
    if (std::filesystem::exists(local_candidate, error)) {
      return local_candidate;
    }
  }
  if (!candidate.is_absolute()) {
    const auto local_candidate = module_root / candidate;
    if (std::filesystem::exists(local_candidate, error)) {
      return local_candidate;
    }
  }
  return candidate;
}

std::filesystem::path ResolveOutputPath(
    const std::string& path, const std::filesystem::path& module_root) {
  const std::filesystem::path candidate(path);
  if (path.rfind(kApolloAirMappingPrefix, 0) == 0) {
    return module_root / path.substr(std::string(kApolloAirMappingPrefix).size());
  }
  if (!candidate.is_absolute()) {
    return module_root / candidate;
  }

  std::error_code error;
  const auto parent = candidate.parent_path();
  if (parent.empty() || std::filesystem::exists(parent, error)) {
    return candidate;
  }
  return candidate;
}

std::filesystem::path ResolveArtifactPath(
    const std::filesystem::path& root, const std::string& artifact_path,
    const std::filesystem::path& module_root) {
  const std::filesystem::path path(artifact_path);
  if (path.is_absolute()) {
    return ResolveExistingPath(artifact_path, module_root);
  }
  return root / path;
}

std::vector<std::string> SplitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

bool ParsePoseCsvFields(const std::vector<std::string>& fields, size_t offset,
                        lightning::SE3* pose) {
  if (pose == nullptr || fields.size() < offset + 7) {
    return false;
  }
  const lightning::Vec3d translation(std::stod(fields[offset]),
                                     std::stod(fields[offset + 1]),
                                     std::stod(fields[offset + 2]));
  Eigen::Quaterniond quaternion(
      std::stod(fields[offset + 6]), std::stod(fields[offset + 3]),
      std::stod(fields[offset + 4]), std::stod(fields[offset + 5]));
  if (!translation.allFinite() || quaternion.norm() < 1e-12) {
    return false;
  }
  quaternion.normalize();
  *pose = lightning::SE3(lightning::SO3(quaternion), translation);
  return true;
}

std::vector<std::string> ReadYamlStringList(const YAML::Node& node) {
  std::vector<std::string> values;
  if (!node || !node.IsSequence()) {
    return values;
  }
  for (const auto& item : node) {
    values.push_back(item.as<std::string>());
  }
  return values;
}

bool ReadStage3Manifest(const Stage4Config& config,
                        const std::filesystem::path& input_dir,
                        Stage4InputMetadata* metadata) {
  if (metadata == nullptr) {
    return false;
  }
  const auto manifest_path = input_dir / "manifest.yaml";
  metadata->stage3_manifest_path = manifest_path.string();
  try {
    YAML::Node manifest = YAML::LoadFile(manifest_path.string());
    if (manifest["vehicle_name"]) {
      const auto vehicle_name = manifest["vehicle_name"].as<std::string>();
      if (!config.vehicle_name.empty() && vehicle_name != config.vehicle_name) {
        AERROR << "Stage3 artifact vehicle mismatch: expected "
               << config.vehicle_name << ", manifest has " << vehicle_name;
        return false;
      }
    }
    if (manifest["generated_at"]) {
      metadata->stage3_generated_at = manifest["generated_at"].as<std::string>();
    }
    if (manifest["utm_origin"]) {
      const auto values = manifest["utm_origin"].as<std::vector<double>>();
      if (values.size() == 3) {
        metadata->utm_origin =
            lightning::Vec3d(values[0], values[1], values[2]);
        metadata->has_utm_origin = true;
      }
    }
    if (manifest["dataset"]) {
      const auto& dataset = manifest["dataset"];
      metadata->dataset_sources = ReadYamlStringList(dataset["sources"]);
      metadata->expanded_records =
          ReadYamlStringList(dataset["expanded_records"]);
    }
    if (manifest["provenance"]) {
      const auto& provenance = manifest["provenance"];
      if (provenance["source_stage2_dir"]) {
        metadata->source_stage2_dir =
            provenance["source_stage2_dir"].as<std::string>();
      }
      if (provenance["source_stage2_manifest"]) {
        metadata->source_stage2_manifest =
            provenance["source_stage2_manifest"].as<std::string>();
      }
      if (provenance["source_stage2_generated_at"]) {
        metadata->source_stage2_generated_at =
            provenance["source_stage2_generated_at"].as<std::string>();
      }
      if (provenance["source_stage1_dir"]) {
        metadata->source_stage1_dir =
            provenance["source_stage1_dir"].as<std::string>();
      }
      if (provenance["source_stage1_manifest"]) {
        metadata->source_stage1_manifest =
            provenance["source_stage1_manifest"].as<std::string>();
      }
      if (provenance["source_stage1_generated_at"]) {
        metadata->source_stage1_generated_at =
            provenance["source_stage1_generated_at"].as<std::string>();
      }
      if (provenance["source_config_path"]) {
        metadata->source_config_path =
            provenance["source_config_path"].as<std::string>();
      }
    }
  } catch (const std::exception& e) {
    AERROR << "Failed to read Stage3 manifest: " << manifest_path.string()
           << ", error: " << e.what();
    return false;
  }

  if (!metadata->has_utm_origin || !metadata->utm_origin.allFinite()) {
    AERROR << "Stage3 manifest has no valid utm_origin: "
           << manifest_path.string();
    return false;
  }
  if (metadata->dataset_sources.empty()) {
    metadata->dataset_sources = metadata->expanded_records;
  }
  return true;
}

bool LoadLidarExtrinsic(const std::string& config_path,
                        const std::filesystem::path& module_root,
                        lightning::SO3* rotation,
                        lightning::Vec3d* translation) {
  if (rotation == nullptr || translation == nullptr) {
    return false;
  }
  *rotation = lightning::SO3();
  *translation = lightning::Vec3d::Zero();
  if (config_path.empty()) {
    AWARN << "No source config path, use identity lidar extrinsic";
    return true;
  }

  const auto resolved_config_path =
      ResolveExistingPath(config_path, module_root);
  std::error_code error;
  if (!std::filesystem::exists(resolved_config_path, error)) {
    AWARN << "Source config not found, use identity lidar extrinsic: "
          << config_path;
    return true;
  }

  try {
    YAML::Node yaml = YAML::LoadFile(resolved_config_path.string());
    if (!yaml["fasterlio"]) {
      AWARN << "No fasterlio section in source config, use identity lidar "
               "extrinsic: "
            << resolved_config_path.string();
      return true;
    }
    const auto& fasterlio = yaml["fasterlio"];
    if (fasterlio["extrinsic_R"]) {
      const auto values = fasterlio["extrinsic_R"].as<std::vector<double>>();
      if (values.size() == 9) {
        lightning::Mat3d matrix;
        matrix << values[0], values[1], values[2], values[3], values[4],
            values[5], values[6], values[7], values[8];
        *rotation = lightning::SO3::fitToSO3(matrix);
      }
    }
    if (fasterlio["extrinsic_T"]) {
      const auto values = fasterlio["extrinsic_T"].as<std::vector<double>>();
      if (values.size() == 3) {
        *translation = lightning::Vec3d(values[0], values[1], values[2]);
      }
    }
  } catch (const std::exception& e) {
    AERROR << "Failed to load lidar extrinsic from "
           << resolved_config_path.string() << ", error: " << e.what();
    return false;
  }

  AINFO << "[Stage4] Loaded lidar extrinsic from "
        << resolved_config_path.string() << ", t=" << translation->transpose();
  return true;
}

bool ReadRefinedKeyframes(const std::filesystem::path& input_dir,
                          const std::filesystem::path& module_root,
                          std::vector<Stage4KeyframeRecord>* records) {
  if (records == nullptr) {
    return false;
  }
  records->clear();
  const auto keyframes_path = input_dir / "keyframes" / "keyframes_refined.csv";
  std::ifstream file(keyframes_path);
  if (!file.is_open()) {
    AERROR << "Failed to open Stage3 refined keyframes: "
           << keyframes_path.string();
    return false;
  }

  std::string line;
  std::getline(file, line);
  size_t row = 1;
  while (std::getline(file, line)) {
    ++row;
    if (line.empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() < 27) {
      AERROR << "Invalid keyframes_refined.csv row " << row << ": " << line;
      return false;
    }

    Stage4KeyframeRecord record;
    record.id = std::stoul(fields[0]);
    record.timestamp = std::stod(fields[1]);
    if (!ParsePoseCsvFields(fields, 9, &record.refined_pose)) {
      AERROR << "Invalid refined pose in keyframes_refined.csv row " << row
             << ": " << line;
      return false;
    }
    record.source_stage1_dir = fields[23];
    record.source_cloud_path = fields[24];
    record.has_gps = std::stoi(fields[26]) != 0;
    record.cloud.reset(new lightning::PointCloudType);

    const auto source_stage1_dir =
        ResolveExistingPath(record.source_stage1_dir, module_root);
    const auto cloud_path =
        ResolveArtifactPath(source_stage1_dir, record.source_cloud_path,
                            module_root);
    if (pcl::io::loadPCDFile<lightning::PointType>(cloud_path.string(),
                                                   *record.cloud) < 0) {
      AERROR << "Failed to load keyframe cloud: " << cloud_path.string();
      return false;
    }
    records->push_back(record);
  }

  AINFO << "[Stage4] Loaded refined keyframes: " << records->size();
  return !records->empty();
}

lightning::CloudPtr FilterFinalMap(const Stage4Config& config,
                                   lightning::CloudPtr input,
                                   Stage4MapStats* stats) {
  if (!input) {
    return lightning::CloudPtr(new lightning::PointCloudType);
  }

  lightning::CloudPtr current(new lightning::PointCloudType(*input));
  if (config.final_map.remove_invalid_points) {
    std::vector<int> indices;
    lightning::CloudPtr filtered(new lightning::PointCloudType);
    pcl::removeNaNFromPointCloud(*current, *filtered, indices);
    current = filtered;
  }
  if (stats != nullptr) {
    stats->after_invalid_points = current->size();
  }

  if (config.final_map.enable_height_crop && current && !current->empty()) {
    pcl::PassThrough<lightning::PointType> pass;
    pass.setInputCloud(current);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(config.final_map.min_z_m),
                         static_cast<float>(config.final_map.max_z_m));
    lightning::CloudPtr filtered(new lightning::PointCloudType);
    pass.filter(*filtered);
    if (!filtered->empty()) {
      current = filtered;
    } else {
      AWARN << "[Stage4] Height crop removed all points, keep previous cloud";
    }
  }
  if (stats != nullptr) {
    stats->after_height_points = current->size();
  }

  if (config.final_map.voxel_size_m > 1e-6 && current && !current->empty()) {
    pcl::VoxelGrid<lightning::PointType> voxel;
    const float leaf = static_cast<float>(config.final_map.voxel_size_m);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(current);
    lightning::CloudPtr filtered(new lightning::PointCloudType);
    voxel.filter(*filtered);
    current = filtered;
  }
  if (stats != nullptr) {
    stats->after_global_voxel_points = current->size();
  }

  if (config.final_map.enable_statistical_outlier_removal && current &&
      current->size() >=
          static_cast<size_t>(std::max(5, config.final_map.sor_mean_k))) {
    pcl::StatisticalOutlierRemoval<lightning::PointType> sor;
    sor.setInputCloud(current);
    sor.setMeanK(std::max(5, config.final_map.sor_mean_k));
    sor.setStddevMulThresh(
        std::max(0.1, config.final_map.sor_stddev_mul_thresh));
    lightning::CloudPtr filtered(new lightning::PointCloudType);
    sor.filter(*filtered);
    if (!filtered->empty()) {
      current = filtered;
    } else {
      AWARN << "[Stage4] SOR removed all points, keep previous cloud";
    }
  }

  current->is_dense = false;
  current->height = 1;
  current->width = current->size();
  if (stats != nullptr) {
    stats->output_points = current->size();
  }
  return current;
}

lightning::CloudPtr BuildFinalMap(
    const Stage4Config& config, const std::vector<Stage4KeyframeRecord>& records,
    const lightning::SO3& lidar_rotation,
    const lightning::Vec3d& lidar_translation, Stage4MapStats* stats) {
  lightning::CloudPtr global_map(new lightning::PointCloudType);
  if (stats != nullptr) {
    stats->keyframe_count = records.size();
  }

  pcl::VoxelGrid<lightning::PointType> voxel;
  const float leaf = static_cast<float>(config.final_map.voxel_size_m);
  voxel.setLeafSize(leaf, leaf, leaf);

  const size_t step =
      static_cast<size_t>(std::max(config.final_map.keyframe_step, 1));
  for (size_t index = 0; index < records.size(); index += step) {
    const auto& record = records[index];
    if (!record.cloud || record.cloud->empty()) {
      continue;
    }
    if (stats != nullptr) {
      ++stats->used_keyframe_count;
      stats->input_points += record.cloud->size();
    }

    lightning::CloudPtr cloud_filtered(new lightning::PointCloudType);
    voxel.setInputCloud(record.cloud);
    voxel.filter(*cloud_filtered);
    if (stats != nullptr) {
      stats->after_keyframe_voxel_points += cloud_filtered->size();
    }

    Eigen::Matrix4d t_imu_lidar = Eigen::Matrix4d::Identity();
    t_imu_lidar.block<3, 3>(0, 0) = lidar_rotation.matrix();
    t_imu_lidar.block<3, 1>(0, 3) = lidar_translation;
    const Eigen::Matrix4d t_world_lidar =
        record.refined_pose.matrix() * t_imu_lidar;

    lightning::CloudPtr cloud_transformed(new lightning::PointCloudType);
    pcl::transformPointCloud(*cloud_filtered, *cloud_transformed,
                             t_world_lidar);
    *global_map += *cloud_transformed;
  }

  global_map->is_dense = false;
  global_map->height = 1;
  global_map->width = global_map->size();
  if (stats != nullptr) {
    stats->assembled_points = global_map->size();
  }
  return FilterFinalMap(config, global_map, stats);
}

bool WriteGlobalPcd(const std::filesystem::path& path,
                    const lightning::CloudPtr& cloud) {
  if (!cloud || cloud->empty()) {
    AERROR << "Final map cloud is empty, skip writing: " << path.string();
    return false;
  }
  if (pcl::io::savePCDFileBinaryCompressed(path.string(), *cloud) < 0) {
    AWARN << "Compressed PCD save failed, using binary PCD: " << path.string();
    if (pcl::io::savePCDFileBinary(path.string(), *cloud) < 0) {
      AERROR << "Failed to save global PCD: " << path.string();
      return false;
    }
  }
  return true;
}

bool WriteUtmAlignment(const std::filesystem::path& path,
                       const lightning::Vec3d& utm_origin) {
  std::ofstream file(path);
  if (!file.is_open()) {
    AERROR << "Failed to write UTM alignment: " << path.string();
    return false;
  }
  file << "# UTM Alignment File\n";
  file << "# Stage4 final map is stored in UTM-local coordinates.\n";
  file << "# p_map = p_utm - offset\n";
  file << "offset_x: " << std::fixed << std::setprecision(12)
       << utm_origin.x() << "\n";
  file << "offset_y: " << std::fixed << std::setprecision(12)
       << utm_origin.y() << "\n";
  file << "offset_z: " << std::fixed << std::setprecision(12)
       << utm_origin.z() << "\n";
  file << "yaw_deg: 0.0\n";
  return true;
}

bool WriteManifest(const Stage4Config& config,
                   const Stage4InputMetadata& metadata,
                   const Stage4MapStats& stats,
                   const std::filesystem::path& output_dir) {
  std::ofstream manifest(output_dir / "manifest.yaml");
  if (!manifest.is_open()) {
    AERROR << "Failed to write stage4 manifest";
    return false;
  }
  manifest << "stage: stage4_map_export\n";
  manifest << "generated_at: " << YamlQuote(CurrentIso8601Utc()) << "\n";
  manifest << "map_name: " << YamlQuote(config.map_name) << "\n";
  manifest << "vehicle_name: " << YamlQuote(config.vehicle_name) << "\n";
  manifest << "run_config_path: " << YamlQuote(config.run_config_path) << "\n";
  manifest << "vehicle_config_path: " << YamlQuote(config.vehicle_config_path)
           << "\n";
  manifest << "module_root: " << YamlQuote(config.module_root) << "\n";
  manifest << "data_root: " << YamlQuote(config.data_root) << "\n";
  manifest << "debug_root: " << YamlQuote(config.debug_root) << "\n";
  manifest << "provenance:\n";
  manifest << "  source_stage3_dir: " << YamlQuote(config.input_dir) << "\n";
  manifest << "  source_stage3_manifest: "
           << YamlQuote(metadata.stage3_manifest_path) << "\n";
  manifest << "  source_stage3_generated_at: "
           << YamlQuote(metadata.stage3_generated_at) << "\n";
  manifest << "  source_stage2_dir: "
           << YamlQuote(metadata.source_stage2_dir) << "\n";
  manifest << "  source_stage2_manifest: "
           << YamlQuote(metadata.source_stage2_manifest) << "\n";
  manifest << "  source_stage2_generated_at: "
           << YamlQuote(metadata.source_stage2_generated_at) << "\n";
  manifest << "  source_stage1_dir: "
           << YamlQuote(metadata.source_stage1_dir) << "\n";
  manifest << "  source_stage1_manifest: "
           << YamlQuote(metadata.source_stage1_manifest) << "\n";
  manifest << "  source_stage1_generated_at: "
           << YamlQuote(metadata.source_stage1_generated_at) << "\n";
  manifest << "  source_config_path: "
           << YamlQuote(metadata.source_config_path.empty()
                            ? config.source_config_path
                            : metadata.source_config_path)
           << "\n";
  manifest << "dataset:\n";
  manifest << "  source_count: " << metadata.dataset_sources.size() << "\n";
  manifest << "  sources:\n";
  for (const auto& source : metadata.dataset_sources) {
    manifest << "    - " << YamlQuote(source) << "\n";
  }
  manifest << "  expanded_record_count: " << metadata.expanded_records.size()
           << "\n";
  manifest << "  expanded_records:\n";
  for (const auto& record : metadata.expanded_records) {
    manifest << "    - " << YamlQuote(record) << "\n";
  }
  manifest << "final_map:\n";
  manifest << "  global_pcd: " << YamlQuote("global.pcd") << "\n";
  manifest << "  utm_alignment: " << YamlQuote("utm_alignment.txt") << "\n";
  manifest << "  coordinate_frame: utm_local\n";
  manifest << "  voxel_size_m: " << std::fixed << std::setprecision(9)
           << config.final_map.voxel_size_m << "\n";
  manifest << "  keyframe_step: " << config.final_map.keyframe_step << "\n";
  manifest << "  remove_invalid_points: "
           << (config.final_map.remove_invalid_points ? "true" : "false")
           << "\n";
  manifest << "  enable_height_crop: "
           << (config.final_map.enable_height_crop ? "true" : "false")
           << "\n";
  manifest << "  min_z_m: " << config.final_map.min_z_m << "\n";
  manifest << "  max_z_m: " << config.final_map.max_z_m << "\n";
  manifest << "  enable_statistical_outlier_removal: "
           << (config.final_map.enable_statistical_outlier_removal ? "true"
                                                                    : "false")
           << "\n";
  manifest << "  sor_mean_k: " << config.final_map.sor_mean_k << "\n";
  manifest << "  sor_stddev_mul_thresh: "
           << config.final_map.sor_stddev_mul_thresh << "\n";
  manifest << "utm_origin: [" << std::fixed << std::setprecision(9)
           << metadata.utm_origin.x() << ", " << metadata.utm_origin.y()
           << ", " << metadata.utm_origin.z() << "]\n";
  manifest << "stats:\n";
  manifest << "  keyframe_count: " << stats.keyframe_count << "\n";
  manifest << "  used_keyframe_count: " << stats.used_keyframe_count << "\n";
  manifest << "  input_points: " << stats.input_points << "\n";
  manifest << "  after_keyframe_voxel_points: "
           << stats.after_keyframe_voxel_points << "\n";
  manifest << "  assembled_points: " << stats.assembled_points << "\n";
  manifest << "  after_invalid_points: " << stats.after_invalid_points << "\n";
  manifest << "  after_height_points: " << stats.after_height_points << "\n";
  manifest << "  after_global_voxel_points: "
           << stats.after_global_voxel_points << "\n";
  manifest << "  output_points: " << stats.output_points << "\n";
  manifest << "output_format_version: 1\n";
  return true;
}

}  // namespace

bool Stage4Runner::Run(const Stage4Config& config) const {
  const auto module_root = LocalModuleRoot(config);
  const auto input_dir = ResolveExistingPath(config.input_dir, module_root);
  const auto output_dir = ResolveOutputPath(config.output_dir, module_root);

  Stage4InputMetadata metadata;
  if (!ReadStage3Manifest(config, input_dir, &metadata)) {
    return false;
  }
  if (metadata.source_config_path.empty()) {
    metadata.source_config_path = config.source_config_path;
  }

  lightning::SO3 lidar_rotation;
  lightning::Vec3d lidar_translation;
  if (!LoadLidarExtrinsic(metadata.source_config_path, module_root,
                          &lidar_rotation, &lidar_translation)) {
    return false;
  }

  std::vector<Stage4KeyframeRecord> records;
  if (!ReadRefinedKeyframes(input_dir, module_root, &records)) {
    return false;
  }

  if (!PrepareCleanOutputDirectory(output_dir, "stage4")) {
    return false;
  }

  Stage4MapStats stats;
  const auto final_map =
      BuildFinalMap(config, records, lidar_rotation, lidar_translation, &stats);
  if (!WriteGlobalPcd(output_dir / "global.pcd", final_map)) {
    return false;
  }
  if (!WriteUtmAlignment(output_dir / "utm_alignment.txt",
                         metadata.utm_origin)) {
    return false;
  }
  if (!WriteManifest(config, metadata, stats, output_dir)) {
    return false;
  }

  AINFO << "[Stage4] Final map written to " << output_dir.string()
        << ", points=" << stats.output_points
        << ", voxel=" << config.final_map.voxel_size_m << "m";
  return true;
}

}  // namespace stage4
}  // namespace air_mapping
}  // namespace apollo
