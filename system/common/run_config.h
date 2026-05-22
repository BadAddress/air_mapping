#pragma once

#include <filesystem>
#include <string>

#include "yaml-cpp/yaml.h"

namespace apollo {
namespace air_mapping {

constexpr const char* kDefaultModuleRoot =
    "/apollo_workspace/modules/air_mapping";
constexpr const char* kDefaultRunConfigPath =
    "/apollo_workspace/modules/air_mapping/conf/current_vehicle.yaml";

struct AirMappingRunConfig {
  std::string module_root = kDefaultModuleRoot;
  std::string active_vehicle = "es6";
  std::string active_vehicle_config = "conf/vehicles/es6.yaml";
  std::string data_root;
  std::string debug_root;
  std::string resolved_vehicle_config_path;
  YAML::Node vehicle_yaml;
};

std::filesystem::path FindAirMappingModuleRoot();
std::filesystem::path ResolveAirMappingPath(const std::string& path,
                                            const std::string& module_root);
std::filesystem::path ResolveAirMappingPath(const std::string& path);
std::string JoinAirMappingPath(const std::string& module_root,
                               const std::string& relative_path);
bool IsTopLevelRunConfig(const YAML::Node& yaml);
bool LoadAirMappingRunConfig(const std::string& config_path,
                             AirMappingRunConfig* config);

}  // namespace air_mapping
}  // namespace apollo
