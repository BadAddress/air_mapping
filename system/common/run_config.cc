#include "modules/air_mapping/system/common/run_config.h"

#include <exception>
#include <system_error>

#include "cyber/common/log.h"

namespace apollo {
namespace air_mapping {

namespace {

std::string ReadString(const YAML::Node& node, const std::string& key,
                       const std::string& default_value) {
  return node && node[key] ? node[key].as<std::string>() : default_value;
}

std::string DefaultVehicleConfigPath(const std::string& vehicle) {
  return "conf/vehicles/" + vehicle + ".yaml";
}

std::string DefaultDataRoot(const std::string& module_root) {
  return JoinAirMappingPath(module_root, "data");
}

std::string DefaultDebugRoot(const std::string& module_root) {
  return JoinAirMappingPath(module_root, "data/debug");
}

}  // namespace

std::filesystem::path FindAirMappingModuleRoot() {
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
  return kDefaultModuleRoot;
}

std::filesystem::path ResolveAirMappingPath(const std::string& path,
                                            const std::string& module_root) {
  if (path.empty()) {
    return module_root;
  }
  const std::filesystem::path candidate(path);
  if (candidate.is_absolute()) {
    return candidate;
  }
  return std::filesystem::path(module_root) / candidate;
}

std::filesystem::path ResolveAirMappingPath(const std::string& path) {
  return ResolveAirMappingPath(path, kDefaultModuleRoot);
}

std::string JoinAirMappingPath(const std::string& module_root,
                               const std::string& relative_path) {
  return (std::filesystem::path(module_root) / relative_path).string();
}

bool IsTopLevelRunConfig(const YAML::Node& yaml) {
  return yaml && yaml["air_mapping"];
}

bool LoadAirMappingRunConfig(const std::string& config_path,
                             AirMappingRunConfig* config) {
  if (config == nullptr) {
    return false;
  }

  try {
    YAML::Node yaml = YAML::LoadFile(config_path);
    if (!IsTopLevelRunConfig(yaml)) {
      AERROR << "Not an air_mapping top-level config: " << config_path;
      return false;
    }

    const auto& root = yaml["air_mapping"];
    config->module_root =
        ReadString(root, "module_root", std::string(kDefaultModuleRoot));
    config->active_vehicle =
        ReadString(root, "active_vehicle", config->active_vehicle);
    config->active_vehicle_config =
        ReadString(root, "active_vehicle_config",
                   DefaultVehicleConfigPath(config->active_vehicle));
    config->data_root =
        ReadString(root, "data_root", DefaultDataRoot(config->module_root));
    config->debug_root =
        ReadString(root, "debug_root", DefaultDebugRoot(config->module_root));

    const auto profile_path =
        ResolveAirMappingPath(config->active_vehicle_config,
                              config->module_root);
    config->resolved_vehicle_config_path = profile_path.string();
    config->vehicle_yaml = YAML::LoadFile(config->resolved_vehicle_config_path);
    if (!config->vehicle_yaml) {
      AERROR << "Empty vehicle config: "
             << config->resolved_vehicle_config_path;
      return false;
    }

    AINFO << "[air_mapping_config] active_vehicle="
          << config->active_vehicle
          << ", vehicle_config=" << config->resolved_vehicle_config_path
          << ", data_root=" << config->data_root
          << ", debug_root=" << config->debug_root;
  } catch (const std::exception& e) {
    AERROR << "Failed to load air_mapping run config: " << config_path
           << ", error: " << e.what();
    return false;
  }
  return true;
}

}  // namespace air_mapping
}  // namespace apollo
