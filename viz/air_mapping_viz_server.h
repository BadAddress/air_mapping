#pragma once

#include <atomic>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include "nlohmann/json.hpp"

namespace apollo {
namespace air_mapping {
namespace viz {

class AirMappingVizServer {
 public:
  using Json = nlohmann::json;

  struct Options {
    int port = 12322;
    std::string module_root = "/apollo_workspace/modules/air_mapping";
    std::string doc_root =
        "/apollo_workspace/modules/air_mapping/viz/frontend";
    std::string hdmap_root = "viz/hdmap_local";
    std::string active_vehicle = "es6";
    std::string data_root = "/apollo_workspace/modules/air_mapping/data";
    std::string debug_root =
        "/apollo_workspace/modules/air_mapping/data/debug";
    std::string vehicle_data_root;
    std::string default_local_pcd;
    std::string default_global_pcd;
    std::string default_hdmap;
    std::string default_utm_alignment;
    bool downsample_enabled = true;
    float voxel_size = 0.2f;
    int max_points = 3000000;
  };

  AirMappingVizServer();
  explicit AirMappingVizServer(const Options& options);
  ~AirMappingVizServer();

  bool Init();
  void Run();
  void Stop();
  bool LoadConfig(const std::string& config_path);

 private:
  struct HttpRequest {
    std::string method;
    std::string path;
    std::string query;
  };

  void AcceptThread();
  void HandleClient(int client_fd);
  bool HandleHttpRequest(int client_fd, const std::string& request);

  bool ParseRequestLine(const std::string& request, HttpRequest* parsed) const;
  void WriteResponse(int client_fd, const std::string& status,
                     const std::string& content_type,
                     const std::string& body) const;
  bool WriteAll(int client_fd, const char* data, size_t size) const;
  void WriteBinaryResponse(int client_fd, const std::string& content_type,
                           const std::string& body) const;
  void WriteJsonResponse(int client_fd, const Json& json) const;

  Json HandleDefaults() const;
  Json HandleListFiles(const std::string& query) const;
  std::string HandlePcdRequest(const std::string& query) const;
  Json HandleHdMapRequest(const std::string& query) const;
  std::string ResolveDefaultHdmapRoot() const;

  std::filesystem::path ResolveModulePath(const std::string& relative_path,
                                          bool must_exist) const;
  std::filesystem::path ResolveDocPath(const std::string& url_path) const;
  bool IsUnderModuleRoot(const std::filesystem::path& path) const;

  std::vector<float> LoadAndDownsamplePcd(
      const std::filesystem::path& path) const;
  Json LoadHdMapJson(const std::filesystem::path& hdmap_path,
                     const Eigen::Vector3d& utm_offset) const;
  bool LoadUtmOffset(const std::filesystem::path& alignment_path,
                     Eigen::Vector3d* offset) const;

  std::string ReadFile(const std::filesystem::path& path) const;
  std::string GetMimeType(const std::filesystem::path& path) const;

  Options options_;
  std::filesystem::path module_root_;
  std::filesystem::path doc_root_;
  int server_fd_ = -1;
  std::thread accept_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace viz
}  // namespace air_mapping
}  // namespace apollo
