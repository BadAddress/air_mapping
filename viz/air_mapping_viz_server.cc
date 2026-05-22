#include "modules/air_mapping/viz/air_mapping_viz_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <system_error>
#include <unordered_map>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "cyber/state.h"
#include "modules/air_mapping/system/common/run_config.h"

namespace apollo {
namespace air_mapping {
namespace viz {

namespace {

std::string Trim(const std::string& text) {
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::string UrlDecode(const std::string& encoded) {
  std::string decoded;
  decoded.reserve(encoded.size());
  for (size_t i = 0; i < encoded.size(); ++i) {
    if (encoded[i] == '%' && i + 2 < encoded.size()) {
      const std::string hex = encoded.substr(i + 1, 2);
      char* end = nullptr;
      const long value = std::strtol(hex.c_str(), &end, 16);
      if (end != nullptr && *end == '\0') {
        decoded.push_back(static_cast<char>(value));
        i += 2;
        continue;
      }
    }
    decoded.push_back(encoded[i] == '+' ? ' ' : encoded[i]);
  }
  return decoded;
}

std::unordered_map<std::string, std::string> ParseQuery(
    const std::string& query) {
  std::unordered_map<std::string, std::string> values;
  size_t cursor = 0;
  while (cursor <= query.size()) {
    const size_t next = query.find('&', cursor);
    const std::string item =
        query.substr(cursor, next == std::string::npos ? std::string::npos
                                                       : next - cursor);
    if (!item.empty()) {
      const size_t equal = item.find('=');
      const std::string key = UrlDecode(item.substr(0, equal));
      const std::string value =
          equal == std::string::npos ? "" : UrlDecode(item.substr(equal + 1));
      values[key] = value;
    }
    if (next == std::string::npos) {
      break;
    }
    cursor = next + 1;
  }
  return values;
}

bool HasExtension(const std::filesystem::path& path,
                  const std::string& extension) {
  std::string actual = path.extension().string();
  std::transform(actual.begin(), actual.end(), actual.begin(), ::tolower);
  return actual == extension;
}

bool FindDoubleAfterKey(const std::string& block, const std::string& key,
                        double* value) {
  if (value == nullptr) {
    return false;
  }
  const size_t pos = block.find(key);
  if (pos == std::string::npos) {
    return false;
  }
  const size_t colon = block.find(':', pos + key.size());
  if (colon == std::string::npos) {
    return false;
  }
  const char* begin = block.c_str() + colon + 1;
  char* end = nullptr;
  const double parsed = std::strtod(begin, &end);
  if (end == begin || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

std::string ExtractBlock(const std::string& content,
                         const std::string& block_name,
                         size_t start_pos = 0) {
  const std::string search = block_name + " {";
  const size_t pos = content.find(search, start_pos);
  if (pos == std::string::npos) {
    return "";
  }

  size_t brace_count = 1;
  size_t end = content.find('{', pos) + 1;
  while (brace_count > 0 && end < content.size()) {
    if (content[end] == '{') {
      ++brace_count;
    } else if (content[end] == '}') {
      --brace_count;
    }
    ++end;
  }
  return content.substr(pos, end - pos);
}

nlohmann::json ParseLineSegmentPoints(const std::string& segment_block,
                                      const Eigen::Vector3d& utm_offset) {
  nlohmann::json points = nlohmann::json::array();
  const size_t line_start = segment_block.find("line_segment {");
  if (line_start == std::string::npos) {
    return points;
  }

  size_t brace_count = 1;
  size_t line_end = segment_block.find('{', line_start) + 1;
  while (brace_count > 0 && line_end < segment_block.size()) {
    if (segment_block[line_end] == '{') {
      ++brace_count;
    } else if (segment_block[line_end] == '}') {
      --brace_count;
    }
    ++line_end;
  }

  const std::string line_block =
      segment_block.substr(line_start, line_end - line_start);
  size_t point_pos = 0;
  while ((point_pos = line_block.find("point {", point_pos)) !=
         std::string::npos) {
    const size_t point_end = line_block.find('}', point_pos);
    if (point_end == std::string::npos) {
      break;
    }
    const std::string point_block =
        line_block.substr(point_pos, point_end - point_pos);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    FindDoubleAfterKey(point_block, "x", &x);
    FindDoubleAfterKey(point_block, "y", &y);
    FindDoubleAfterKey(point_block, "z", &z);

    nlohmann::json point;
    point["x"] = x - utm_offset.x();
    point["y"] = y - utm_offset.y();
    point["z"] = z - utm_offset.z();
    points.push_back(point);
    point_pos = point_end + 1;
  }
  return points;
}

std::vector<nlohmann::json> ParseCurveSegments(
    const std::string& curve_block, const Eigen::Vector3d& utm_offset) {
  std::vector<nlohmann::json> segments;
  size_t pos = 0;
  while ((pos = curve_block.find("segment {", pos)) != std::string::npos) {
    size_t brace_count = 1;
    size_t end = curve_block.find('{', pos) + 1;
    while (brace_count > 0 && end < curve_block.size()) {
      if (curve_block[end] == '{') {
        ++brace_count;
      } else if (curve_block[end] == '}') {
        --brace_count;
      }
      ++end;
    }
    const std::string segment_block = curve_block.substr(pos, end - pos);
    nlohmann::json points =
        ParseLineSegmentPoints(segment_block, utm_offset);
    if (points.size() > 1) {
      segments.push_back(points);
    }
    pos = end;
  }
  return segments;
}

std::string ExtractId(const std::string& block) {
  const size_t id_pos = block.find("id: \"");
  if (id_pos == std::string::npos) {
    return "";
  }
  const size_t id_start = id_pos + 5;
  const size_t id_end = block.find('"', id_start);
  if (id_end == std::string::npos) {
    return "";
  }
  return block.substr(id_start, id_end - id_start);
}

std::string ExtractBoundaryType(const std::string& boundary_block) {
  const size_t type_pos = boundary_block.find("types:");
  if (type_pos == std::string::npos) {
    return "UNKNOWN";
  }
  size_t start = type_pos + 6;
  while (start < boundary_block.size() &&
         (boundary_block[start] == ' ' || boundary_block[start] == '\n')) {
    ++start;
  }
  size_t end = start;
  while (end < boundary_block.size() && boundary_block[end] != '\n' &&
         boundary_block[end] != '}') {
    ++end;
  }
  return Trim(boundary_block.substr(start, end - start));
}

bool IsGitLfsPointerContent(const std::string& content) {
  static constexpr const char* kLfsPrefix =
      "version https://git-lfs.github.com/spec/v1";
  return content.rfind(kLfsPrefix, 0) == 0;
}

bool IsGitLfsPointerFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input.is_open()) {
    return false;
  }
  std::string header;
  std::getline(input, header);
  return IsGitLfsPointerContent(header);
}

std::filesystem::path FindDefaultModuleRoot() {
  std::error_code error;
  std::filesystem::path path = std::filesystem::current_path(error);
  while (!error && !path.empty()) {
    if (std::filesystem::exists(path / "cyberfile.xml", error) &&
        std::filesystem::exists(path / "stage1", error) &&
        std::filesystem::exists(path / "stage2", error)) {
      return std::filesystem::weakly_canonical(path, error);
    }
    const auto candidate = path / "modules" / "air_mapping";
    if (std::filesystem::exists(candidate / "cyberfile.xml", error)) {
      return std::filesystem::weakly_canonical(candidate, error);
    }
    if (path == path.root_path()) {
      break;
    }
    path = path.parent_path();
  }
  return "/apollo_workspace/modules/air_mapping";
}

std::string VehicleDataRoot(const std::string& active_vehicle) {
  return (std::filesystem::path("data") / active_vehicle).string();
}

std::string StagePreviewPcd(const std::string& vehicle_data_root,
                            const std::string& stage,
                            const std::string& filename) {
  return (std::filesystem::path(vehicle_data_root) / stage / "preview" /
          filename)
      .string();
}

std::string StageAlignmentFile(const std::string& vehicle_data_root,
                               const std::string& stage,
                               const std::string& filename) {
  return (std::filesystem::path(vehicle_data_root) / stage / "alignment" /
          filename)
      .string();
}

}  // namespace

AirMappingVizServer::AirMappingVizServer() = default;

AirMappingVizServer::AirMappingVizServer(const Options& options)
    : options_(options) {}

AirMappingVizServer::~AirMappingVizServer() { Stop(); }

bool AirMappingVizServer::Init() {
  std::error_code error;
  module_root_ = std::filesystem::weakly_canonical(options_.module_root, error);
  if (error || !std::filesystem::exists(module_root_)) {
    module_root_ = FindDefaultModuleRoot();
    options_.module_root = module_root_.string();
  }
  doc_root_ = std::filesystem::weakly_canonical(options_.doc_root, error);
  if (error || !std::filesystem::exists(doc_root_)) {
    doc_root_ = module_root_ / "viz" / "frontend";
    options_.doc_root = doc_root_.string();
  }

  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ == -1) {
    AERROR << "Failed to create socket";
    return false;
  }

  int opt = 1;
  setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt,
             sizeof(opt));

  sockaddr_in address;
  std::memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(options_.port);
  if (bind(server_fd_, reinterpret_cast<sockaddr*>(&address),
           sizeof(address)) < 0) {
    AERROR << "Bind failed on port " << options_.port
           << ": " << std::strerror(errno);
    return false;
  }
  if (listen(server_fd_, 16) < 0) {
    AERROR << "Listen failed";
    return false;
  }

  running_ = true;
  accept_thread_ = std::thread(&AirMappingVizServer::AcceptThread, this);
  AINFO << "[air_mapping_viz] Server started on port " << options_.port
        << ", module_root=" << module_root_.string()
        << ", doc_root=" << doc_root_.string();
  return true;
}

void AirMappingVizServer::Run() {
  while (running_ && !apollo::cyber::IsShutdown()) {
    sleep(1);
  }
  Stop();
}

void AirMappingVizServer::Stop() {
  running_ = false;
  if (server_fd_ != -1) {
    shutdown(server_fd_, SHUT_RDWR);
    close(server_fd_);
    server_fd_ = -1;
  }
  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }
}

bool AirMappingVizServer::LoadConfig(const std::string& config_path) {
  try {
    YAML::Node yaml = YAML::LoadFile(config_path);
    if (IsTopLevelRunConfig(yaml)) {
      AirMappingRunConfig run_config;
      if (!LoadAirMappingRunConfig(config_path, &run_config)) {
        return false;
      }
      options_.module_root = run_config.module_root;
      options_.active_vehicle = run_config.active_vehicle;
      options_.data_root = run_config.data_root;
      options_.debug_root = run_config.debug_root;
      options_.vehicle_data_root = VehicleDataRoot(options_.active_vehicle);
      options_.default_local_pcd =
          StagePreviewPcd(options_.vehicle_data_root, "stage3_graph_refine",
                          "refined_global_preview.pcd");
      options_.default_global_pcd =
          StagePreviewPcd(options_.vehicle_data_root, "stage2_graph_opt",
                          "optimized_global_preview.pcd");
      options_.default_hdmap = ResolveDefaultHdmapRoot();
      options_.default_utm_alignment =
          StageAlignmentFile(options_.vehicle_data_root, "stage2_graph_opt",
                             "utm_origin.txt");
    }
    const Options default_options;
    if (yaml["server"]) {
      const auto& server = yaml["server"];
      if (server["port"] && options_.port == default_options.port) {
        options_.port = server["port"].as<int>();
      }
      if (server["module_root"] &&
          options_.module_root == default_options.module_root) {
        options_.module_root = server["module_root"].as<std::string>();
      }
      if (server["doc_root"] && options_.doc_root == default_options.doc_root) {
        options_.doc_root = server["doc_root"].as<std::string>();
      }
      if (server["hdmap_root"] &&
          options_.hdmap_root == default_options.hdmap_root) {
        options_.hdmap_root = server["hdmap_root"].as<std::string>();
      }
    }
    if (yaml["pcd"]) {
      const auto& pcd = yaml["pcd"];
      if (pcd["downsample_enabled"]) {
        options_.downsample_enabled = pcd["downsample_enabled"].as<bool>();
      }
      if (pcd["voxel_size"]) {
        options_.voxel_size = pcd["voxel_size"].as<float>();
      }
      if (pcd["max_points"]) {
        options_.max_points = pcd["max_points"].as<int>();
      }
    }
  } catch (const std::exception& e) {
    AERROR << "Failed to load viz config: " << config_path
           << ", error=" << e.what();
    return false;
  }
  return true;
}

void AirMappingVizServer::AcceptThread() {
  while (running_) {
    sockaddr_in client_addr;
    socklen_t addrlen = sizeof(client_addr);
    const int client_fd =
        accept(server_fd_, reinterpret_cast<sockaddr*>(&client_addr),
               &addrlen);
    if (client_fd < 0) {
      continue;
    }
    std::thread(&AirMappingVizServer::HandleClient, this, client_fd).detach();
  }
}

void AirMappingVizServer::HandleClient(int client_fd) {
  char buffer[8192];
  const int bytes = read(client_fd, buffer, sizeof(buffer));
  if (bytes <= 0) {
    close(client_fd);
    return;
  }
  const std::string request(buffer, bytes);
  HandleHttpRequest(client_fd, request);
  close(client_fd);
}

bool AirMappingVizServer::HandleHttpRequest(int client_fd,
                                            const std::string& request) {
  HttpRequest parsed;
  if (!ParseRequestLine(request, &parsed)) {
    WriteResponse(client_fd, "400 Bad Request", "text/plain", "Bad Request");
    return false;
  }
  if (parsed.method != "GET") {
    WriteResponse(client_fd, "405 Method Not Allowed", "text/plain",
                  "Only GET is supported");
    return false;
  }

  try {
    if (parsed.path == "/api/defaults") {
      WriteJsonResponse(client_fd, HandleDefaults());
      return true;
    }
    if (parsed.path == "/api/files") {
      WriteJsonResponse(client_fd, HandleListFiles(parsed.query));
      return true;
    }
    if (parsed.path == "/api/pcd") {
      WriteBinaryResponse(client_fd, "application/octet-stream",
                          HandlePcdRequest(parsed.query));
      return true;
    }
    if (parsed.path == "/api/hdmap") {
      WriteJsonResponse(client_fd, HandleHdMapRequest(parsed.query));
      return true;
    }

    const auto doc_path = ResolveDocPath(parsed.path);
    if (doc_path.empty() || !std::filesystem::exists(doc_path)) {
      WriteResponse(client_fd, "404 Not Found", "text/plain", "Not Found");
      return false;
    }
    WriteResponse(client_fd, "200 OK", GetMimeType(doc_path),
                  ReadFile(doc_path));
    return true;
  } catch (const std::exception& e) {
    Json error;
    error["ok"] = false;
    error["error"] = e.what();
    WriteJsonResponse(client_fd, error);
    return false;
  }
}

bool AirMappingVizServer::ParseRequestLine(const std::string& request,
                                           HttpRequest* parsed) const {
  if (parsed == nullptr) {
    return false;
  }
  std::istringstream stream(request);
  std::string url;
  std::string version;
  stream >> parsed->method >> url >> version;
  if (parsed->method.empty() || url.empty()) {
    return false;
  }
  const size_t query_pos = url.find('?');
  parsed->path =
      query_pos == std::string::npos ? url : url.substr(0, query_pos);
  parsed->query =
      query_pos == std::string::npos ? "" : url.substr(query_pos + 1);
  return true;
}

void AirMappingVizServer::WriteResponse(int client_fd,
                                        const std::string& status,
                                        const std::string& content_type,
                                        const std::string& body) const {
  std::ostringstream response;
  response << "HTTP/1.1 " << status << "\r\n"
           << "Content-Type: " << content_type << "\r\n"
           << "Content-Length: " << body.size() << "\r\n"
           << "Cache-Control: no-cache\r\n"
           << "Connection: close\r\n\r\n";
  const std::string header = response.str();
  WriteAll(client_fd, header.data(), header.size());
  WriteAll(client_fd, body.data(), body.size());
}

bool AirMappingVizServer::WriteAll(int client_fd, const char* data,
                                   size_t size) const {
  size_t written = 0;
  while (written < size) {
    const ssize_t n = write(client_fd, data + written, size - written);
    if (n <= 0) {
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

void AirMappingVizServer::WriteBinaryResponse(
    int client_fd, const std::string& content_type,
    const std::string& body) const {
  WriteResponse(client_fd, "200 OK", content_type, body);
}

void AirMappingVizServer::WriteJsonResponse(int client_fd,
                                            const Json& json) const {
  WriteResponse(client_fd, "200 OK", "application/json; charset=utf-8",
                json.dump());
}

AirMappingVizServer::Json AirMappingVizServer::HandleDefaults() const {
  Json defaults;
  const std::string vehicle_data_root =
      options_.vehicle_data_root.empty()
          ? VehicleDataRoot(options_.active_vehicle)
          : options_.vehicle_data_root;
  defaults["ok"] = true;
  defaults["module_root"] = module_root_.string();
  defaults["active_vehicle"] = options_.active_vehicle;
  defaults["pcd_root"] = vehicle_data_root;
  defaults["hdmap_root"] = ResolveDefaultHdmapRoot();
  defaults["alignment_root"] = vehicle_data_root;
  defaults["local_pcd"] = options_.default_local_pcd.empty()
                               ? StagePreviewPcd(vehicle_data_root,
                                                 "stage3_graph_refine",
                                                 "refined_global_preview.pcd")
                               : options_.default_local_pcd;
  defaults["global_pcd"] = options_.default_global_pcd.empty()
                                ? StagePreviewPcd(vehicle_data_root,
                                                  "stage2_graph_opt",
                                                  "optimized_global_preview.pcd")
                                : options_.default_global_pcd;
  defaults["hdmap"] = options_.default_hdmap.empty()
                          ? "viz/hdmap_local"
                          : options_.default_hdmap;
  defaults["utm_alignment"] = options_.default_utm_alignment.empty()
                                  ? StageAlignmentFile(vehicle_data_root,
                                                       "stage2_graph_opt",
                                                       "utm_origin.txt")
                                  : options_.default_utm_alignment;
  return defaults;
}

AirMappingVizServer::Json AirMappingVizServer::HandleListFiles(
    const std::string& query) const {
  const auto params = ParseQuery(query);
  const std::string type = params.count("type") ? params.at("type") : "";
  const std::string root = params.count("root") ? params.at("root") : "";
  const std::string effective_root =
      !root.empty() ? root
                    : (type == "hdmap"
                           ? ResolveDefaultHdmapRoot()
                           : (options_.vehicle_data_root.empty()
                                  ? VehicleDataRoot(options_.active_vehicle)
                                  : options_.vehicle_data_root));
  const auto root_path = ResolveModulePath(effective_root, false);
  Json response;
  response["ok"] = true;
  response["root"] = std::filesystem::relative(root_path, module_root_).string();
  response["files"] = Json::array();

  if (!std::filesystem::exists(root_path) ||
      !std::filesystem::is_directory(root_path)) {
    return response;
  }

  std::vector<std::filesystem::path> files;
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(root_path)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto& path = entry.path();
    bool accept = false;
    if (type == "pcd") {
      accept = HasExtension(path, ".pcd") &&
               path.parent_path().filename() == "preview";
    } else if (type == "hdmap") {
      accept = path.filename() == "base_map.txt" ||
               path.filename() == "sim_map.txt" ||
               path.filename() == "routing_map.txt";
      if (accept && IsGitLfsPointerFile(path)) {
        accept = false;
      }
    } else if (type == "alignment") {
      const auto name = path.filename().string();
      accept = name == "utm_alignment.txt" || name == "gnss-map-offset.txt" ||
               name == "utm_alignment_lio_gps_aligned.txt" ||
               name == "utm_origin.txt";
    } else {
      accept = true;
    }
    if (accept) {
      files.push_back(path);
    }
  }
  std::sort(files.begin(), files.end());
  for (const auto& path : files) {
    Json file;
    file["path"] = std::filesystem::relative(path, module_root_).string();
    file["size"] = static_cast<uint64_t>(std::filesystem::file_size(path));
    response["files"].push_back(file);
  }
  return response;
}

std::string AirMappingVizServer::HandlePcdRequest(
    const std::string& query) const {
  const auto params = ParseQuery(query);
  if (!params.count("path")) {
    throw std::runtime_error("missing path");
  }
  const auto pcd_path = ResolveModulePath(params.at("path"), true);
  if (!HasExtension(pcd_path, ".pcd")) {
    throw std::runtime_error("path is not a .pcd file");
  }
  const auto points = LoadAndDownsamplePcd(pcd_path);
  std::string body;
  body.resize(4 + points.size() * sizeof(float));
  std::memcpy(&body[0], "PCL1", 4);
  if (!points.empty()) {
    std::memcpy(&body[4], points.data(), points.size() * sizeof(float));
  }
  return body;
}

AirMappingVizServer::Json AirMappingVizServer::HandleHdMapRequest(
    const std::string& query) const {
  const auto params = ParseQuery(query);
  if (!params.count("path")) {
    throw std::runtime_error("missing hdmap path");
  }
  if (!params.count("alignment")) {
    throw std::runtime_error("missing utm alignment path");
  }
  const auto hdmap_path = ResolveModulePath(params.at("path"), true);
  const auto alignment_path = ResolveModulePath(params.at("alignment"), true);
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  LoadUtmOffset(alignment_path, &offset);
  Json hdmap = LoadHdMapJson(hdmap_path, offset);
  hdmap["utm_offset"]["x"] = offset.x();
  hdmap["utm_offset"]["y"] = offset.y();
  hdmap["utm_offset"]["z"] = offset.z();
  return hdmap;
}

std::filesystem::path AirMappingVizServer::ResolveModulePath(
    const std::string& relative_path, bool must_exist) const {
  if (relative_path.empty()) {
    return module_root_;
  }
  std::filesystem::path candidate(relative_path);
  if (candidate.is_absolute()) {
    std::error_code error;
    const auto canonical =
        must_exist ? std::filesystem::canonical(candidate, error)
                   : std::filesystem::weakly_canonical(candidate, error);
    if (error || !IsUnderModuleRoot(canonical)) {
      throw std::runtime_error("absolute path must be under air_mapping");
    }
    return canonical;
  } else {
    candidate = module_root_ / candidate;
  }

  std::error_code error;
  const auto canonical =
      must_exist ? std::filesystem::canonical(candidate, error)
                 : std::filesystem::weakly_canonical(candidate, error);
  if (error) {
    throw std::runtime_error("failed to resolve path: " + relative_path);
  }
  if (!IsUnderModuleRoot(canonical)) {
    throw std::runtime_error("path escapes air_mapping module root");
  }
  return canonical;
}

std::string AirMappingVizServer::ResolveDefaultHdmapRoot() const {
  if (!options_.hdmap_root.empty()) {
    return options_.hdmap_root;
  }
  return "viz/hdmap_local";
}

std::filesystem::path AirMappingVizServer::ResolveDocPath(
    const std::string& url_path) const {
  std::string path = url_path == "/" ? "/index.html" : url_path;
  if (path.find("..") != std::string::npos) {
    return {};
  }
  if (!path.empty() && path[0] == '/') {
    path.erase(path.begin());
  }
  std::error_code error;
  const auto resolved = std::filesystem::weakly_canonical(doc_root_ / path, error);
  if (error) {
    return {};
  }
  const auto relative = std::filesystem::relative(resolved, doc_root_, error);
  if (error || relative.empty() || relative.string().rfind("..", 0) == 0) {
    return {};
  }
  return resolved;
}

bool AirMappingVizServer::IsUnderModuleRoot(
    const std::filesystem::path& path) const {
  std::error_code error;
  const auto relative = std::filesystem::relative(path, module_root_, error);
  return !error && !relative.empty() && relative.string().rfind("..", 0) != 0;
}

std::vector<float> AirMappingVizServer::LoadAndDownsamplePcd(
    const std::filesystem::path& path) const {
  std::vector<float> result;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(path.string(), *cloud) < 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.string(), *cloud_xyz) < 0) {
      throw std::runtime_error("failed to load PCD: " + path.string());
    }
    cloud->resize(cloud_xyz->size());
    for (size_t i = 0; i < cloud_xyz->size(); ++i) {
      cloud->points[i].x = cloud_xyz->points[i].x;
      cloud->points[i].y = cloud_xyz->points[i].y;
      cloud->points[i].z = cloud_xyz->points[i].z;
      cloud->points[i].intensity = 0.0f;
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered = cloud;
  if (options_.downsample_enabled && options_.voxel_size > 0.0f &&
      !cloud->empty()) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(options_.voxel_size, options_.voxel_size,
                      options_.voxel_size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(
        new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter(*tmp);
    filtered = tmp;
  }

  size_t stride = 1;
  if (options_.max_points > 0 &&
      filtered->size() > static_cast<size_t>(options_.max_points)) {
    stride = static_cast<size_t>(
        std::ceil(static_cast<double>(filtered->size()) /
                  static_cast<double>(options_.max_points)));
  }

  result.reserve(((filtered->size() + stride - 1) / stride) * 4);
  for (size_t i = 0; i < filtered->size(); i += stride) {
    const auto& point = filtered->points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    result.push_back(point.x);
    result.push_back(point.y);
    result.push_back(point.z);
    result.push_back(point.intensity);
  }
  AINFO << "[air_mapping_viz] Loaded PCD " << path.string()
        << " raw_points=" << cloud->size()
        << " output_points=" << result.size() / 4
        << " voxel=" << (options_.downsample_enabled ? options_.voxel_size
                                                      : 0.0f)
        << " stride=" << stride;
  return result;
}

AirMappingVizServer::Json AirMappingVizServer::LoadHdMapJson(
    const std::filesystem::path& hdmap_path,
    const Eigen::Vector3d& utm_offset) const {
  std::ifstream input(hdmap_path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open HDMap: " + hdmap_path.string());
  }
  const std::string content((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
  if (IsGitLfsPointerContent(content)) {
    throw std::runtime_error(
        "HDMap file is a Git LFS pointer, not the real map content: " +
        hdmap_path.string() +
        ". Run `git lfs pull` (or fetch the actual file) before loading it.");
  }

  Json hdmap;
  hdmap["ok"] = true;
  hdmap["type"] = "hdmap";
  hdmap["features"] = Json::array();

  auto append_polygon_feature =
      [&](const std::string& feature_type, const std::string& search,
          const std::string& default_prefix) {
        size_t pos = 0;
        int count = 0;
        while ((pos = content.find(search, pos)) != std::string::npos) {
          if (search == "junction {" && pos >= 4 &&
              content.substr(pos - 4, 4) == "pnc_") {
            ++pos;
            continue;
          }
          size_t brace_count = 1;
          size_t end = content.find('{', pos) + 1;
          while (brace_count > 0 && end < content.size()) {
            if (content[end] == '{') {
              ++brace_count;
            } else if (content[end] == '}') {
              --brace_count;
            }
            ++end;
          }
          const std::string block = content.substr(pos, end - pos);
          const std::string polygon_block = ExtractBlock(block, "polygon");
          Json points = Json::array();
          size_t point_pos = 0;
          while (!polygon_block.empty() &&
                 (point_pos = polygon_block.find("point {", point_pos)) !=
                     std::string::npos) {
            const size_t point_end = polygon_block.find('}', point_pos);
            if (point_end == std::string::npos) {
              break;
            }
            const std::string point_block =
                polygon_block.substr(point_pos, point_end - point_pos);
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            FindDoubleAfterKey(point_block, "x", &x);
            FindDoubleAfterKey(point_block, "y", &y);
            FindDoubleAfterKey(point_block, "z", &z);
            Json point;
            point["x"] = x - utm_offset.x();
            point["y"] = y - utm_offset.y();
            point["z"] = z - utm_offset.z();
            points.push_back(point);
            point_pos = point_end + 1;
          }
          if (!points.empty()) {
            Json feature;
            const std::string id = ExtractId(block);
            feature["type"] = feature_type;
            feature["id"] =
                id.empty() ? default_prefix + "_" + std::to_string(count) : id;
            feature["points"] = points;
            hdmap["features"].push_back(feature);
            ++count;
          }
          pos = end;
        }
      };

  append_polygon_feature("crosswalk", "crosswalk {", "crosswalk");
  append_polygon_feature("junction", "junction {", "junction");

  size_t lane_pos = 0;
  while ((lane_pos = content.find("lane {", lane_pos)) != std::string::npos) {
    if (lane_pos > 0 && content[lane_pos - 1] != '\n' &&
        content[lane_pos - 1] != ' ') {
      ++lane_pos;
      continue;
    }
    size_t brace_count = 1;
    size_t lane_end = content.find('{', lane_pos) + 1;
    while (brace_count > 0 && lane_end < content.size()) {
      if (content[lane_end] == '{') {
        ++brace_count;
      } else if (content[lane_end] == '}') {
        --brace_count;
      }
      ++lane_end;
    }
    const std::string lane_block =
        content.substr(lane_pos, lane_end - lane_pos);
    const std::string id = ExtractId(lane_block);

    const std::string central_curve_block =
        ExtractBlock(lane_block, "central_curve");
    if (!central_curve_block.empty()) {
      const auto segments = ParseCurveSegments(central_curve_block, utm_offset);
      for (size_t i = 0; i < segments.size(); ++i) {
        Json feature;
        feature["type"] = "lane_center";
        feature["id"] = id + "_center_" + std::to_string(i);
        feature["points"] = segments[i];
        hdmap["features"].push_back(feature);
      }
    }

    const std::string left_boundary_block =
        ExtractBlock(lane_block, "left_boundary");
    if (!left_boundary_block.empty()) {
      const std::string curve_block = ExtractBlock(left_boundary_block, "curve");
      const std::string boundary_type =
          ExtractBoundaryType(left_boundary_block);
      const auto segments = ParseCurveSegments(curve_block, utm_offset);
      for (size_t i = 0; i < segments.size(); ++i) {
        Json feature;
        feature["type"] = "lane_left_boundary";
        feature["id"] = id + "_left_" + std::to_string(i);
        feature["boundary_type"] = boundary_type;
        feature["points"] = segments[i];
        hdmap["features"].push_back(feature);
      }
    }

    const std::string right_boundary_block =
        ExtractBlock(lane_block, "right_boundary");
    if (!right_boundary_block.empty()) {
      const std::string curve_block = ExtractBlock(right_boundary_block, "curve");
      const std::string boundary_type =
          ExtractBoundaryType(right_boundary_block);
      const auto segments = ParseCurveSegments(curve_block, utm_offset);
      for (size_t i = 0; i < segments.size(); ++i) {
        Json feature;
        feature["type"] = "lane_right_boundary";
        feature["id"] = id + "_right_" + std::to_string(i);
        feature["boundary_type"] = boundary_type;
        feature["points"] = segments[i];
        hdmap["features"].push_back(feature);
      }
    }
    lane_pos = lane_end;
  }

  size_t stop_pos = 0;
  int stop_count = 0;
  while ((stop_pos = content.find("stop_sign {", stop_pos)) !=
         std::string::npos) {
    size_t brace_count = 1;
    size_t stop_end = content.find('{', stop_pos) + 1;
    while (brace_count > 0 && stop_end < content.size()) {
      if (content[stop_end] == '{') {
        ++brace_count;
      } else if (content[stop_end] == '}') {
        --brace_count;
      }
      ++stop_end;
    }
    const std::string block = content.substr(stop_pos, stop_end - stop_pos);
    const std::string stop_line_block = ExtractBlock(block, "stop_line");
    Json points = Json::array();
    for (const auto& segment : ParseCurveSegments(stop_line_block, utm_offset)) {
      for (const auto& point : segment) {
        points.push_back(point);
      }
    }
    if (!points.empty()) {
      Json feature;
      const std::string id = ExtractId(block);
      feature["type"] = "stop_sign";
      feature["id"] =
          id.empty() ? "stop_sign_" + std::to_string(stop_count) : id;
      feature["points"] = points;
      hdmap["features"].push_back(feature);
      ++stop_count;
    }
    stop_pos = stop_end;
  }

  AINFO << "[air_mapping_viz] Loaded HDMap " << hdmap_path.string()
        << " features=" << hdmap["features"].size()
        << " offset=[" << utm_offset.transpose() << "]";
  return hdmap;
}

bool AirMappingVizServer::LoadUtmOffset(
    const std::filesystem::path& alignment_path, Eigen::Vector3d* offset) const {
  if (offset == nullptr) {
    return false;
  }
  offset->setZero();
  std::ifstream input(alignment_path);
  if (!input.is_open()) {
    return false;
  }
  std::string line;
  bool parsed = false;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const size_t colon = line.find(':');
    if (colon != std::string::npos) {
      const std::string key = Trim(line.substr(0, colon));
      const std::string value = Trim(line.substr(colon + 1));
      try {
        if (key == "offset_x") {
          offset->x() = std::stod(value);
          parsed = true;
        } else if (key == "offset_y") {
          offset->y() = std::stod(value);
          parsed = true;
        } else if (key == "offset_z") {
          offset->z() = std::stod(value);
          parsed = true;
        }
      } catch (const std::exception&) {
        return false;
      }
      continue;
    }

    std::istringstream stream(line);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (stream >> x >> y >> z) {
      *offset = Eigen::Vector3d(x, y, z);
      parsed = true;
      break;
    }
  }
  return parsed;
}

std::string AirMappingVizServer::ReadFile(
    const std::filesystem::path& path) const {
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    return "";
  }
  std::ostringstream stream;
  stream << input.rdbuf();
  return stream.str();
}

std::string AirMappingVizServer::GetMimeType(
    const std::filesystem::path& path) const {
  const auto extension = path.extension().string();
  if (extension == ".html") {
    return "text/html; charset=utf-8";
  }
  if (extension == ".js") {
    return "application/javascript; charset=utf-8";
  }
  if (extension == ".css") {
    return "text/css; charset=utf-8";
  }
  if (extension == ".json") {
    return "application/json; charset=utf-8";
  }
  return "text/plain; charset=utf-8";
}

}  // namespace viz
}  // namespace air_mapping
}  // namespace apollo
