#include "modules/air_mapping/viz/air_mapping_viz_server.h"

#include <csignal>

#include "cyber/cyber.h"
#include "gflags/gflags.h"

DEFINE_int32(port, 12322, "Air mapping viz web server port");
DEFINE_string(module_root, "/apollo_workspace/modules/air_mapping",
              "air_mapping module root exposed to frontend");
DEFINE_string(doc_root, "/apollo_workspace/modules/air_mapping/viz/frontend",
              "frontend document root");
DEFINE_string(config, "", "optional viz config yaml");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::cyber::Init(argv[0]);
  std::signal(SIGINT, apollo::cyber::OnShutdown);
  std::signal(SIGTERM, apollo::cyber::OnShutdown);

  apollo::air_mapping::viz::AirMappingVizServer::Options options;
  options.port = FLAGS_port;
  options.module_root = FLAGS_module_root;
  options.doc_root = FLAGS_doc_root;

  apollo::air_mapping::viz::AirMappingVizServer server(options);
  if (!FLAGS_config.empty() && !server.LoadConfig(FLAGS_config)) {
    return -1;
  }
  if (!server.Init()) {
    return -1;
  }
  server.Run();
  return 0;
}
