#include <string>

#include "gflags/gflags.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/stage2/stage2_config.h"
#include "modules/air_mapping/stage2/stage2_runner.h"

DEFINE_string(
    config, "/apollo_workspace/modules/air_mapping/conf/current_vehicle.yaml",
    "Path to air_mapping top-level vehicle yaml config.");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::air_mapping::stage2::Stage2Config config;
  if (!apollo::air_mapping::stage2::LoadStage2Config(FLAGS_config, &config)) {
    AERROR << "Failed to load stage2 config: " << FLAGS_config;
    return 1;
  }

  apollo::air_mapping::stage2::Stage2Runner runner;
  if (!runner.Run(config)) {
    AERROR << "air_mapping stage2_graph_opt failed";
    return 2;
  }

  AINFO << "air_mapping stage2_graph_opt finished";
  return 0;
}
