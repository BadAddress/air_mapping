#include <string>

#include "gflags/gflags.h"

#include "cyber/common/log.h"
#include "modules/air_mapping/stage3/stage3_config.h"
#include "modules/air_mapping/stage3/stage3_runner.h"

DEFINE_string(
    config,
    "/apollo_workspace/modules/air_mapping/conf/stage3_graph_refine.yaml",
    "Path to air_mapping stage3 graph refinement yaml config.");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::air_mapping::stage3::Stage3Config config;
  if (!apollo::air_mapping::stage3::LoadStage3Config(FLAGS_config, &config)) {
    AERROR << "Failed to load stage3 config: " << FLAGS_config;
    return 1;
  }

  apollo::air_mapping::stage3::Stage3Runner runner;
  if (!runner.Run(config)) {
    AERROR << "air_mapping stage3_graph_refine failed";
    return 2;
  }

  AINFO << "air_mapping stage3_graph_refine finished";
  return 0;
}
