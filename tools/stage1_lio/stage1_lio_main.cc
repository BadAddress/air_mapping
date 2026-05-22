#include <string>

#include "cyber/common/log.h"
#include "gflags/gflags.h"
#include "modules/air_mapping/stage1/stage1_config.h"
#include "modules/air_mapping/stage1/stage1_runner.h"

DEFINE_string(config,
              "/apollo_workspace/modules/air_mapping/conf/current_vehicle.yaml",
              "Path to air_mapping top-level vehicle yaml config.");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::air_mapping::stage1::Stage1Config config;
  if (!apollo::air_mapping::stage1::LoadStage1Config(FLAGS_config, &config)) {
    AERROR << "Failed to load stage1 config: " << FLAGS_config;
    return 1;
  }

  apollo::air_mapping::stage1::Stage1Runner runner;
  if (!runner.Run(config)) {
    AERROR << "air_mapping stage1_lio failed";
    return 2;
  }

  AINFO << "air_mapping stage1_lio finished";
  return 0;
}
