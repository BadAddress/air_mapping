#pragma once

#include "modules/air_mapping/stage2/stage2_config.h"

namespace apollo {
namespace air_mapping {
namespace stage2 {

class Stage2Runner {
 public:
  bool Run(const Stage2Config& config);
};

}  // namespace stage2
}  // namespace air_mapping
}  // namespace apollo
