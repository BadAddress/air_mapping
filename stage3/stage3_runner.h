#pragma once

#include "modules/air_mapping/stage3/stage3_config.h"

namespace apollo {
namespace air_mapping {
namespace stage3 {

class Stage3Runner {
 public:
  bool Run(const Stage3Config& config);
};

}  // namespace stage3
}  // namespace air_mapping
}  // namespace apollo
