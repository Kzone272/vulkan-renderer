#include "pass.h"

void Pass::init(const VulkanState& vs) {
  fbo.init(vs);

  for (auto& lo : los) {
    lo->init(vs);
    lo->alloc(vs, vs.kMaxFramesInFlight);
  }

  for (auto& pl : pls) {
    initPipeline(vs.device, fbo, *pl);
  }
}
