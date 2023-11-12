#pragma once

#include <memory>

#include "descriptors.h"
#include "fbo.h"
#include "pipelines.h"
#include "render-state.h"

struct Pass {
  Fbo fbo;
  std::vector<std::unique_ptr<DescLayout>> los;
  std::vector<std::unique_ptr<Pipeline>> pls;

  void init(const VulkanState& vs);

  DescLayout* makeDescLayout() {
    los.push_back(std::make_unique<DescLayout>());
    return los.back().get();
  }

  Pipeline* makePipeline() {
    pls.push_back(std::make_unique<Pipeline>());
    return pls.back().get();
  }
};
