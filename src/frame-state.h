#pragma once

#include "glm-include.h"
#include "object.h"

struct AnimationState {
  float clear_val = 0.0f;
  float model_rot = 0.0f;
};

struct FrameState {
  uint64_t frame_num = 0;
  AnimationState anim;
  std::vector<RenderObject> objects;
  mat4 model;
  mat4 view;
  mat4 proj;
};
