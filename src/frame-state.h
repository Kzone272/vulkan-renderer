#pragma once

#include "glm-include.h"
#include "render-objects.h"

// TODO: move to app.h
struct AnimationState {
  float clear_val = 0.0f;
  float model_rot = 0.0f;
};

// TODO: move to render-objects.h
struct FrameState {
  uint64_t frame_num = 0;
  AnimationState anim;
  std::vector<RenderObject> objects;
  std::vector<Light> lights;
  mat4 model;
  mat4 view;
  mat4 proj;
  PostFxData post_fx;
};
