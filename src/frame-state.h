#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_LEFT_HANDED
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct UniformBufferObject {
  alignas(16) glm::mat4 model;
  alignas(16) glm::mat4 view;
  alignas(16) glm::mat4 proj;
};

struct AnimationState {
  float clear_val = 0.0f;
  float model_rot = 0.0f;
};

struct FrameState {
  uint64_t frame_num = 0;
  AnimationState anim;
  UniformBufferObject ubo;
};
