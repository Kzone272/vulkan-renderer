#pragma once

#include <vector>

#include "glm-include.h"
#include "render-objects.h"
#include "entities.h"

struct DebugDraws {
  DebugDraws() = default;

  void init(Entities* world, MaterialId mat1);
  void start();
  void finish();

  void drawSphere(vec3 position, float radius);
  void drawBox(vec3 position, vec3 size);

  Entities* world_ = nullptr;
  MaterialId mat1_ = kMaterialIdNone;

  bool recording_ = false;
  std::vector<EntityId> draws_;
};

extern DebugDraws gDebugDraws;

#define DbgSphere(position, radius) \
  gDebugDraws.drawSphere(position, radius)

#define DbgBox(position, size) \
  gDebugDraws.drawBox(position, size)