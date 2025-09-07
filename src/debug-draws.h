#pragma once

#include <vector>

#include "glm-include.h"
#include "render-objects.h"
#include "entities.h"

struct DebugDraws {
  DebugDraws() = default;

  void init(Entities* world, MaterialId mat1);
  void start();
  void finish(const mat4& viewProj);

  void drawSphere(vec3 position, float radius);
  void drawBox(vec3 position, vec3 size);

  void drawLine(vec3 a, vec3 b, vec4 color = vec4(1), float strokeWidth = .25f);
  void drawLine(vec2 a, vec2 b, vec4 color = vec4(1), float strokeWidth = .25f);
  void drawCircle(
      vec3 center, float radius, vec4 color = vec4(1),
      float strokeWidth = .25f);
  void drawCircle(
      vec2 center, float radius, vec4 color = vec4(1),
      float strokeWidth = .25f);

  Entities* world_ = nullptr;
  MaterialId mat1_ = kMaterialIdNone;

  bool recording_ = false;
  std::vector<EntityId> draws_;
  std::vector<Draw2d> draws2d_;
  struct WorldPositions {
    vec3 pos1{0};
    vec3 pos2{0};
  };
  std::vector<std::pair<size_t, WorldPositions>> worldPositions_;
};

extern DebugDraws gDebugDraws;

#define DbgSphere(position, radius) \
  gDebugDraws.drawSphere(position, radius)

#define DbgBox(position, size) \
  gDebugDraws.drawBox(position, size)

#define DbgCircle \
  gDebugDraws.drawCircle
#define DbgLine \
  gDebugDraws.drawLine
