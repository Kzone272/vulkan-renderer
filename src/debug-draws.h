#pragma once

#include <vector>

#include "entities.h"
#include "glm-include.h"
#include "render-objects.h"

struct DebugDraws {
  DebugDraws() = default;

  void init(Entities* world, MaterialId mat1);
  void start();
  void finish(const mat4& viewProj);

  EntityId getEntity();
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
  size_t nextEntity_ = 0;
  std::vector<EntityId> entities_;
  std::vector<Draw2d> draws2d_;
  struct WorldPositions {
    vec3 pos1{0};
    vec3 pos2{0};
  };
  std::vector<std::pair<size_t, WorldPositions>> worldPositions_;
};

extern DebugDraws gDebugDraws;

#define DbgSphere gDebugDraws.drawSphere
#define DbgBox gDebugDraws.drawBox

#define DbgCircle gDebugDraws.drawCircle
#define DbgLine gDebugDraws.drawLine
