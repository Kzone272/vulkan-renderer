#include "debug-draws.h"

#include <print>

#include "entities.h"
#include "asserts.h"
#include "vec-maths.h"

DebugDraws gDebugDraws{};

void DebugDraws::init(Entities* world, MaterialId mat1) {
  world_ = world;
  mat1_ = mat1;
}

void DebugDraws::start() {
  for (auto ent : draws_) {
    world_->deleteEntity(ent);
  }
  draws_.clear();
  draws2d_.clear();
  worldPositions_.clear();

  recording_ = true;
}

void DebugDraws::finish(const mat4& viewProj) {
  recording_ = false;

  for (auto& [index, worldPs] : worldPositions_) {
    auto& draw = draws2d_[index];
    draw.pos1 = toNdc(viewProj, worldPs.pos1);
    draw.pos2 = toNdc(viewProj, worldPs.pos2);
  }
}

void DebugDraws::drawSphere(vec3 position, float radius) {
  ASSERT(recording_);

  auto id = world_->makeObject(ModelId::BallControl, mat1_);
  world_->setSkipTransform(id, true);
  world_->setDrawMatrix(
      id, glm::translate(position) * glm::scale(vec3(radius)));

  draws_.push_back(id);
}

void DebugDraws::drawBox(vec3 position, vec3 size) {
  ASSERT(recording_);

  auto id = world_->makeObject(ModelId::Cube, mat1_);
  world_->setSkipTransform(id, true);
  world_->setDrawMatrix(id, glm::translate(position) * glm::scale(size));

  draws_.push_back(id);
}

constexpr float kVhToNdc = 1.f / 100 * 2;

vec2 vhToNdc(vec2 p) {
  return p * kVhToNdc + vec2(-1);
}

float vhToNdc(float x) {
  return x * kVhToNdc;
}

void DebugDraws::drawLine(vec3 a, vec3 b, vec4 color, float strokeWidth) {
  ASSERT(recording_);

  worldPositions_.emplace_back(draws2d_.size(), WorldPositions{a, b});
  drawLine(vec2(0), vec2(0), color, strokeWidth);
}

void DebugDraws::drawLine(vec2 a, vec2 b, vec4 color, float strokeWidth) {
  ASSERT(recording_);

  draws2d_.push_back({
      .pos1 = vhToNdc(a),
      .pos2 = vhToNdc(b),
      .color = color,
      .values = {vhToNdc(strokeWidth), 0, 0, 0},
      .type = Draw2d::Type::Line,
  });
}

void DebugDraws::drawCircle(
    vec3 center, float radius, vec4 color, float strokeWidth) {
  ASSERT(recording_);

  worldPositions_.emplace_back(draws2d_.size(), WorldPositions{center});
  drawCircle(vec2(0), radius, color, strokeWidth);
}

void DebugDraws::drawCircle(
    vec2 center, float radius, vec4 color, float strokeWidth) {
  ASSERT(recording_);

  draws2d_.push_back({
      .pos1 = vhToNdc(center),
      .color = color,
      .values = {vhToNdc(strokeWidth), vhToNdc(radius), 0, 0},
      .type = Draw2d::Type::Circle,
  });
}
