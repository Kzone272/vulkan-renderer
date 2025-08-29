#include "debug-draws.h"

#include "entities.h"
#include "asserts.h"

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

  recording_ = true;
}

void DebugDraws::finish() {
  recording_ = false;
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