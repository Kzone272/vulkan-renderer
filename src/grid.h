#pragma once

#include "entities.h"
#include "render-objects.h"

struct Grid {
  Grid(Entities* world, MaterialId mat1, MaterialId mat2)
      : world_(world), mat1_(mat1), mat2_(mat2) {
    gridId_ = world_->makeObject();
    world_->setUpdater(
        gridId_, std::bind(&Grid::update, this, std::placeholders::_1));
  }

  void makeGrid(int size) {
    world_->deleteRange(gridRange_);
    gridRange_ = world_->createRange(size * size);

    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        EntityId id = gridRange_ + i * size + j;

        ModelId model = ((i + j) % 2 == 0) ? ModelId::Cube : ModelId::Tetra;
        MaterialId material = (i % 2) == 0 ? mat1_ : mat2_;
        mat4 model_t = glm::scale(vec3(100));

        world_->setModel(id, model);
        world_->setMaterial(id, material);
        world_->setModelMatrix(id, model_t);
        world_->setParent(id, gridId_);
        world_->setPos(id, 500.f * vec3(i - size / 2, 0, j - size / 2));
      }
    }
  }

  void update(float deltaS) {
    updateModelRotation(deltaS);
    auto spin = glm::angleAxis(glm::radians(itemRot_), vec3(0.f, 1.f, 0.f));

    auto transforms = world_->getTransforms(gridRange_);
    for (uint32_t i = 0; i < transforms.size(); i++) {
      auto& transform = transforms[i];
      if (bounce_) {
        float dist = glm::length(transform.pos);
        float t = dist / 600.f + 4 * deltaS;
        float height = sinf(t) * 25.f;
        transform.pos.y = height;
      }

      transform.rot = spin;
    }
  }

  void updateModelRotation(float deltaS) {
    static const float seqDurS = 4;
    static const float totalRot = 360;
    static float seq = 0;
    seq += deltaS / seqDurS;
    while (seq > 1) {
      seq -= 1;
    }
    itemRot_ = seq * totalRot;
  }

  void setBounce(bool value) {
    bounce_ = value;
  }

  Entities* world_ = nullptr;
  EntityId gridId_ = kNoEntry;
  RangeId gridRange_ = kNoEntry;
  MaterialId mat1_ = kMaterialIdNone;
  MaterialId mat2_ = kMaterialIdNone;

  float itemRot_ = 0;
  bool bounce_ = false;
};
