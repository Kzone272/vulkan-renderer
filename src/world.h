#pragma once

// Rough ideas for how rendering a world of objects might be set up.
// None of this code is used yet.

#include <map>
#include <string>

#include "glm-include.h"

enum class ModelId {
  UNKNOWN,
  VIKING,
};

struct ModelInfo {
  std::string model_path;
  std::string texture_path;
};

std::map<ModelId, ModelInfo> models = {
    {ModelId::VIKING,
     {"assets/models/viking_room.obj", "assets/textures/viking_room.png"}},
};

class Object {
 public:
  void setScale(const glm::vec3& scale) {
    scale_ = scale;
    dirty_ = true;
  }

  void setRot(float angle, const glm::vec3& axis) {
    rot_angle_ = angle;
    rot_axis_ = axis;
    dirty_ = true;
  }

  void setPos(const glm::vec3& pos) {
    pos_ = pos;
    dirty_ = true;
  }

  glm::mat4 getTransform() {
    if (dirty_) {
      updateTransform();
      dirty_ = false;
    }
    return transform_;
  }

  ModelId getModel() {
    return model_;
  }

 private:
  void updateTransform() {
    transform_ = {};
    glm::translate(
        glm::rotate(glm::scale(transform_, scale_), rot_angle_, rot_axis_),
        pos_);
  }

  ModelId model_;

  bool dirty_ = false;
  glm::mat4 transform_;
  glm::vec3 scale_;
  float rot_angle_;
  glm::vec3 rot_axis_;
  glm::vec3 pos_;

  std::vector<std::unique_ptr<Object>> children_;
};

struct ModelVulkan {
  // vertdata
  // texture
};

struct Camera {
  glm::vec3 pos;
  glm::vec3 focus;
  glm::vec3 up;
};

class World {
  void addObject(std::unique_ptr<Object> obj) {
    if (!loaded_models.contains(obj->getModel())) {
      loadModel(obj->getModel());
    }
    objects.push_back(std::move(obj));
  }

  void loadModel(ModelId model) {
  }

  std::map<ModelId, ModelVulkan> loaded_models;
  std::vector<std::unique_ptr<Object>> objects;
};
