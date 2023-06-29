#pragma once

// Rough ideas for how rendering a world of objects might be set up.
// None of this code is used yet.

#include <map>
#include <string>

#include "glm-include.h"
#include "render-objects.h"

class Object {
 public:
  Object(ModelId model) : model_(model) {
  }

  void setScale(const vec3& scale) {
    scale_ = scale;
    dirty_ = true;
  }

  void setRot(float angle, const vec3& axis) {
    rot_angle_ = angle;
    rot_axis_ = axis;
    dirty_ = true;
  }

  void setPos(const vec3& pos) {
    pos_ = pos;
    dirty_ = true;
  }

  vec3 getPos() {
    return pos_;
  }

  mat4 getTransform() {
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
    transform_ = glm::scale(
        glm::rotate(glm::translate(mat4(1), pos_), rot_angle_, rot_axis_),
        scale_);
  }

  ModelId model_;

  bool dirty_ = false;
  mat4 transform_{1};
  vec3 scale_{1};
  float rot_angle_ = 0;
  vec3 rot_axis_{0, 1, 0};
  vec3 pos_{0};

  std::vector<std::unique_ptr<Object>> children_;
};

struct Camera {
  vec3 pos;
  vec3 focus;
  vec3 up;
};

struct World {
  // TODO: World probably shouldn't own the Model render object.
  std::map<ModelId, std::unique_ptr<Model>> loaded_models;
  std::vector<std::unique_ptr<Object>> objects;
};
