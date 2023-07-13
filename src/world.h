#pragma once

// Rough ideas for how rendering a world of objects might be set up.
// None of this code is used yet.

#include <map>
#include <string>

#include "animation.h"
#include "glm-include.h"

enum class ModelId {
  UNKNOWN,
  VIKING,
  PONY,
  CUBE,
  MOVER,
  TETRA,
};

struct ModelInfo {
  std::string obj_path;
  std::string texture_path;
  mat4 init_transform;
};

std::map<ModelId, ModelInfo> model_registry = {
    {ModelId::VIKING,
     {
         "assets/models/viking_room.obj",
         "assets/textures/viking_room.png",
         glm::rotate(mat4(1), glm::radians(-90.f), vec3(1, 0, 0)) *
             glm::scale(mat4(1), vec3(-300, 300, 300)),
     }},
    {ModelId::PONY,
     {
         "assets/models/pony/pony.obj",
         "assets/models/pony/pony-body-diffuse.jpg",
         glm::scale(mat4(1), vec3(0.5)),
     }},
};

class Object {
 public:
  Object(ModelId model) : model_(model) {
    auto it = model_registry.find(model);
    if (it != model_registry.end()) {
      init_transform_ = it->second.init_transform;
      dirty_ = true;
    }
  }

  void setScale(const vec3& scale) {
    scale_ = scale;
    dirty_ = true;
  }

  void setRot(glm::quat rot) {
    rot_ = rot;
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

  void animZ(Animation a) {
    anims_.clear();  // TODO: Probably don't always want to clear all animations
    a.value = &pos_.z;
    anims_.push_back(a);
  }

  void update(Time now) {
    if (anims_.empty()) {
      return;
    }
    // Update animations, and erase finished ones.
    std::erase_if(
        anims_, [&now](auto& anim) { return Animation::update(anim, now); });
    dirty_ = true;
  }

 private:
  void updateTransform() {
    transform_ = glm::translate(mat4(1), pos_) * glm::toMat4(rot_) *
                 glm::scale(mat4(1), scale_) * init_transform_;
  }

  ModelId model_;
  mat4 init_transform_{1};

  bool dirty_ = false;
  mat4 transform_{1};
  vec3 scale_{1};
  glm::quat rot_ = {1, {0, 0, 0}};
  vec3 pos_{0};

  std::vector<Animation> anims_;
  std::vector<std::unique_ptr<Object>> children_;
};

enum class CameraType {
  Spin,
  Trackball,
  Fps,
};

struct Camera {
  vec3 pos;
  vec3 focus;
  vec3 up;
};

struct Trackball {
  float dist;
  vec3 focus;
  glm::quat rot;
};

struct FpsCamera {
  vec3 pos{0};
  float yaw = 0;
  float pitch = 0;
};

struct World {
  std::vector<std::unique_ptr<Object>> objects;
};
