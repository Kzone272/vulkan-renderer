#pragma once

// Rough ideas for how rendering a world of objects might be set up.
// None of this code is used yet.

#include <map>
#include <string>

#include "animation.h"
#include "glm-include.h"

enum class ModelId {
  None,
  Viking,
  Pony,
  Cube,
  Bone,
  Tetra,
};

struct ModelInfo {
  std::string obj_path;
  std::string texture_path;
  mat4 model_transform{1};
};

std::map<ModelId, ModelInfo> model_registry = {
    {ModelId::Viking,
     {
         "assets/models/viking_room.obj",
         "assets/textures/viking_room.png",
         glm::rotate(mat4(1), glm::radians(-90.f), vec3(1, 0, 0)) *
             glm::scale(mat4(1), vec3(-300, 300, 300)),
     }},
    {ModelId::Pony,
     {
         "assets/models/pony/pony.obj",
         "assets/models/pony/pony-body-diffuse.jpg",
         glm::scale(mat4(1), vec3(0.5)),
     }},
};

struct RenderObject {
  ModelId model;
  mat4 transform;
};

class Object {
 public:
  Object(ModelId model, std::optional<mat4> model_transform = std::nullopt)
      : model_(model) {
    if (model_transform) {
      model_transform_ = *model_transform;
    } else {
      auto it = model_registry.find(model);
      if (it != model_registry.end()) {
        model_transform_ = it->second.model_transform;
      }
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
  glm::quat getRot() {
    return rot_;
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

  std::vector<ModelId> getModels() {
    std::vector<ModelId> models{model_};
    for (auto& child : children_) {
      auto child_models = child->getModels();
      models.insert(models.end(), child_models.begin(), child_models.end());
    }
    return models;
  }

  void setPosAnim(Animation a) {
    pos_anim_ = a;
  }

  void addPosAnim(Animation a) {
    add_anims_.push_back(a);
  }

  void clearAddAnims() {
    add_anims_.clear();
  }

  bool animate(Time now) {
    bool finished = false;
    if (pos_anim_) {
      setPos(Animation::sample(*pos_anim_, now));
      if (now > pos_anim_->to_time) {
        pos_anim_.reset();
        finished = true;
      }
    }

    // Erase finished animations.
    std::erase_if(add_anims_, [&now](auto& anim) {
      return !anim.loop && now > anim.to_time;
    });

    anim_pos_ = vec3(0);
    for (const auto& anim : add_anims_) {
      anim_pos_ += Animation::sample(anim, now);
      dirty_ = true;
    }

    return finished;
  }

  Object* addChild(std::unique_ptr<Object> child) {
    auto* ptr = child.get();
    children_.push_back(std::move(child));
    return ptr;
  }
  const std::vector<std::unique_ptr<Object>>& children() {
    return children_;
  }
  void clearChildren() {
    children_.clear();
  }

  void getRenderObjects(const mat4& parent, std::vector<RenderObject>& objs) {
    mat4 transform = parent * getTransform();
    if (model_ != ModelId::None) {
      objs.push_back({
          model_,
          transform * model_transform_,
      });
    }
    for (auto& child : children_) {
      child->getRenderObjects(transform, objs);
    }
  }

 private:
  void updateTransform() {
    transform_ = glm::translate(pos_ + anim_pos_) * glm::toMat4(rot_) *
                 glm::scale(scale_);
  }

  ModelId model_;
  // Transform that applies to this object's mesh only, and not to children.
  mat4 model_transform_{1};

  bool dirty_ = false;
  mat4 transform_{1};
  vec3 scale_{1};
  glm::quat rot_ = {1, {0, 0, 0}};
  vec3 pos_{0};
  vec3 anim_pos_{0};

  // Animation that sets absolute position.
  std::optional<Animation> pos_anim_;
  // Animations that add to current position.
  std::vector<Animation> add_anims_;
  std::vector<std::unique_ptr<Object>> children_;
};

enum class CameraType {
  Spin,
  Trackball,
  Fps,
  Follow,
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

struct FollowCamera {
  float dist;
  vec3 focus{0};
  float yaw = 0;
  float pitch = 0;
};
