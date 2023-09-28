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
  Control,
  Tetra,
  Floor,
};

struct ModelInfo {
  std::string obj_path;
  std::string texture_path;
  mat4 model_transform{1};
};

struct Texture;

struct MaterialInfo {
  // TODO: Make this a TextureId
  Texture* diffuse_texture;
  std::optional<std::string> diffuse_path;
  struct UniformBufferObject {
    vec3 color = {1, 1, 1};
  } ubo;
};
typedef uint32_t MaterialId;

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
  Object(
      ModelId model = ModelId::None,
      std::optional<mat4> model_transform = std::nullopt)
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

  void setParent(Object* parent) {
    parent_ = parent;
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
    return rot_ * anim_rot_;
  }

  void setPos(const vec3& pos) {
    pos_ = pos;
    dirty_ = true;
  }

  vec3 getPos() {
    return pos_ + pos_offset_ + anim_pos_;
  }

  void setPosOffset(vec3 offset) {
    pos_offset_ = offset;
    dirty_ = true;
  }

  const mat4& getTransform() {
    if (dirty_) {
      updateTransform();
      dirty_ = false;
    }
    return transform_;
  }

  // Transform from this object's space to an ancestor.
  const mat4 toAncestor(Object* ancestor) {
    if (ancestor == this) {
      return mat4(1);
    }
    if (parent_ == nullptr) {
      // You should not request transform to an invalid ancestor.
      DASSERT(ancestor == nullptr);
      return getTransform();
    }
    return parent_->toAncestor(ancestor) * getTransform();
  }

  // Local space to world (root of tree).
  const mat4 toWorld() {
    return toAncestor(nullptr);
  }

  // From world space to local.
  const mat4 toLocal() {
    return glm::inverse(toWorld());
  }

  // From ancestor's space to local.
  const mat4 toLocal(Object* ancestor) {
    return glm::inverse(toAncestor(ancestor));
  }

  std::vector<ModelId> getModels() {
    std::vector<ModelId> models{model_};
    for (auto* child : children_) {
      auto child_models = child->getModels();
      models.insert(models.end(), child_models.begin(), child_models.end());
    }
    return models;
  }

  void addPosAnim(Animation<vec3>* a) {
    pos_anims_.push_back(a);
  }

  void clearAddAnims() {
    pos_anims_.clear();
  }

  void addRotAnim(Animation<float>* a) {
    rot_anims_.push_back(a);
  }

  void clearRotAnims() {
    rot_anims_.clear();
  }

  void animate(Time now) {
    // Erase finished animations.
    std::erase_if(pos_anims_, [&now](auto& anim) {
      return !anim->loop && now > anim->to_time;
    });
    std::erase_if(rot_anims_, [&now](auto& anim) {
      return !anim->loop && now > anim->to_time;
    });

    anim_pos_ = vec3(0);
    for (auto* anim : pos_anims_) {
      anim_pos_ += sampleAnimation(*anim, now);
      dirty_ = true;
    }

    anim_rot_ = {1, {0, 0, 0}};
    for (auto* anim : rot_anims_) {
      anim_rot_ =
          glm::angleAxis(sampleAnimation(*anim, now), anim->axis) * anim_rot_;
      dirty_ = true;
    }
  }

  Object* addChild(std::unique_ptr<Object> child) {
    auto* ptr = child.get();
    owned_children_.push_back(std::move(child));
    addChild(ptr);
    return ptr;
  }
  void addChild(Object* child) {
    children_.push_back(child);
    child->setParent(this);
  }
  const std::vector<Object*>& children() {
    return children_;
  }
  void clearChildren() {
    for (auto* child : children_) {
      child->clearChildren();
    }
    children_.clear();
    owned_children_.clear();
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
    transform_ =
        glm::translate(getPos()) * glm::toMat4(getRot()) * glm::scale(scale_);
  }

  ModelId model_;
  // Transform that applies to this object's mesh only, and not to children.
  mat4 model_transform_{1};

  Object* parent_ = nullptr;
  bool dirty_ = false;
  mat4 transform_{1};
  vec3 scale_{1};
  glm::quat rot_ = {1, {0, 0, 0}};
  glm::quat anim_rot_ = {1, {0, 0, 0}};
  vec3 pos_{0};
  vec3 anim_pos_{0};

  // Offset from the current position.
  vec3 pos_offset_{0};
  // Animations that add to current position.
  std::vector<Animation<vec3>*> pos_anims_;
  std::vector<Animation<float>*> rot_anims_;
  std::vector<Object*> children_;
  std::vector<std::unique_ptr<Object>> owned_children_;
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
