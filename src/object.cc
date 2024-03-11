#include "object.h"

#include <map>
#include <string>

#include "animation.h"
#include "glm-include.h"
#include "render-objects.h"

const std::map<ModelId, ModelInfo> kModelRegistry = {
    {ModelId::Viking,
     {
         "assets/models/viking_room.obj",
         "assets/textures/viking_room.png",
         glm::rotate(glm::radians(-90.f), vec3(1, 0, 0)) *
             glm::scale(vec3(-300, 300, 300)),
     }},
    {ModelId::Pony,
     {
         "assets/models/pony/pony.obj",
         "assets/models/pony/pony-body-diffuse.jpg",
         glm::scale(vec3(0.5)),
     }},
};

Object::Object(ModelId model, std::optional<mat4> model_transform)
    : model_(model) {
  if (model_transform) {
    model_transform_ = *model_transform;
  } else {
    auto it = kModelRegistry.find(model);
    if (it != kModelRegistry.end()) {
      model_transform_ = it->second.model_transform;
    }
  }
}

void Object::setParent(Object* parent) {
  parent_ = parent;
}

void Object::setScale(const vec3& scale) {
  scale_ = scale;
  dirty_ = true;
}
vec3 Object::getScale() const {
  return scale_;
}

void Object::setRot(glm::quat rot) {
  rot_ = rot;
  dirty_ = true;
}
glm::quat Object::getRot() const {
  return rot_ * anim_rot_;
}

void Object::setPos(const vec3& pos) {
  pos_ = pos;
  dirty_ = true;
}

vec3 Object::getPos() const {
  return pos_ + pos_offset_ + anim_pos_;
}

void Object::setPosOffset(vec3 offset) {
  pos_offset_ = offset;
  dirty_ = true;
}

const mat4& Object::getTransform() {
  if (dirty_) {
    updateTransform();
    dirty_ = false;
  }
  return transform_;
}

// Transform from this object's space to an ancestor.
mat4 Object::toAncestor(Object* ancestor) {
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

vec3 Object::posToAncestor(Object* ancestor, vec3 pos) {
  return toAncestor(ancestor) * vec4(pos, 1);
}

// Local space to world (root of tree).
mat4 Object::toWorld() {
  return toAncestor(nullptr);
}

vec3 Object::posToWorld(vec3 pos) {
  return toWorld() * vec4(pos, 1);
}

// From world space to local.
mat4 Object::toLocal() {
  return glm::inverse(toWorld());
}

// From ancestor's space to local.
mat4 Object::toLocal(Object* ancestor) {
  return glm::inverse(toAncestor(ancestor));
}

vec3 Object::posToLocal(Object* ancestor, vec3 pos) {
  return toLocal(ancestor) * vec4(pos, 1);
}

std::vector<ModelId> Object::getModels() {
  std::vector<ModelId> models{model_};
  for (auto* child : children_) {
    auto child_models = child->getModels();
    models.insert(models.end(), child_models.begin(), child_models.end());
  }
  return models;
}

void Object::addPosAnim(Animation<vec3>* a) {
  pos_anims_.push_back(a);
}

void Object::clearAddAnims() {
  pos_anims_.clear();
}

void Object::addRotAnim(Animation<float>* a) {
  rot_anims_.push_back(a);
}

void Object::clearRotAnims() {
  rot_anims_.clear();
}

void Object::animate(Time now) {
  // Erase finished animations.
  std::erase_if(pos_anims_, [&now](auto& anim) {
    return !anim->loop_ && now > anim->to_time_;
  });
  std::erase_if(rot_anims_, [&now](auto& anim) {
    return !anim->loop_ && now > anim->to_time_;
  });

  anim_pos_ = vec3(0);
  for (auto* anim : pos_anims_) {
    anim_pos_ += anim->sample(now);
    dirty_ = true;
  }

  anim_rot_ = {1, {0, 0, 0}};
  for (auto* anim : rot_anims_) {
    // TOOD: Properly sort out the rotation axis.
    anim_rot_ = glm::angleAxis(anim->sample(now), vec3(-1, 0, 0)) * anim_rot_;
    dirty_ = true;
  }
}

Object* Object::addChild(Object&& child) {
  auto unique_child = std::make_unique<Object>(std::move(child));
  auto* ptr = unique_child.get();
  owned_children_.push_back(std::move(unique_child));
  addChild(ptr);
  return ptr;
}
void Object::addChild(Object* child) {
  children_.push_back(child);
  child->setParent(this);
}
const std::vector<Object*>& Object::children() {
  return children_;
}
void Object::clearChildren() {
  for (auto* child : children_) {
    child->clearChildren();
  }
  children_.clear();
  owned_children_.clear();
}

void Object::getSceneObjects(
    const mat4& parent, std::vector<SceneObject>& objs) {
  mat4 transform = parent * getTransform();
  if (model_ != ModelId::None) {
    objs.push_back({
        model_,
        transform * model_transform_,
    });
  }
  for (auto& child : children_) {
    child->getSceneObjects(transform, objs);
  }
}

void Object::updateTransform() {
  transform_ =
      glm::translate(getPos()) * glm::toMat4(getRot()) * glm::scale(scale_);
}
