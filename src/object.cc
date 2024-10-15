#include "object.h"

#include <map>
#include <string>

#include "animation.h"
#include "glm-include.h"
#include "render-objects.h"
#include "vec-maths.h"
#include "world-tree.h"

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
  world_->setScale(obj_ind_, scale);
}
vec3 Object::getScale() const {
  return world_->getScale(obj_ind_);
}

void Object::setRot(glm::quat rot) {
  world_->setRot(obj_ind_, rot);
}
glm::quat Object::getRot() const {
  if (rot_anims_.size()) {
    return anim_transform_.getRot() * world_->getRot(obj_ind_);
  } else {
    return world_->getRot(obj_ind_);
  }
}

void Object::setPos(const vec3& pos) {
  world_->setPos(obj_ind_, pos);
}

vec3 Object::getPos() const {
  if (pos_anims_.size()) {
    return world_->getPos(obj_ind_) + anim_transform_.getPos();
  } else {
    return world_->getPos(obj_ind_);
  }
}

// void Object::setTransform(const Transform& t) {
//   world_->= obj_ind_, t;
// }

// const Transform& Object::getTransform() const {
//   return world_->
// obj_ind_, };

const mat4& Object::matrix() {
  // if (pos_anims_.size() || rot_anims_.size()) {
  //   return Transform::addBlend(transform_, anim_transform_, 1).matrix();
  // } else {
  local_m_ = world_->matrix(obj_ind_);
  return local_m_;
  // }
}

// Transform from this object's space to an ancestor.
mat4 Object::toAncestor(Object* ancestor) {
  if (ancestor == this) {
    return mat4(1);
  }
  if (parent_ == nullptr) {
    // You should not request transform to an invalid ancestor.
    DASSERT(ancestor == nullptr);
    return matrix();
  }
  return parent_->toAncestor(ancestor) * matrix();
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

vec3 Object::posToLocal(vec3 pos) {
  return toLocal() * vec4(pos, 1);
}

// From ancestor's space to local.
mat4 Object::toLocal(Object* ancestor) {
  return glm::inverse(toAncestor(ancestor));
}

vec3 Object::posToLocal(Object* ancestor, vec3 pos) {
  return toLocal(ancestor) * vec4(pos, 1);
}

std::vector<std::pair<Object*, ModelId>> Object::getModels() {
  std::vector<std::pair<Object*, ModelId>> models = {{this, model_}};
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

  if (pos_anims_.size()) {
    vec3 anim_pos(0);
    for (auto* anim : pos_anims_) {
      anim_pos += anim->sample(now);
    }
    anim_transform_.setPos(anim_pos);
  }

  if (rot_anims_.size()) {
    quat anim_rot = glm::identity<quat>();
    for (auto* anim : rot_anims_) {
      // TOOD: Properly sort out the rotation axis.
      anim_rot = glm::angleAxis(anim->sample(now), vec3(-1, 0, 0)) * anim_rot;
    }
    anim_transform_.setRot(anim_rot);
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
    const mat4& parent, std::vector<SceneObject>& objs,
    const std::set<ModelId>& hidden) {
  mat4 local = matrix();
  mat4 root;
  fastMult(parent, local, root);
  if (model_ != ModelId::None && !hidden.contains(model_)) {
    mat4 final;
    fastMult(root, model_transform_, final);
    objs.push_back({
        model_,
        material_,
        final,
    });
  }
  for (auto& child : children_) {
    child->getSceneObjects(root, objs, hidden);
  }
}

Object* Object::lca(Object* o1, Object* o2) {
  std::vector<Object*> o1_path;
  std::vector<Object*> o2_path;

  Object* parent = o1->parent_;
  while (parent != nullptr) {
    o1_path.push_back(parent);
    parent = parent->parent_;
  }
  parent = o2->parent_;
  while (parent != nullptr) {
    o2_path.push_back(parent);
    parent = parent->parent_;
  }

  Object* lca = nullptr;
  for (auto it = std::make_pair(o1_path.rbegin(), o2_path.rbegin());
       it.first != o1_path.rend() && it.second != o2_path.rend();
       it.first++, it.second++) {
    if (*it.first == *it.second) {
      lca = *it.first;
    } else {
      break;
    }
  }
  return lca;
}

mat4 Object::matFromTo(Object* src, Object* dst) {
  auto* lca = Object::lca(src, dst);
  return dst->toLocal(lca) * src->toAncestor(lca);
}
