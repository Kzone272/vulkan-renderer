#pragma once

#include <print>
#include <vector>

#include "asserts.h"
#include "glm-include.h"
#include "object.h"
#include "transform.h"

struct WorldTree {
  WorldTree() {
    std::println("WorldTree()");
    initObj(&root_);
  }

  Object* root() {
    return &root_;
  }

  void addChild(Object* child) {
    root_.addChild(child);
    initObj(child);
  }
  void addChild(Object* child, Object* parent) {
    parent->addChild(child);
    initObj(child);
  }
  Object* addChild(Object&& child) {
    Object* child_p = root_.addChild(std::move(child));
    initObj(child_p);
    return child_p;
  }
  Object* addChild(Object&& child, Object* parent) {
    Object* child_p = parent->addChild(std::move(child));
    initObj(child_p);
    return child_p;
  }

  void initObj(Object* obj) {
    obj->setWorld(this);
    obj->setObjectIndex(objects_.size());
    objects_.push_back(obj);
    ts_.push_back({});
  }

  void setScale(size_t i, const vec3& scale) {
    ts_[i].scale = scale;
  }
  vec3 getScale(size_t i) const {
    return ts_[i].scale;
  }

  void setRot(size_t i, glm::quat rot) {
    ts_[i].rot = rot;
  }
  glm::quat getRot(size_t i) const {
    return ts_[i].rot;
  }

  void setPos(size_t i, const vec3& pos) {
    ts_[i].pos = pos;
  }

  vec3 getPos(size_t i) const {
    return ts_[i].pos;
  }

  mat4 matrix(size_t i) const {
    auto& t = ts_[i];
    auto m = glm::toMat4(t.rot);
    m[0] *= t.scale[0];
    m[1] *= t.scale[1];
    m[2] *= t.scale[2];
    m[3][0] = t.pos[0];
    m[3][1] = t.pos[1];
    m[3][2] = t.pos[2];
    return m;
  }

  void order() {
    objects_.clear();
    parents_.clear();
    model_ms_.clear();
    traverse(&root_);

    std::vector<TData> new_ts;
    new_ts.reserve(objects_.size());

    size_t index = 0;
    for (auto* obj : objects_) {
      size_t prev_ind = obj->getObjectIndex();
      if (prev_ind != -1) {
        new_ts.push_back(ts_[prev_ind]);
      } else {
        new_ts.push_back(TData());
      }
      obj->setObjectIndex(index);
      auto* parent = obj->getParent();
      parents_.push_back(parent ? parent->getObjectIndex() : -1);
      model_ms_.push_back(obj->getModelMatrix());

      index++;
    }

    ts_ = std::move(new_ts);
    root_ms_ = std::vector(objects_.size(), mat4(1));
  }

  void traverse(Object* obj) {
    objects_.push_back(obj);
    for (auto* child : obj->children()) {
      traverse(child);
    }
  }

  void getSceneObjects(
      const mat4& parent, std::vector<SceneObject>& objs,
      const std::set<ModelId>& hidden) {
    // Set locals
    std::vector<mat4> temp_ms(objects_.size(), mat4(1));

    for (size_t i = 1; i < objects_.size(); i++) {
      auto& t = ts_[i];
      temp_ms[i] = glm::toMat4(t.rot);
      temp_ms[i][0] *= t.scale[0];
      temp_ms[i][1] *= t.scale[1];
      temp_ms[i][2] *= t.scale[2];
      temp_ms[i][3][0] = t.pos[0];
      temp_ms[i][3][1] = t.pos[1];
      temp_ms[i][3][2] = t.pos[2];
    }

    for (size_t i = 1; i < objects_.size(); i++) {
      glm_mat4_mul(
          (glm_vec4*)&temp_ms[parents_[i]], (glm_vec4*)&temp_ms[i],
          (glm_vec4*)&root_ms_[i]);
      // root_ms_[i] = root_ms_[parents_[i]] * root_ms_[i];
    }
    for (size_t i = 1; i < objects_.size(); i++) {
      // root_ms_[i] = root_ms_[i] * model_ms_[i];
      glm_mat4_mul(
          (glm_vec4*)&root_ms_[i], (glm_vec4*)&model_ms_[i],
          (glm_vec4*)&temp_ms[i]);
    }

    for (size_t i = 1; i < objects_.size(); i++) {
      auto* obj = objects_[i];
      auto model = obj->getModel();
      if (model == ModelId::None || hidden.contains(model)) {
        continue;
      }

      objs.push_back({
          model,
          obj->getMaterial(),
          temp_ms[i],
      });
    }
  }

  struct TData {
    quat rot = glm::identity<quat>();
    vec3 pos = vec3(0);
    vec3 scale = vec3(1);
  };

  Object root_;
  std::vector<Object*> objects_;
  std::vector<size_t> parents_;
  std::vector<TData> ts_;
  std::vector<mat4> root_ms_;
  std::vector<mat4> model_ms_;
};
