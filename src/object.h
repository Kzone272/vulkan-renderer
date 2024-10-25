#pragma once

// Rough ideas for how rendering a world of objects might be set up.
// None of this code is used yet.

#include <map>
#include <set>
#include <string>

#include "animation.h"
#include "glm-include.h"
#include "render-objects.h"
#include "transform.h"

class WorldTree;

extern const std::map<ModelId, ModelInfo> kModelRegistry;

class Object {
 public:
  Object(
      WorldTree* world, ModelId model = ModelId::None,
      std::optional<mat4> model_transform = std::nullopt);

  // Move only.
  Object(Object&& other) = default;
  Object& operator=(Object&& other) = default;
  Object(const Object& other) = delete;
  Object& operator=(const Object&& other) = delete;

  // Lowest Common Ancestor
  static Object* lca(Object* o1, Object* o2);
  static mat4 matFromTo(Object* src, Object* dst);

  void setParent(Object* parent);
  Object* getParent() {
    return parent_;
  }
  void setScale(const vec3& scale);
  vec3 getScale() const;
  void setRot(glm::quat rot);
  glm::quat getRot() const;
  void setPos(const vec3& pos);
  vec3 getPos() const;
  void setTransform(const Transform& t);
  const Transform& getTransform() const;
  const mat4& matrix();

  void setMaterial(MaterialId material) {
    material_ = material;
  }

  // Transform from this object's space to an ancestor.
  mat4 toAncestor(Object* ancestor);
  vec3 posToAncestor(Object* ancestor, vec3 pos = vec3(0, 0, 0));
  // Local space to world (root of tree).
  mat4 toWorld();
  vec3 posToWorld(vec3 pos = vec3(0, 0, 0));
  // From world space to local.
  mat4 toLocal();
  vec3 posToLocal(vec3 pos = vec3(0, 0, 0));
  // From ancestor's space to local.
  mat4 toLocal(Object* ancestor);
  vec3 posToLocal(Object* ancestor, vec3 pos = vec3(0, 0, 0));

  void addPosAnim(Animation<vec3>* a);
  void clearAddAnims();
  void addRotAnim(Animation<float>* a);
  void clearRotAnims();
  void animate(Time now);

  void setObjectIndex(size_t ind) {
    obj_ind_ = ind;
  }
  size_t getObjectIndex() {
    return obj_ind_;
  }

  Object* addChild(Object&& child);
  void addChild(Object* child);
  const std::vector<Object*>& children();
  void clearChildren();

  ModelId getModel() {
    return model_;
  }
  MaterialId getMaterial() {
    return material_;
  }
  const mat4& getModelMatrix() {
    return model_transform_;
  }

  void getModels(std::vector<std::pair<Object*, ModelId>>& pairs);
  void getSceneObjects(
      const mat4& parent, std::vector<SceneObject>& objs,
      const std::set<ModelId>& hidden);

 private:
  ModelId model_;
  MaterialId material_ = kMaterialIdNone;
  // Transform that applies to this object's mesh only, and not to children.
  mat4 model_transform_{1};
  mat4 local_m_{1};

  WorldTree* world_ = nullptr;
  Object* parent_ = nullptr;
  size_t obj_ind_ = -1;
  // Transform transform_;
  Transform anim_transform_;

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
