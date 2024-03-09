#pragma once

// Rough ideas for how rendering a world of objects might be set up.
// None of this code is used yet.

#include <map>
#include <string>

#include "animation.h"
#include "glm-include.h"
#include "render-objects.h"

extern const std::map<ModelId, ModelInfo> kModelRegistry;

class Object {
 public:
  Object(
      ModelId model = ModelId::None,
      std::optional<mat4> model_transform = std::nullopt);
  
  // Move only.
  Object(Object&& other) = default;
  Object& operator=(Object&& other) = default;
  Object(const Object& other) = delete;
  Object& operator=(const Object&& other) = delete;

  void setParent(Object* parent);
  void setScale(const vec3& scale);
  void setRot(glm::quat rot);
  glm::quat getRot();
  void setPos(const vec3& pos);
  vec3 getPos();
  void setPosOffset(vec3 offset);
  const mat4& getTransform();

  // Transform from this object's space to an ancestor.
  const mat4 toAncestor(Object* ancestor);
  // Local space to world (root of tree).
  const mat4 toWorld();
  // From world space to local.
  const mat4 toLocal();
  // From ancestor's space to local.
  const mat4 toLocal(Object* ancestor);

  void addPosAnim(Animation<vec3>* a);
  void clearAddAnims();
  void addRotAnim(Animation<float>* a);
  void clearRotAnims();
  void animate(Time now);

  Object* addChild(Object&& child);
  void addChild(Object* child);
  const std::vector<Object*>& children();
  void clearChildren();

  std::vector<ModelId> getModels();
  void getSceneObjects(const mat4& parent, std::vector<SceneObject>& objs);

 private:
  void updateTransform();

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
