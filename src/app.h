#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_image.h>
#include <SDL_vulkan.h>

#include <vector>

#include "defines.h"
#include "entities.h"
#include "glm-include.h"
#include "input.h"
#include "object.h"
#include "render-objects.h"
#include "renderer.h"
#include "skelly.h"
#include "time-include.h"

namespace {

constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;

}  // namespace

extern std::map<ModelId, ModelInfo> kModelRegistry;

class App {
 public:
  void run();

  // Apparently this can be called on another thread by the OS. That could
  // potentially cause problems in the future.
  static int SdlEventWatcher(void* data, SDL_Event* event);
  void WindowResized();
  void WindowMoved();

 private:
  void initWindow();
  void initImgui();

  void initFrameState();
  void setupWorld();
  vec3 getSkellyPos();
  void loadMaterials();
  void loadModels();
  void useMesh(ModelId model, const Mesh& mesh);
  void loadPrimitives();
  void remakeGrid(int grid);
  void setupLights();

  void mainLoop();
  void processEvents();
  // Returns true if app should quit.
  bool handleWindowEvent(const SDL_Event& event);
  void updateWindowSize();

  void update();
  void updateTime();
  void checkFps(Time now);

  void resetFrameState();
  float updateModelRotation();
  void updateObjects();
  void updateMaterials();

  void updateCamera();
  void updateSpinCamera();
  void updateTrackballCamera();
  vec3 getTrackballVec(ivec2 mouse);
  void updateFpsCamera();
  void updateYawPitch(float& yaw, float& pitch);
  void updateDist(float& dist);
  void updateFollowCamera();

  void handleInput();
  void updateProjectionMatrix();
  void flattenObjectTree();

  void updateImgui();
  bool ImGuiDebugData(DebugData& debug);

  void cleanup();

  SDL_Window* window_ = nullptr;

  bool quit_ = false;
  bool window_minimized_ = false;
  bool window_empty_ = false;
  bool window_resized_ = false;
  uint32_t width_ = WIDTH;
  uint32_t height_ = HEIGHT;

  Time start_;
  float time_s_;
  Time frame_time_;
  // Time in s since the last draw, as a float
  float time_delta_s_ = 0.0f;
  // Time in ms since the last draw, as a float
  float time_delta_ms_ = 0.0f;
  // Used to calculate FPS
  uint64_t last_fps_frame_ = 0;
  Time next_fps_time_;
  // Last 1s of frame times.
  std::vector<float> frame_times_;

  struct Options {
    CameraType cam_type = CameraType::Follow;
    bool animate = true;
    bool show_controls = false;
    bool bounce_objects = false;
    int grid_size = 2;
    bool tetra_in = true;
    int tetra_steps = 2;
    float time_scale = 1;
  } options_;
  const Options default_options_ = {};

  enum class Floor {
    Viking,
    Drawing,
    Voronoi,
    Gradient,
    NumFloors,
  };
  struct UiState {
    std::string fps;
    std::string skelly;
    std::string flatten;
    bool gooch = false;
    int floor = (int)Floor::Gradient;
  } ui_;
  struct Stats {
    float skelly_total = 0;
    int skelly_num = 0;
    float flatten_total = 0;
    int flatten_num = 0;
  } stats_;

  struct AnimationState {
    float clear_val = 0.0f;
    float model_rot = 0.0f;
  } anim_;
  std::vector<vec2> cell_centers_;

  Camera cam_;
  Trackball trackball_;
  FpsCamera fps_cam_;
  FollowCamera follow_cam_;

  InputState input_;
  FrameState frame_state_;
  std::unique_ptr<Renderer> renderer_;

  Entities world_;
  EntityId grid_ = kNoEntry;
  std::vector<EntityId> gridItems_;
  EntityId floor_ = kNoEntry;
  EntityId skellyId_ = kNoEntry;
  Skelly skelly_ = {&world_};

  struct AppMaterials {
    MaterialData botData;
    MaterialId bot = kMaterialIdNone;
    MaterialData cube_data;
    MaterialId cube = kMaterialIdNone;
    MaterialId cube2 = kMaterialIdNone;
    MaterialData bone_data;
    MaterialId bone = kMaterialIdNone;
    MaterialId control = kMaterialIdNone;
    MaterialId viking = kMaterialIdNone;
    MaterialId drawing = kMaterialIdNone;
    MaterialId voronoi = kMaterialIdNone;
    MaterialData gooch_data;
    MaterialId gooch = kMaterialIdNone;
    MaterialData gradient_floor_data;
    MaterialId gradient_floor = kMaterialIdNone;
  } mats_;
  std::vector<MaterialId> floor_mats_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};

bool ImGuiMaterial(MaterialData& data, std::string label);
