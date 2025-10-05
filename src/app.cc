#include "app.h"

#include <imgui/backends/imgui_impl_sdl2.h>
#include <imgui/imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <print>
#include <set>
#include <vector>

#include "animation.h"
#include "asserts.h"
#include "assets.h"
#include "debug-draws.h"
#include "grid.h"
#include "imgui-helpers.h"
#include "maths.h"
#include "primitives.h"
#include "renderer.h"
#include "vec-maths.h"

App::App() = default;
App::~App() = default;

void App::run() {
  initWindow();
  renderer_ = std::make_unique<Renderer>(window_, width_, height_);

  initImgui();
  renderer_->init(&frame_state_);

  setupWorld();
  initFrameState();
  mainLoop();

  cleanup();
}

// Apparently this can be called on another thread by the OS. That could
// potentiall cause problems in the future.
int App::SdlEventWatcher(void* data, SDL_Event* event) {
  if (event->type == SDL_WINDOWEVENT) {
    if (event->window.event == SDL_WINDOWEVENT_RESIZED) {
      reinterpret_cast<App*>(data)->WindowResized();
    } else if (event->window.event == SDL_WINDOWEVENT_MOVED) {
      reinterpret_cast<App*>(data)->WindowMoved();
    }
  }
  return 0;
}

void App::WindowResized() {
  window_resized_ = true;
  update();
}

void App::WindowMoved() {
  update();
}

void App::initWindow() {
  ASSERT(SDL_Init(SDL_INIT_VIDEO) == 0);

  uint32_t window_flags = SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE;
  window_ = SDL_CreateWindow(
      "Vulkan Renderer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width_,
      height_, window_flags);
  if (!window_) {
    std::string error(SDL_GetError());
    std::println("{}", error);
    ASSERT(false);
  }
  SDL_AddEventWatch(SdlEventWatcher, this);
}

void App::initImgui() {
  ImGui::CreateContext();
  ImGui_ImplSDL2_InitForVulkan(window_);
  ImGui::GetIO().ConfigWindowsMoveFromTitleBarOnly = true;
}

void App::initFrameState() {
  updateProjectionMatrix();

  spinCam_ = {
      .pos{0, 300, -300 * options_.grid_size}, .focus{0, 0, 0}, .up{0, 1, 0}};
  trackball_ = {
      .dist = 300.f * static_cast<float>(options_.grid_size),
      .focus{0, 0, 0},
      .rot{1, 0, 0, 0}};
  fps_cam_.pos = {0, 300, -100 * options_.grid_size / 2};
  follow_cam_ = {
      .dist = 800.f,
      .focus = getSkellyPos(),
      .yaw = 0,
      .pitch = -30,
  };
  updateCamera();
}

void App::setupWorld() {
  loadModels();
  loadMaterials();
  gDebugDraws.init(&world_, mats_.gooch);

  skellyId_ = skelly_.getEntity();
  world_.setPos(skellyId_, vec3(200, 0, 0));
  skelly_.setMaterials(mats_.bot, kMaterialIdNone);

  grid_ = std::make_unique<Grid>(&world_, mats_.cube, mats_.cube2);
  grid_->makeGrid(options_.grid_size);

  floor_ = world_.makeObject(ModelId::Floor, floor_mats_[ui_.floor]);

  auto& ponyInfo = kModelRegistry[ModelId::Pony];
  auto car = world_.makeObject(
      ModelId::Pony, ponyInfo.material, ponyInfo.model_transform);
  world_.setPos(car, vec3(-300, -1, 0));

  auto& vikingInfo = kModelRegistry[ModelId::Viking];
  auto viking = world_.makeObject(
      ModelId::Viking, vikingInfo.material, vikingInfo.model_transform);
  world_.setPos(viking, vec3(300, 1, 300));

  auto ball =
      world_.makeObject(ModelId::Sphere, mats_.cube, glm::scale(vec3(100)));
  world_.setPos(ball, vec3(-300, 100, 600));

  loadPrimitives();

  // Random points. Chose by fair dice roll.
  cell_centers_ = {
      vec2(0.5, 0.5),   vec2(0.75, -0.6), vec2(-0.5, 0.75), vec2(0.8, 0.4),
      vec2(-0.3, -0.2), vec2(-0.9, 0.8),  vec2(0.3, -0.6),  vec2(-0.8, -0.1),
  };
  frame_state_.voronoi_cells.push_back({.color{1, 0, 0}});
  frame_state_.voronoi_cells.push_back({.color{0, 1, 1}});
  frame_state_.voronoi_cells.push_back({.color{0, 0, 1}});
  frame_state_.voronoi_cells.push_back({.color{1, 0.8, 0}});
  frame_state_.voronoi_cells.push_back({.color{0, 1, 0}});
  frame_state_.voronoi_cells.push_back({.color{1, 1, 0}});
  frame_state_.voronoi_cells.push_back({.color{1, 0, 1}});
  frame_state_.voronoi_cells.push_back({.color{0.2, 0.8, 0.6}});

  setupLights();
}

vec3 App::getSkellyPos() {
  return world_.getPos(skellyId_);
}

void App::loadMaterials() {
  mats_.cube_data = {
      .color1{0, 0.06, 1},
      .color2{0.88, 0, 1},
      .color3{0.011f, 0.000f, 0.358f, 1.000f},
      .color4{0.245f, 0.000f, 0.000f, 1.000f},
      .data1{0, 0, 1, 0},  // left -> right
  };
  mats_.cube = renderer_->useMaterial(
      {.data = mats_.cube_data, .pipeline = ScenePipeline::Gradient});

  mats_.gradient_floor_data = {
      .color1{0.912, 1, 0},
      .color2{0, 0.314, 0.212},
      .color3{0.511f, 0.789f, 0.000f, 1.000f},
      .color4{0.000f, 0.121f, 0.216f, 1.000f},
      .data1{0, 0, 1, 1},  // top left -> bottom right
  };
  mats_.gradient_floor = renderer_->useMaterial(
      {.data = mats_.gradient_floor_data, .pipeline = ScenePipeline::Gradient});

  mats_.cube2 = renderer_->useMaterial({.data{.color1{0.8, 0.8, 0}}});

  mats_.botData = {
      .color1{0.335, 0.877, 0.862},
      .color2{0.056, 0.103, 0.284},
      .color3{0.000f, 0.471f, 1.000f, 1.000f},
      .color4{0.030f, 0.017f, 0.162f, 1.000f},
      .data1{0, 0, 0, 1},  // placeholder
  };
  mats_.bot = renderer_->useMaterial(
      {.data = mats_.botData, .pipeline = ScenePipeline::Gradient});

  mats_.bone_data = {
      .color1{0.9, 0.2, 0.1},
      .color2{0.25, 0.02, 0.1},
  };
  mats_.bone = renderer_->useMaterial({.data = mats_.bone_data});

  mats_.control = renderer_->useMaterial({.data{.color1{0.1, 1, 0.2}}});
  mats_.viking = renderer_->useMaterial({
      .diffuse_path = "assets/textures/viking_room.png",
      .data{.color1{0.2, 0.2, 0.2}},
  });
  mats_.drawing = renderer_->useMaterial({
      .diffuse_texture = renderer_->getDrawingTexture(),
      .data{.color1{0.2, 0.2, 0.2}},
  });
  mats_.voronoi = renderer_->useMaterial({
      .diffuse_texture = renderer_->getVoronoiTexture(),
      .data{.color1{0.2, 0.2, 0.2}},
  });
  floor_mats_ = {
      mats_.viking, mats_.drawing, mats_.voronoi, mats_.gradient_floor};
  mats_.gooch_data = {
      .color1{1, 0.8, 0},
      .color2{0, 0.2, 1},
      .type = MaterialData::Type::Gooch,
  };
  mats_.gooch = renderer_->useMaterial({.data = mats_.gooch_data});
}

void App::loadModels() {
  for (auto& [model, modelInfo] : kModelRegistry) {
    modelInfo.material =
        renderer_->useMaterial({.diffuse_path = modelInfo.texture_path});

    Mesh mesh = loadObj(modelInfo.obj_path);
    renderer_->useMesh(model, mesh);
  }
}

void App::useMesh(ModelId model, const Mesh& mesh) {
  renderer_->useMesh(model, mesh);
}

void App::loadPrimitives() {
  auto cube = makeCube();
  useMesh(ModelId::Cube, cube);
  useMesh(ModelId::Bone, cube);
  useMesh(ModelId::BoxControl, cube);
  useMesh(ModelId::BallControl, makeSphere(8));
  useMesh(ModelId::Floor, makePlane(10000, 10000));
  useMesh(ModelId::Tetra, tetrahedron(options_.tetra_steps, options_.tetra_in));
  useMesh(ModelId::Sphere, makeSphere(20));
}

void App::setupLights() {
  auto& lights = frame_state_.scene.lights;
  for (size_t i = 0; i < lights.size(); i++) {
    lights[i].type = Light::Type::None;
  }

  // Mostly downward sun
  lights[0] = {
      .vec = vec4(0.3, -1, 0, 0),
      .color = vec4(1),
      .type = Light::Type::Directional,
  };
  // Corner torch
  lights[1] = {
      .vec = vec4(385, 200, 475, 1),
      .color = vec4(1, 1, 0, 400),
      .type = Light::Type::Point,
  };
  // Front torch
  lights[2] = {
      .vec = vec4(385, 200, 110, 1),
      .color = vec4(1, 1, 0, 400),
      .type = Light::Type::Point,
  };
  // Cauldron
  lights[3] = {
      .vec = vec4(270, 20, 300, 1),
      .color = vec4(2, 1.2, 0, 500),
      .type = Light::Type::Point,
  };
  // Right tail light
  lights[4] = {
      .vec = vec4(-225, 40, -180, 1),
      .color = vec4(4, 0, 0, 500),
      .type = Light::Type::Point,
  };
  // Left tail light
  lights[5] = {
      .vec = vec4(-370, 40, -180, 1),
      .color = vec4(4, 0, 0, 500),
      .type = Light::Type::Point,
  };
}

void App::mainLoop() {
  while (true) {
    processEvents();
    if (quit_) {
      break;
    }
    update();
  }
}

void App::processEvents() {
  resetRelativeInput(input_);

  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    if (event.type == SDL_WINDOWEVENT) {
      if (handleWindowEvent(event)) {
        break;
      }
    }
    processInputState(event, input_);
  }
}

// Returns true if app should quit.
bool App::handleWindowEvent(const SDL_Event& event) {
  if (event.window.event == SDL_WINDOWEVENT_CLOSE) {
    quit_ = true;
    return true;
  } else if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
    window_resized_ = true;
  } else if (event.window.event == SDL_WINDOWEVENT_MINIMIZED) {
    window_minimized_ = true;
  } else if (event.window.event == SDL_WINDOWEVENT_RESTORED) {
    window_minimized_ = false;
  }
  return false;
}

void App::updateWindowSize() {
  int width = 0;
  int height = 0;
  SDL_GL_GetDrawableSize(window_, &width, &height);
  if (width == 0 || height == 0) {
    window_empty_ = true;
    return;
  }
  width_ = width;
  height_ = height;
  window_empty_ = false;
}

void App::update() {
  updateTime();

  gDebugDraws.start();

  if (window_resized_) {
    window_resized_ = false;
    updateWindowSize();
    updateProjectionMatrix();
    renderer_->resizeWindow(width_, height_);
  }

  bool should_draw = !window_minimized_ && !window_empty_;
  if (should_draw) {
    resetFrameState();
    updateImgui();
    handleInput();
    if (options_.animate) {
      updateObjects();
    }
    updateCamera();

    flattenObjectTree();
    updateMaterials();

    gDebugDraws.finish(viewProj_);
    updateDraws();
    renderer_->drawFrame(&frame_state_);
    frame_state_.frame_num++;
  }
}

void App::updateTime() {
  Time now = Clock::now();
  if (frame_state_.frame_num == 0) {
    start_ = now;
    frame_time_ = now;
  }

  time_s_ = FloatS(now - start_).count();

  auto time_delta = now - frame_time_;
  time_delta_s_ = options_.time_scale * FloatS(time_delta).count();
  time_delta_ms_ = options_.time_scale * FloatMs(time_delta).count();
  frame_times_.push_back(FloatMs(time_delta).count());

  frame_time_ = now;

  checkFps(now);
}

void App::checkFps(Time now) {
  if (frame_state_.frame_num == 0) {
    next_fps_time_ = now + 1s;
    return;
  }

  if (now > next_fps_time_) {
    float time_sum = 0.f;
    for (auto time : frame_times_) {
      time_sum += time;
    }
    float avg_time = time_sum / frame_times_.size();
    frame_times_.clear();

    int fps = frame_state_.frame_num - last_fps_frame_;
    ui_.fps = std::format("{:.2f}ms avg frame ({} fps)\n", avg_time, fps);

    auto& gridTime = world_.getTime(grid_->id());
    ui_.grid = std::format("Grid avg time: {:.3f}ms", gridTime.avg());
    gridTime.reset();

    auto& skellyTime = world_.getTime(skellyId_);
    ui_.skelly = std::format("Skeleton avg time: {:.3f}ms", skellyTime.avg());
    skellyTime.reset();

    ui_.flatten =
        std::format("Flatten tree avg time: {:.3f}ms", stats_.flatten.avg());
    stats_.flatten.reset();

    next_fps_time_ = now + 1s;
    last_fps_frame_ = frame_state_.frame_num;
  }
}

void App::resetFrameState() {
  bool firstFrame = frame_state_.frame_num == 0;
  frame_state_.update_drawing = firstFrame;
  frame_state_.update_voronoi = firstFrame;
  frame_state_.drawsNeedUpdate = firstFrame;
}

float App::updateModelRotation() {
  constexpr float seq_dur_ms = 4000;
  constexpr float total_rot = 360;
  static float seq = 0;
  seq += time_delta_ms_ / seq_dur_ms;
  while (seq > 1) {
    seq -= 1;
  }
  return seq * total_rot;
}

void App::updateObjects() {
  world_.update(time_delta_s_);

  if ((Floor)ui_.floor == Floor::Voronoi) {
    for (int i = 0; i < cell_centers_.size(); i++) {
      frame_state_.voronoi_cells[i].pos =
          cell_centers_[i] +
          glm::rotate(vec2(0, 0.1), time_s_ / glm::length(cell_centers_[i]));
    }
    frame_state_.update_voronoi = true;
  }
}

void App::updateMaterials() {
  const mat4& drawM = world_.getDrawMatrix(skellyId_);
  vec3 end = drawM[3];
  vec3 start = drawM * vec4(skelly_.getTopOfHead(), 1);
  DbgSphere(start, 5);
  DbgBox(end, vec3(5));

  mats_.botData.data1 =
      vec4(toScreenSpace(start, viewProj_), toScreenSpace(end, viewProj_));
  renderer_->updateMaterial(mats_.bot, mats_.botData);
}

void App::updateCamera() {
  if (options_.cam_type == CameraType::Spin) {
    updateSpinCamera();
  } else if (options_.cam_type == CameraType::Trackball) {
    updateTrackballCamera();
  } else if (options_.cam_type == CameraType::Fps) {
    updateFpsCamera();
  } else if (options_.cam_type == CameraType::Follow) {
    updateFollowCamera();
  } else {
    ASSERT(false);
  }
  viewProj_ = frame_state_.scene.proj * frame_state_.scene.view;

  // TODO: Do a better job at making these matrices.
  vec3 sunPos =
      follow_cam_.focus + 500.f * frame_state_.scene.lights[0].vec.xyz();
  mat4 sunView = glm::lookAt(sunPos, follow_cam_.focus, vec3(0, 1, 0));
  // Width / Height ratio of shadow map texture.
  float aspect = 1;
  float span = 500;
  mat4 sunProj =
      glm::ortho(span * aspect, -span * aspect, span, -span, 0.f, 1000.f);
  frame_state_.scene.shadowViewProj = sunProj * sunView;
}

void App::updateSpinCamera() {
  float t = time_s_;
  float r = 2400 * cosf(t / 3.f) + 3000;
  spinCam_.pos = vec3{r * cosf(t), 800, r * sinf(t)};
  spinCam_.focus = vec3{0};
  spinCam_.up = vec3(0, 1, 0);
  frame_state_.scene.view =
      glm::lookAt(spinCam_.pos, spinCam_.focus, spinCam_.up);
}

void App::updateTrackballCamera() {
  if (input_.mouse.middle && input_.mouse.moved) {
    float focus_scale = 0.001f * trackball_.dist;
    vec3 move = {
        input_.mouse.xrel * focus_scale, -input_.mouse.yrel * focus_scale, 0};
    move = glm::inverse(frame_state_.scene.view) * vec4(move, 0);
    trackball_.focus += move;
  }

  if (input_.mouse.left && input_.mouse.moved) {
    ivec2 pos = {input_.mouse.x, input_.mouse.y};
    ivec2 prev = {
        input_.mouse.x - input_.mouse.xrel, input_.mouse.y - input_.mouse.yrel};

    vec3 start = getTrackballVec(prev);
    vec3 end = getTrackballVec(pos);
    auto rot = glm::rotation(start, end);
    trackball_.rot = rot * trackball_.rot;
  }

  updateDist(trackball_.dist);

  frame_state_.scene.view = glm::translate(vec3(0, 0, trackball_.dist)) *
                            glm::toMat4(trackball_.rot) *
                            glm::translate(trackball_.focus);
}

vec3 App::getTrackballVec(ivec2 mouse) {
  ivec2 ipos = {
      std::clamp(mouse.x, 0, (int)width_),
      std::clamp(mouse.y, 0, (int)height_)};
  vec2 pos = ipos;

  vec2 center = {width_ / 2.f, height_ / 2.f};
  pos -= center;

  float r = glm::length(center);
  pos /= r;
  pos.y *= -1;

  float length = std::min(glm::length(pos), 1.f);
  float posz = -sqrtf(1.f - length);
  return glm::normalize(vec3{pos, posz});
}

void App::updateFpsCamera() {
  updateYawPitch(fps_cam_.yaw, fps_cam_.pitch);

  mat4 rot = glm::eulerAngleXY(
      glm::radians(fps_cam_.pitch), glm::radians(fps_cam_.yaw));

  vec2 dir = getWasdDir(input_);
  if (abs(glm::length(dir)) > 0.01) {
    float move_scale = 3 * time_delta_ms_;
    fps_cam_.pos +=
        vec3(glm::inverse(rot) * move_scale * vec4(dir.x, 0, dir.y, 0));
  }

  frame_state_.scene.view = rot * glm::translate(-fps_cam_.pos);
}

void App::updateYawPitch(float& yaw, float& pitch) {
  constexpr float mouse_scale = -0.1;
  // TODO: Capture mouse so FPS cam doesn't require holding left click.
  if (input_.mouse.left && input_.mouse.moved) {
    yaw += mouse_scale * input_.mouse.xrel;
    pitch += mouse_scale * input_.mouse.yrel;
    pitch = glm::clamp(pitch, -90.f, 90.f);
  }
}

void App::updateDist(float& dist) {
  constexpr float zoom_frac = 0.9f;
  if (input_.mouse.scrollyrel == 1) {
    dist *= zoom_frac;
  } else if (input_.mouse.scrollyrel == -1) {
    dist /= zoom_frac;
  }
}

void App::updateFollowCamera() {
  follow_cam_.focus = getSkellyPos() + vec3(0, skelly_.getPelvisHeight(), 0);
  updateYawPitch(follow_cam_.yaw, follow_cam_.pitch);
  updateDist(follow_cam_.dist);

  mat4 rot = glm::eulerAngleXY(
      glm::radians(follow_cam_.pitch), glm::radians(follow_cam_.yaw));

  frame_state_.scene.view = glm::translate(vec3(0, 0, follow_cam_.dist)) * rot *
                            glm::translate(-follow_cam_.focus);
}

void App::handleInput() {
  if (options_.cam_type == CameraType::Follow) {
    input_.move.dir =
        glm::rotate(getWasdDir(input_), glm::radians(follow_cam_.yaw));
  }

  skelly_.handleInput(input_);
}

void App::updateProjectionMatrix() {
  constexpr float near = 20.f;
  frame_state_.scene.proj = perspectiveInfRevZ(
      glm::radians(45.0f), (float)width_ / (float)height_, near);
  frame_state_.scene.invProj = glm::inverse(frame_state_.scene.proj);

  frame_state_.scene.width = width_;
  frame_state_.scene.height = height_;
  frame_state_.scene.near = near;
  frame_state_.scene.far = std::numeric_limits<float>::infinity();
}

void App::flattenObjectTree() {
  Time start = Clock::now();

  world_.compress();
  world_.updateMats();

  Time end = Clock::now();
  stats_.flatten.addTime(FloatMs(end - start).count());
}

void App::updateDraws() {
  frame_state_.transforms = world_.drawMs_;
  frame_state_.draws2d = gDebugDraws.draws2d_;

  frame_state_.drawsNeedUpdate |= world_.drawsDirty_;
  if (!frame_state_.drawsNeedUpdate) {
    return;
  }
  std::println("updating draws frame:{}", frame_state_.frame_num);

  frame_state_.draws = world_.draws_;
  world_.drawsDirty_ = false;

  if (ui_.gooch) {
    for (auto& draw : frame_state_.draws) {
      if (draw.material != kMaterialIdNone) {
        draw.material = mats_.gooch;
      }
    }
  }
}

void App::updateImgui() {
  renderer_->imguiNewFrame();
  ImGui_ImplSDL2_NewFrame();
  ImGui::NewFrame();

  // ImGui::ShowDemoWindow();

  ImGui::Begin("Stats");
  ImGui::Text("%s", ui_.fps.c_str());
  ImGui::Text("%s", ui_.grid.c_str());
  ImGui::Text("%s", ui_.skelly.c_str());
  ImGui::Text("%s", ui_.flatten.c_str());
  bool show_pos = false;
  if (show_pos) {
    std::string pos_str = std::format("Pos: {}", toStr(getSkellyPos()));
    ImGui::Text(pos_str.c_str());
  }
  ImGui::End();

  ImGui::Begin("Misc Controls");

  ImGui::BeginTabBar("Misc Tabs");
  if (ImGui::BeginTabItem("General")) {
    SliderFloatDefault(
        "Time Scale", &options_.time_scale, 0.1, 2,
        default_options_.time_scale);
    ImGui::Separator();

    ImGui::SliderInt("Debug View", (int*)&frame_state_.debug_view, 0, 2);
    if (ImGui::Checkbox("Gooch", &ui_.gooch)) {
      frame_state_.drawsNeedUpdate = true;
    }
    if (ImGui::SliderInt("Floor", &ui_.floor, 0, (int)Floor::NumFloors - 1)) {
      world_.setMaterial(floor_, floor_mats_[ui_.floor]);
    }
    ImGui::Checkbox("Edges", &frame_state_.draw_edges);
    ImGui::SliderFloat(
        "Edge Width", &frame_state_.edge_w, 0, 1000, "%.2f",
        ImGuiSliderFlags_Logarithmic);
    ImGui::Checkbox("Debug Draws", &gDebugDraws.enabled_);
    ImGui::EndTabItem();
  }
  if (ImGui::BeginTabItem("Objects")) {
    ImGui::Checkbox("Animate", &options_.animate);
    if (ImGui::Checkbox("Show Controls", &options_.show_controls)) {
      skelly_.setMaterials(
          mats_.bot, options_.show_controls ? mats_.control : kMaterialIdNone);
    }
    if (ImGui::Checkbox("Bounce Objects", &options_.bounce_objects)) {
      grid_->setBounce(options_.bounce_objects);
    }
    if (ImGui::SliderInt("Grid Size", &options_.grid_size, 1, 200)) {
      // Make a new Grid here just to test Entity deletion.
      grid_ = std::make_unique<Grid>(&world_, mats_.cube, mats_.cube2);
      grid_->makeGrid(options_.grid_size);
    }

    ImGui::ColorEdit3(
        "BG Up", (float*)&frame_state_.scene.colorUp,
        ImGuiColorEditFlags_Float);
    ImGui::ColorEdit3(
        "BG Down", (float*)&frame_state_.scene.colorDown,
        ImGuiColorEditFlags_Float);

    if (ImGui::SliderFloat3(
            "Sun Dir", (float*)&frame_state_.scene.lights[0].vec, -1, 1)) {
      frame_state_.scene.lights[0].vec =
          vec4(glm::normalize(frame_state_.scene.lights[0].vec.xyz()), 0);
    }

    if (ImGui::SliderInt("Steps", &options_.tetra_steps, 0, 8)) {
      loadPrimitives();
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("In", &options_.tetra_in)) {
      loadPrimitives();
    }

    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("Materials")) {
    if (ImGuiMaterial(mats_.botData, "Bot")) {
      renderer_->updateMaterial(mats_.bot, mats_.botData);
    }
    if (ImGuiMaterial(mats_.cube_data, "Cube1")) {
      renderer_->updateMaterial(mats_.cube, mats_.cube_data);
    }
    if (ImGuiMaterial(mats_.gradient_floor_data, "Floor")) {
      renderer_->updateMaterial(
          mats_.gradient_floor, mats_.gradient_floor_data);
    }
    if (ImGuiMaterial(mats_.bone_data, "Bone")) {
      renderer_->updateMaterial(mats_.bone, mats_.bone_data);
    }
    if (ImGuiMaterial(mats_.gooch_data, "Gooch")) {
      renderer_->updateMaterial(mats_.gooch, mats_.gooch_data);
    }

    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("Edges")) {
    ImGuiDebugData(frame_state_.edges);
    ImGui::Checkbox("Stained Glass", &frame_state_.stained_glass);
    ImGui::SliderFloat(
        "Voronoi Tweak", &frame_state_.v_tweak, 0, 1000, "%.7f",
        ImGuiSliderFlags_Logarithmic);
    ImGui::EndTabItem();
  }
  if (ImGui::BeginTabItem("Drawing")) {
    frame_state_.update_drawing |= ImGuiDebugData(frame_state_.drawing);
    ImGui::EndTabItem();
  }
  if (ImGui::BeginTabItem("Camera")) {
    ImGui::Text("Camera Type:");
    int cam_ind = static_cast<int>(options_.cam_type);
    ImGui::RadioButton("Spin", &cam_ind, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Trackball", &cam_ind, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Fps", &cam_ind, 2);
    ImGui::SameLine();
    ImGui::RadioButton("Follow", &cam_ind, 3);
    CameraType cam_types[] = {
        CameraType::Spin,
        CameraType::Trackball,
        CameraType::Fps,
        CameraType::Follow,
    };
    options_.cam_type = cam_types[cam_ind];
    std::string cam_str = std::format("Follow Cam Dist: {}", follow_cam_.dist);
    ImGui::Text(cam_str.c_str());
    ImGui::EndTabItem();
  }
  ImGui::EndTabBar();
  ImGui::End();

  skelly_.UpdateImgui();

  ImGui::Render();
}

bool ImGuiMaterial(MaterialData& data, std::string label) {
  auto label1 = std::format("{}-1##2f", label);
  auto label2 = std::format("{}-2##2f", label);
  auto label3 = std::format("{}-3##2f", label);
  auto label4 = std::format("{}-4##2f", label);
  auto typeLabel = std::format("{}-Type", label);

  const char* types[] = {"Phong", "Gooch", "Toon"};
  bool update = ImGui::Combo(
      typeLabel.c_str(), (int*)(&data.type), types, IM_ARRAYSIZE(types));
  update |= ImGui::ColorEdit3(
      label1.c_str(), (float*)&data.color1, ImGuiColorEditFlags_Float);
  update |= ImGui::ColorEdit3(
      label2.c_str(), (float*)&data.color2, ImGuiColorEditFlags_Float);
  update |= ImGui::ColorEdit3(
      label3.c_str(), (float*)&data.color3, ImGuiColorEditFlags_Float);
  update |= ImGui::ColorEdit3(
      label4.c_str(), (float*)&data.color4, ImGuiColorEditFlags_Float);

  return update;
}

bool App::ImGuiDebugData(DebugData& debug) {
  bool changed = false;
  changed |= ImGui::SliderInt("i1", (int*)&debug.i1.i, 0, 10);
  changed |= ImGui::SliderInt("i2", (int*)&debug.i2.i, 0, 4);
  changed |= ImGui::SliderInt("i3", (int*)&debug.i3.i, 0, 4);
  changed |= ImGui::Checkbox("i4.b1", &debug.i4.b.b1);
  ImGui::SameLine();
  changed |= ImGui::Checkbox("i4.b2", &debug.i4.b.b2);
  ImGui::SameLine();
  changed |= ImGui::Checkbox("i4.b3", &debug.i4.b.b3);
  ImGui::SameLine();
  changed |= ImGui::Checkbox("i4.b4", &debug.i4.b.b4);
  changed |= ImGui::SliderFloat("f1", &debug.f1, 0, 4);
  changed |= ImGui::SliderFloat("f2", &debug.f2, 0, 3) &&
             (Floor)ui_.floor == Floor::Drawing;
  changed |= ImGui::SliderFloat(
      "f3", &debug.f3, 0, 10000, "%.5f", ImGuiSliderFlags_Logarithmic);
  changed |= ImGui::SliderFloat("f4", &debug.f4, 0, 90);
  return changed;
}

void App::cleanup() {
  renderer_.reset();

  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_DestroyWindow(window_);
  SDL_Quit();
}

std::map<ModelId, ModelInfo> kModelRegistry = {
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
