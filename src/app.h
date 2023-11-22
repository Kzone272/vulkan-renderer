#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_image.h>
#include <SDL_vulkan.h>
#include <imgui/backends/imgui_impl_sdl2.h>
#include <imgui/imgui.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>

#include "animation.h"
#include "asserts.h"
#include "assets.h"
#include "defines.h"
#include "glm-include.h"
#include "input.h"
#include "maths.h"
#include "object.h"
#include "primitives.h"
#include "renderer.h"
#include "skelly.h"
#include "time-include.h"
#include "vec-maths.h"

namespace {

constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;

}  // namespace

class HelloTriangleApp {
 public:
  void run() {
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
  static int SdlEventWatcher(void* data, SDL_Event* event) {
    if (event->type == SDL_WINDOWEVENT) {
      if (event->window.event == SDL_WINDOWEVENT_RESIZED) {
        reinterpret_cast<HelloTriangleApp*>(data)->WindowResized();
      } else if (event->window.event == SDL_WINDOWEVENT_MOVED) {
        reinterpret_cast<HelloTriangleApp*>(data)->WindowMoved();
      }
    }
    return 0;
  }

  void WindowResized() {
    window_resized_ = true;
    update();
  }

  void WindowMoved() {
    update();
  }

 private:
  void initWindow() {
    ASSERT(SDL_Init(SDL_INIT_VIDEO) == 0);

    uint32_t window_flags = SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE;
    window_ = SDL_CreateWindow(
        "Vulkan Tutorial", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, window_flags);
    if (!window_) {
      std::string error{SDL_GetError()};
      printf("%s\n", error.c_str());
      ASSERT(false);
    }
    SDL_AddEventWatch(SdlEventWatcher, this);
  }

  void initImgui() {
    ImGui::CreateContext();
    ImGui_ImplSDL2_InitForVulkan(window_);
  }

  void initFrameState() {
    updateProjectionMatrix();

    cam_ = {
        .pos{0, 300, -300 * options_.grid_size}, .focus{0, 0, 0}, .up{0, 1, 0}};
    trackball_ = {
        .dist = 300.f * static_cast<float>(options_.grid_size),
        .focus{0, 0, 0},
        .rot{1, 0, 0, 0}};
    fps_cam_.pos = {0, 300, -100 * options_.grid_size / 2};
    follow_cam_ = {
        .dist = 800.f,
        .focus = skelly_.getPos(),
        .yaw = 0,
        .pitch = -30,
    };
    updateCamera();
  }

  void setupWorld() {
    world_.addChild(skelly_.getObj());
    world_.addChild(&grid_);
    remakeGrid(options_.grid_size);
    world_.addChild(std::make_unique<Object>(ModelId::Floor));

    auto* car = world_.addChild(std::make_unique<Object>(ModelId::Pony));
    car->setPos(vec3(-300, -1, 0));

    auto* room = world_.addChild(std::make_unique<Object>(ModelId::Viking));
    room->setPos(vec3(300, 1, 300));

    loadMaterials();
    loadModels();
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

  void loadMaterials() {
    auto cube_mat = renderer_->useMaterial({.data{.color1{0, 0.8, 0.8}}});
    auto bone_mat = renderer_->useMaterial({.data{.color1{0.9, 0.2, 0.1}}});
    auto control_mat = renderer_->useMaterial({.data{.color1{0.1, 1, 0.2}}});
    auto viking_mat = renderer_->useMaterial({
        .diffuse_path = "assets/textures/viking_room.png",
        .data{.color1{0.2, 0.2, 0.2}},
    });
    auto drawing_mat = renderer_->useMaterial({
        .diffuse_texture = renderer_->getDrawingTexture(),
        .data{.color1{0.2, 0.2, 0.2}},
    });
    auto voronoi_mat = renderer_->useMaterial({
        .diffuse_texture = renderer_->getVoronoiTexture(),
        .data{.color1{0.2, 0.2, 0.2}},
    });
    floor_mats_ = {viking_mat, drawing_mat, voronoi_mat};
    gooch_mat_ = renderer_->useMaterial({.data{
        .type = MaterialData::Type::Gooch,
        .color1{fromHex(0xff8d83)},
        .color2{fromHex(0x8e9ce2)},
    }});

    default_mats_.insert({ModelId::Cube, cube_mat});
    default_mats_.insert({ModelId::Bone, bone_mat});
    default_mats_.insert({ModelId::Control, control_mat});
    default_mats_.insert({ModelId::Floor, floor_mats_[ui_.floor]});
    default_mats_.insert({ModelId::Tetra, cube_mat});
  }

  void loadModels() {
    auto models = world_.getModels();
    std::set<ModelId> model_set(models.begin(), models.end());
    for (auto id : model_set) {
      if (id == ModelId::None) {
        continue;
      }
      auto it = model_registry.find(id);
      if (it == model_registry.end()) {
        continue;
      }
      auto& info = it->second;
      auto mat_id = renderer_->useMaterial({.diffuse_path = info.texture_path});
      Mesh mesh = loadObj(info.obj_path);
      renderer_->useMesh(id, mesh, mat_id);
      default_mats_.insert({id, mat_id});
    }
  }

  void useMesh(ModelId model, const Mesh& mesh) {
    renderer_->useMesh(model, mesh, default_mats_[model]);
  }

  void loadPrimitives() {
    auto cube = makeCube();
    useMesh(ModelId::Cube, cube);
    useMesh(ModelId::Bone, cube);
    useMesh(ModelId::Control, cube);
    useMesh(ModelId::Floor, makePlane(10000, 10000));
    useMesh(
        ModelId::Tetra, tetrahedron(options_.tetra_steps, options_.tetra_in));
  }

  void setGooch(bool on) {
    std::vector<ModelId> gooch_models = {
        ModelId::Cube,  ModelId::Bone, ModelId::Control,
        ModelId::Tetra, ModelId::Pony, ModelId::Viking,
    };
    for (auto& model : gooch_models) {
      renderer_->setModelMaterial(
          model, on ? gooch_mat_ : default_mats_[model]);
    }
  }

  void remakeGrid(int grid) {
    grid_.clearChildren();

    for (int i = 0; i < grid; i++) {
      for (int j = 0; j < grid; j++) {
        ModelId id = ((i + j) % 2 == 0) ? ModelId::Cube : ModelId::Tetra;
        mat4 model_t = glm::scale(vec3(100));
        auto obj = std::make_unique<Object>(id, model_t);
        obj->setPos(500.f * vec3(i - grid / 2, 0, j - grid / 2));
        grid_.addChild(std::move(obj));
      }
    }
  }

  void setupLights() {
    frame_state_.lights.clear();
    // Mostly downward sun
    frame_state_.lights.push_back({
        .type = Light::Type::Directional,
        .vec = vec3(0.3, -1, 0),
        .color = vec3(1),
    });
    // Corner torch
    frame_state_.lights.push_back({
        .type = Light::Type::Point,
        .vec = vec3(385, 200, 475),
        .color = vec3(1, 1, 0),
        .falloff = 400,
    });
    // Front torch
    frame_state_.lights.push_back({
        .type = Light::Type::Point,
        .vec = vec3(385, 200, 110),
        .color = vec3(1, 1, 0),
        .falloff = 400,
    });
    // Cauldron
    frame_state_.lights.push_back({
        .type = Light::Type::Point,
        .vec = vec3(270, 20, 300),
        .color = vec3(2, 1.2, 0),
        .falloff = 500,
    });
    // Right tail light
    frame_state_.lights.push_back({
        .type = Light::Type::Point,
        .vec = vec3(-225, 40, -180),
        .color = vec3(4, 0, 0),
        .falloff = 500,
    });
    // Left tail light
    frame_state_.lights.push_back({
        .type = Light::Type::Point,
        .vec = vec3(-370, 40, -180),
        .color = vec3(4, 0, 0),
        .falloff = 500,
    });
  }

  void mainLoop() {
    while (true) {
      processEvents();
      if (quit_) {
        break;
      }
      update();
    }
  }

  void processEvents() {
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
  bool handleWindowEvent(const SDL_Event& event) {
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

  void updateWindowSize() {
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

  void update() {
    updateTime();

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
        animate();
      }
      updateCamera();

      flattenObjectTree();
      renderer_->drawFrame(&frame_state_);
      frame_state_.frame_num++;
    }
  }

  void updateTime() {
    Time now = Clock::now();
    if (frame_state_.frame_num == 0) {
      start_ = now;
      frame_time_ = now;
    }

    time_s_ = FloatS(now - start_).count();

    auto time_delta = now - frame_time_;
    time_delta_s_ = FloatS(time_delta).count();
    time_delta_ms_ = FloatMs(time_delta).count();
    frame_times_.push_back(time_delta_ms_);

    frame_time_ = now;

    checkFps(now);
  }

  void checkFps(Time now) {
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
      ui_.fps = strFmt("%.2fms avg frame (%d fps)\n", avg_time, fps);

      next_fps_time_ = now + 1s;
      last_fps_frame_ = frame_state_.frame_num;
    }
  }

  void resetFrameState() {
    frame_state_.update_drawing = frame_state_.frame_num == 0;
    frame_state_.update_voronoi = frame_state_.frame_num == 0;
  }

  void animate() {
    anim_.clear_val = updateClearValue();
    anim_.model_rot = updateModelRotation();
    updateObjects();

    if ((Floor)ui_.floor == Floor::Voronoi) {
      for (int i = 0; i < cell_centers_.size(); i++) {
        frame_state_.voronoi_cells[i].pos =
            cell_centers_[i] +
            glm::rotate(vec2(0, 0.1), time_s_ / glm::length(cell_centers_[i]));
      }
      frame_state_.update_voronoi = true;
    }
  }

  float updateClearValue() {
    constexpr float seq_dur_ms = 5000;
    static float seq = 0;
    seq += time_delta_ms_ / seq_dur_ms;
    while (seq > 1) {
      seq -= 1;
    }
    // return seq < 0.5f ? seq : (1.0f - seq); // linear bounce
    return cos(seq * 2.0f * M_PI) / 2.0f + 0.5f;
  }

  float updateModelRotation() {
    constexpr float seq_dur_ms = 4000;
    constexpr float total_rot = 360;
    static float seq = 0;
    seq += time_delta_ms_ / seq_dur_ms;
    while (seq > 1) {
      seq -= 1;
    }
    return seq * total_rot;
  }

  void updateObjects() {
    for (auto* object : grid_.children()) {
      vec3 pos = object->getPos();
      pos.y = 0.f;

      if (options_.bounce_objects) {
        float dist = glm::length(pos);
        float t = dist / 600.f + 4 * time_s_;
        float height = sinf(t) * 25.f;
        pos.y = height;
      }
      object->setPos(pos);

      auto spin =
          glm::angleAxis(glm::radians(anim_.model_rot), vec3(0.f, 1.f, 0.f));
      object->setRot(spin);
    }

    skelly_.update(frame_time_, time_delta_s_);
  }

  void updateCamera() {
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
  }

  void updateSpinCamera() {
    float t = time_s_;
    float r = 2400 * cosf(t / 3.f) + 3000;
    cam_.pos = vec3{r * cosf(t), 800, r * sinf(t)};
    cam_.focus = vec3{0};
    cam_.up = vec3(0, 1, 0);
    frame_state_.view = glm::lookAt(cam_.pos, cam_.focus, cam_.up);
  }

  void updateTrackballCamera() {
    if (input_.mouse.middle && input_.mouse.moved) {
      float focus_scale = 0.001f * trackball_.dist;
      vec3 move = {
          input_.mouse.xrel * focus_scale, -input_.mouse.yrel * focus_scale, 0};
      move = glm::inverse(frame_state_.view) * vec4(move, 0);
      trackball_.focus += move;
    }

    if (input_.mouse.left && input_.mouse.moved) {
      ivec2 pos = {input_.mouse.x, input_.mouse.y};
      ivec2 prev = {
          input_.mouse.x - input_.mouse.xrel,
          input_.mouse.y - input_.mouse.yrel};

      vec3 start = getTrackballVec(prev);
      vec3 end = getTrackballVec(pos);
      auto rot = glm::rotation(start, end);
      trackball_.rot = rot * trackball_.rot;
    }

    updateDist(trackball_.dist);

    frame_state_.view = glm::translate(mat4(1), vec3(0, 0, trackball_.dist)) *
                        glm::toMat4(trackball_.rot) *
                        glm::translate(mat4(1), trackball_.focus);
  }

  vec3 getTrackballVec(ivec2 mouse) {
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

  void updateFpsCamera() {
    updateYawPitch(fps_cam_.yaw, fps_cam_.pitch);

    mat4 rot = glm::eulerAngleXY(
        glm::radians(fps_cam_.pitch), glm::radians(fps_cam_.yaw));

    vec2 dir = getWasdDir(input_);
    if (abs(glm::length(dir)) > 0.01) {
      float move_scale = 3 * time_delta_ms_;
      fps_cam_.pos +=
          vec3(glm::inverse(rot) * move_scale * vec4(dir.x, 0, dir.y, 0));
    }

    frame_state_.view = rot * glm::translate(mat4(1), -fps_cam_.pos);
  }

  void updateYawPitch(float& yaw, float& pitch) {
    constexpr float mouse_scale = -0.1;
    // TODO: Capture mouse so FPS cam doesn't require holding left click.
    if (input_.mouse.left && input_.mouse.moved) {
      yaw += mouse_scale * input_.mouse.xrel;
      pitch += mouse_scale * input_.mouse.yrel;
      pitch = glm::clamp(pitch, -90.f, 90.f);
    }
  }

  void updateDist(float& dist) {
    constexpr float zoom_frac = 0.9f;
    if (input_.mouse.scrollyrel == 1) {
      dist *= zoom_frac;
    } else if (input_.mouse.scrollyrel == -1) {
      dist /= zoom_frac;
    }
  }

  void updateFollowCamera() {
    follow_cam_.focus =
        skelly_.getPos() + vec3(0, skelly_.getSkellySizes()->leg, 0);
    updateYawPitch(follow_cam_.yaw, follow_cam_.pitch);
    updateDist(follow_cam_.dist);

    mat4 rot = glm::eulerAngleXY(
        glm::radians(follow_cam_.pitch), glm::radians(follow_cam_.yaw));

    frame_state_.view = glm::translate(mat4(1), vec3(0, 0, follow_cam_.dist)) *
                        rot * glm::translate(mat4(1), -follow_cam_.focus);
  }

  void handleInput() {
    if (options_.cam_type == CameraType::Follow) {
      input_.move.dir =
          glm::rotate(getWasdDir(input_), glm::radians(follow_cam_.yaw));
    }

    skelly_.handleInput(input_, frame_time_);
  }

  void updateProjectionMatrix() {
    frame_state_.proj = glm::perspective(
        glm::radians(45.0f), (float)width_ / (float)height_, 5.f, 50000.0f);
    // Invert y-axis because Vulkan is opposite GL.
    frame_state_.proj[1][1] *= -1;
  }

  void flattenObjectTree() {
    frame_state_.objects.clear();
    mat4 identity(1);
    world_.getSceneObjects(identity, frame_state_.objects);
  }

  void updateImgui() {
    renderer_->imguiNewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Stats");
    ImGui::Text("%s", ui_.fps.c_str());
    std::string pos_str = "Pos: " + tostr(skelly_.getPos());
    ImGui::Text(pos_str.c_str());
    ImGui::End();

    ImGui::Begin("Misc Controls");

    ImGui::BeginTabBar("Misc Tabs");
    if (ImGui::BeginTabItem("General")) {
      if (ImGui::Checkbox("Gooch", &ui_.gooch)) {
        setGooch(ui_.gooch);
      }
      if (ImGui::SliderInt("Floor", &ui_.floor, 0, (int)Floor::NumFloors - 1)) {
        MaterialId floor = floor_mats_[ui_.floor];
        renderer_->setModelMaterial(ModelId::Floor, floor);
      }
      ImGui::Checkbox("Edges", &frame_state_.draw_edges);
      ImGui::SliderInt("Debug View", (int*)&frame_state_.debug_view, 0, 2);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Edges")) {
      ImGuiDebugData(frame_state_.edges);
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
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Objects")) {
      ImGui::Checkbox("Animate", &options_.animate);
      ImGui::Checkbox("Bounce Objects", &options_.bounce_objects);
      if (ImGui::SliderInt("Grid Size", &options_.grid_size, 1, 50)) {
        remakeGrid(options_.grid_size);
      }

      if (ImGui::Checkbox("In", &options_.tetra_in)) {
        loadPrimitives();
      }
      if (ImGui::SliderInt("Steps", &options_.tetra_steps, 0, 8)) {
        loadPrimitives();
      }
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::End();

    ImGui::Begin("Skeleton");

    MoveOptions& move = *skelly_.getMoveOptions();
    ImGui::SliderFloat("Max Speed", &move.max_speed, 0, 500);
    ImGui::Separator();

    ImGui::BeginTabBar("Skeleton Tabs");
    if (ImGui::BeginTabItem("Movement")) {
      if (ImGui::Combo(
              "Movement Presets", &move_preset_,
              "Normal\0Tightrope\0Preppy\0Snow\0Runway\0Crouch\0Flanders")) {
        if (move_preset_ == 0) {
          moveDefault(move);
        } else if (move_preset_ == 1) {
          moveTightrope(move);
        } else if (move_preset_ == 2) {
          movePreppy(move);
        } else if (move_preset_ == 3) {
          moveSnow(move);
        } else if (move_preset_ == 4) {
          moveRunway(move);
        } else if (move_preset_ == 5) {
          moveCrouch(move);
        } else if (move_preset_ == 6) {
          moveFlanders(move);
        }
        skelly_.makeBones();
      }

      ImGui::SliderFloat("Vel Adjust Time", &move.adjust_time, 0, 1000);
      ImGui::SliderFloat("Max Rot Speed", &move.max_rot_speed, 1, 360);
      ImGui::SliderFloat("Crouch %", &move.crouch_pct, 0.01, 1.2);
      if (ImGui::SliderFloat("Stance W", &move.stance_w, 0, 60)) {
        skelly_.makeBones();
      }
      ImGui::SliderFloat("Step Height", &move.step_height, 0, 50);
      ImGui::SliderFloat("Lean", &move.lean, -0.2, 0.2);
      ImGui::SliderFloat("Bounce", &move.bounce, 0, 10);
      ImGui::SliderFloat("Hip Sway", &move.hip_sway, 0, 30);
      ImGui::SliderFloat("Hip Spin", &move.hip_spin, 0, 45);
      ImGui::SliderFloat("Heel Lift %", &move.heel_lift_pct, 0, 2);
      ImGui::SliderFloat("Heels Shift", &move.heel_shift, 0, 90);
      ImGui::SliderFloat("Shoulder Spin", &move.shoulder_spin, 0, 45);
      ImGui::SliderFloat("Arm Span %", &move.arm_span_pct, -0.2, 1);
      ImGui::SliderFloat("Hand Height %", &move.hand_height_pct, -1, 1);
      ImGui::SliderFloat("Hands Forward", &move.hands_forward, -50, 50);

      ImGui::EndTabItem();
    }

    if (ImGui::BeginTabItem("Sizes")) {
      SkellySizes& sizes = *skelly_.getSkellySizes();
      if (ImGui::Combo(
              "Size Presets", &size_preset_,
              "Default\0Tall\0Big\0Dwarf\0Chimp")) {
        if (size_preset_ == 0) {
          sizeDefault(sizes);
        } else if (size_preset_ == 1) {
          sizeTall(sizes);
        } else if (size_preset_ == 2) {
          sizeBig(sizes);
        } else if (size_preset_ == 3) {
          sizeDwarf(sizes);
        } else if (size_preset_ == 4) {
          sizeChimp(sizes);
        }
        skelly_.makeBones();
      }
      ImGui::Separator();

      bool changed = false;
      changed |= ImGui::SliderFloat("Height", &sizes.height, 1, 250);
      changed |= ImGui::SliderFloat("Leg", &sizes.leg, 1, sizes.height);
      changed |= ImGui::SliderFloat("Femur", &sizes.femur_pct, 0.05, 1);
      changed |= ImGui::SliderFloat("Bone W", &sizes.bone_w, 0.5, 10);
      changed |= ImGui::SliderFloat("Pelvis W", &sizes.pelvis_w, 1, 60);
      changed |= ImGui::SliderFloat("Shoudlers W", &sizes.shoulders_w, 1, 100);
      changed |= ImGui::SliderFloat("Arm", &sizes.arm, 1, 150);
      changed |= ImGui::SliderFloat("Bicep %", &sizes.bicep_pct, 0.05, 1);
      if (changed) {
        skelly_.makeBones();
      }

      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::End();

    ImGui::Render();
  }

  bool ImGuiDebugData(DebugData& debug) {
    bool changed = false;
    changed |= ImGui::SliderInt("i1", (int*)&debug.i1.i, 0, 4);
    changed |= ImGui::SliderInt("i2", (int*)&debug.i2.i, 0, 4);
    changed |= ImGui::SliderInt("i3", (int*)&debug.i3.i, 0, 4);
    changed |= ImGui::Checkbox("i4.b1", &debug.i4.b.b1);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("i4.b2", &debug.i4.b.b2);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("i4.b3", &debug.i4.b.b3);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("i4.b4", &debug.i4.b.b4);
    changed |= ImGui::SliderFloat("f1", &debug.f1, 0, 10);
    changed |= ImGui::SliderFloat("f2", &debug.f2, 0, 1) &&
               (Floor)ui_.floor == Floor::Drawing;
    changed |= ImGui::SliderFloat(
        "f3", &debug.f3, 0, 10000, "%.5f", ImGuiSliderFlags_Logarithmic);
    changed |= ImGui::SliderFloat("f4", &debug.f4, 0, 90);
    return changed;
  }

  void cleanup() {
    renderer_.reset();

    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyWindow(window_);
    SDL_Quit();
  }

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
    bool bounce_objects = false;
    int grid_size = 2;
    bool tetra_in = true;
    int tetra_steps = 2;
  } options_;

  enum class Floor {
    Viking,
    Drawing,
    Voronoi,
    NumFloors,
  };
  struct UiState {
    std::string fps;
    bool gooch = false;
    int floor = (int)Floor::Viking;
  } ui_;

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
  Object world_;
  Object grid_;
  Skelly skelly_;
  int move_preset_ = 0;
  int size_preset_ = 0;

  std::map<ModelId, MaterialId> default_mats_;
  MaterialId gooch_mat_;
  std::vector<MaterialId> floor_mats_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
