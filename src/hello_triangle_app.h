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
#include "defines.h"
#include "frame-state.h"
#include "glm-include.h"
#include "input.h"
#include "primitives.h"
#include "renderer.h"
#include "time-include.h"
#include "utils.h"
#include "vulkan-include.h"
#include "world.h"

namespace {

constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;

}  // namespace

class HelloTriangleApp {
 public:
  void run() {
    initWindow();
    renderer_ = std::make_unique<Renderer>(window_, width_, height_);

    initFrameState();
    initImgui();
    renderer_->init(&frame_state_);

    setupWorld();
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
    frame_state_.world = &world_;
    updateProjectionMatrix();

    cam_ = {
        .pos{0, 300, -300 * options_.grid_size}, .focus{0, 0, 0}, .up{0, 1, 0}};
    trackball_ = {
        .dist = 300.f * static_cast<float>(options_.grid_size),
        .focus{0, 0, 0},
        .rot{1, 0, 0, 0}};
    fps_cam_.pos = {0, 300, -100 * options_.grid_size / 2};
    updateCamera();
  }

  void setupWorld() {
    loadPrimitives();
    remakeGrid(options_.grid_size);
  }

  void loadPrimitives() {
    renderer_->useMesh(ModelId::CUBE, makeCube({0, 0.8, 0.8}));
    renderer_->useMesh(ModelId::MOVER, makeCube({0.9, 0.2, 0.1}));
    renderer_->useMesh(
        ModelId::TETRA, tetrahedron(options_.tetra_steps, options_.tetra_in));
  }

  void remakeGrid(int grid) {
    world_.objects.clear();

    for (int i = 0; i < grid; i++) {
      for (int j = 0; j < grid; j++) {
        ModelId id = ((i + j) % 2 == 0) ? ModelId::CUBE : ModelId::TETRA;
        auto obj = std::make_unique<Object>(id);
        if (id == ModelId::TETRA) {
          obj->setScale(vec3(100));
        }
        obj->setPos(500.f * vec3(i - grid / 2, 0, j - grid / 2));
        addObject(std::move(obj));
      }
    }

    makeMyObject();
  }

  // Make a placeholder object as something to animate.
  void makeMyObject() {
    auto obj = std::make_unique<Object>(ModelId::MOVER);
    obj->setPos(vec3(0, 300, 0));
    my_obj_ = obj.get();
    addObject(std::move(obj));
  }

  void addObject(std::unique_ptr<Object> obj) {
    renderer_->useModel(obj->getModel());
    world_.objects.push_back(std::move(obj));
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
      updateImgui();
      updateCamera();
      handleInput();
      if (options_.animate) {
        animate();
      }
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

  void animate() {
    frame_state_.anim.clear_val = updateClearValue();
    frame_state_.anim.model_rot = updateModelRotation();

    updateObjects();
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
    for (auto& object : world_.objects) {
      // For now, my_obj_ gets special handling.
      if (object.get() == my_obj_) {
        continue;
      }

      vec3 pos = object->getPos();
      pos.y = 0.f;

      if (options_.bounce_objects) {
        float dist = glm::length(pos);
        float t = dist / 600.f + 4 * time_s_;
        float height = sinf(t) * 25.f;
        pos.y = height;
      }
      object->setPos(pos);

      auto spin = glm::angleAxis(
          glm::radians(frame_state_.anim.model_rot), vec3(0.f, 1.f, 0.f));
      object->setRot(spin);
    }

    my_obj_->update(frame_time_);
  }

  void updateCamera() {
    if (options_.cam_type == CameraType::Spin) {
      updateSpinCamera();
    } else if (options_.cam_type == CameraType::Trackball) {
      updateTrackballCamera();
    } else if (options_.cam_type == CameraType::Fps) {
      updateFpsCamera();
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

    constexpr float zoom_frac = 0.9f;
    if (input_.mouse.scrollyrel == 1) {
      trackball_.dist *= zoom_frac;
    } else if (input_.mouse.scrollyrel == -1) {
      trackball_.dist /= zoom_frac;
    }

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
    // TODO: Capture mouse so FPS cam doesn't require holding left click.
    if (input_.mouse.left && input_.mouse.moved) {
      float mouse_scale = -0.1;
      fps_cam_.yaw += mouse_scale * input_.mouse.xrel;
      fps_cam_.pitch += mouse_scale * input_.mouse.yrel;
      fps_cam_.pitch = glm::clamp(fps_cam_.pitch, -90.f, 90.f);
    }

    mat4 rot = glm::eulerAngleXY(
        glm::radians(fps_cam_.pitch), glm::radians(fps_cam_.yaw));

    vec2 dir = getWasdDir();
    if (abs(glm::length(dir)) > 0.01) {
      float move_scale = 5 * time_delta_ms_;
      fps_cam_.pos +=
          vec3(glm::inverse(rot) * move_scale * vec4(dir.x, 0, dir.y, 0));
    }

    frame_state_.view = rot * glm::translate(mat4(1), -fps_cam_.pos);
  }

  vec2 getWasdDir() {
    vec2 dir{0};
    if (input_.kb.down.contains('w')) {
      dir += vec2(0, 1);
    }
    if (input_.kb.down.contains('s')) {
      dir += vec2(0, -1);
    }
    if (input_.kb.down.contains('a')) {
      dir += vec2(-1, 0);
    }
    if (input_.kb.down.contains('d')) {
      dir += vec2(1, 0);
    }
    return glm::normalize(dir);
  }

  void handleInput() {
    if (input_.kb.pressed.contains(' ')) {
      float z = my_obj_->getPos().z;
      float speed = 200;
      if (input_.kb.down.contains(Keys::Shift)) {
        speed *= -1;
      }
      float to_z = z + speed;
      my_obj_->animZ(makeAnimation(z, to_z, 500, frame_time_));
    }
  }

  void updateProjectionMatrix() {
    frame_state_.proj = glm::perspective(
        glm::radians(45.0f), (float)width_ / (float)height_, 0.1f, 100000.0f);
    // Invert y-axis because Vulkan is opposite GL.
    frame_state_.proj[1][1] *= -1;
  }

  void updateImgui() {
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Controls");
    ImGui::Text("%s", ui_.fps.c_str());

    ImGui::Text("Camera Type:");
    int cam_ind = static_cast<int>(options_.cam_type);
    ImGui::RadioButton("Spin", &cam_ind, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Trackball", &cam_ind, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Fps", &cam_ind, 2);
    CameraType cam_types[] = {
        CameraType::Spin,
        CameraType::Trackball,
        CameraType::Fps,
    };
    options_.cam_type = cam_types[cam_ind];

    ImGui::Checkbox("Animate", &options_.animate);
    ImGui::Checkbox("Bounce Objects", &options_.bounce_objects);
    if (ImGui::SliderInt("Grid Size", &options_.grid_size, 1, 50)) {
      remakeGrid(options_.grid_size);
    }

    if (ImGui::Checkbox("In", &options_.tetra_in)) {
      loadPrimitives();
    }
    ImGui::SameLine();
    if (ImGui::SliderInt("Steps", &options_.tetra_steps, 0, 8)) {
      loadPrimitives();
    }

    ImGui::End();

    ImGui::Render();
  }

  void cleanup() {
    renderer_->cleanup();

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    renderer_.reset();
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
  // Time in ms since the last draw, as a float
  float time_delta_ms_ = 0.0f;
  // Used to calculate FPS
  uint64_t last_fps_frame_ = 0;
  Time next_fps_time_;
  // Last 1s of frame times.
  std::vector<float> frame_times_;

  struct Options {
    CameraType cam_type = CameraType::Trackball;
    bool animate = true;
    bool bounce_objects = false;
    int grid_size = 2;
    bool tetra_in = true;
    int tetra_steps = 4;
  } options_;

  struct UiState {
    std::string fps;
  } ui_;

  Camera cam_;
  Trackball trackball_;
  FpsCamera fps_cam_;

  InputState input_;
  FrameState frame_state_;
  std::unique_ptr<Renderer> renderer_;
  World world_;
  Object* my_obj_ = nullptr;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
