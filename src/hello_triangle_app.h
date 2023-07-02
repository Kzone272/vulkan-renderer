#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_image.h>
#include <SDL_vulkan.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>

#include "asserts.h"
#include "defines.h"
#include "frame-state.h"
#include "glm-include.h"
#include "renderer.h"
#include "vulkan-include.h"
#include "world.h"

using std::cerr;
using std::cout;
using std::endl;
using std::printf;

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Time = std::chrono::time_point<Clock>;

namespace {

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;

}  // namespace

using glm::mat4;
using glm::vec2;
using glm::vec3;

class HelloTriangleApp {
 public:
  void run() {
    initWindow();
    renderer_ = std::make_unique<Renderer>(window_, width_, height_);
    frame_state_.world = &world_;
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
      cerr << error << endl;
      ASSERT(false);
    }
    SDL_AddEventWatch(SdlEventWatcher, this);
  }

  void setupWorld() {
    const int grid = 20;
    for (int i = 0; i < grid; i++) {
      for (int j = 0; j < grid; j++) {
        if ((i + j) % 2 == 0) {
          auto obj = std::make_unique<Object>(ModelId::VIKING);
          obj->setPos(vec3(i - grid / 2, 0, j - grid / 2));
          obj->setScale(vec3{0.5f});
          addObject(std::move(obj));
        } else {
          auto obj = std::make_unique<Object>(ModelId::PONY);
          obj->setPos(vec3(i - grid / 2, 0, j - grid / 2));
          obj->setScale(vec3{0.001f});
          addObject(std::move(obj));
        }
      }
    }
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
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_WINDOWEVENT) {
        if (event.window.event == SDL_WINDOWEVENT_CLOSE) {
          quit_ = true;
          break;
        } else if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
          window_resized_ = true;
        } else if (event.window.event == SDL_WINDOWEVENT_MINIMIZED) {
          window_minimized_ = true;
        } else if (event.window.event == SDL_WINDOWEVENT_RESTORED) {
          window_minimized_ = false;
        }
      }
    }
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
    timeTick();
    animate();

    if (window_resized_) {
      updateWindowSize();
      renderer_->resizeWindow(width_, height_);
      window_resized_ = false;
    }
    if (!window_minimized_ && !window_empty_) {
      renderer_->drawFrame(&frame_state_);
      frame_state_.frame_num++;
    }
  }

  using float_ms = std::chrono::duration<float, std::ratio<1, 1000>>;
  using float_s = std::chrono::duration<float, std::chrono::seconds::period>;

  void timeTick() {
    Time now = Clock::now();
    if (frame_state_.frame_num == 0) {
      start_ = now;
      last_frame_time_ = now;
    }

    time_s_ = float_s(now - start_).count();

    auto time_delta = now - last_frame_time_;
    time_delta_ms_ = float_ms(time_delta).count();
    frame_times_.push_back(time_delta_ms_);

    last_frame_time_ = now;

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
      printf("%.2fms avg frame (%d fps)\n", avg_time, fps);

      next_fps_time_ = now + 1s;
      last_fps_frame_ = frame_state_.frame_num;
    }
  }

  void animate() {
    frame_state_.anim.clear_val = updateClearValue();
    frame_state_.anim.model_rot = updateModelRotation();

    updateObjects();
    updateUniformBuffer();
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
    auto spin = glm::angleAxis(
        glm::radians(frame_state_.anim.model_rot), vec3(0.f, 1.f, 0.f));
    auto orient_spin =
        spin * glm::angleAxis(glm::radians(-90.f), vec3(1, 0, 0));

    for (auto& object : world_.objects) {
      vec3 pos = object->getPos();
      pos.y = 0.f;

      float dist = glm::length(pos);
      float t = dist / 2.f + 4 * time_s_;
      float height = sinf(t);
      pos.y = height;
      object->setPos(pos);

      // Viking model is off by 90deg.
      auto quat = object->getModel() == ModelId::VIKING ? orient_spin : spin;
      object->setRot(glm::angle(quat), glm::axis(quat));
    }
  }

  void updateUniformBuffer() {
    Camera c;
    float t = time_s_;
    float r = 8 * cosf(t / 3.f) + 10;
    float t2 = time_s_;
    c.pos = vec3{r * cosf(t2), cosf(t) + 2.5, r * sinf(t2)};
    c.focus = vec3{0};
    c.up = vec3(0, 1, 0);
    frame_state_.view = glm::lookAt(c.pos, c.focus, c.up);
    frame_state_.proj = glm::perspective(
        glm::radians(45.0f), (float)width_ / (float)height_, 0.1f, 100.0f);
    // Invert y-axis because Vulkan is opposite GL.
    frame_state_.proj[1][1] *= -1;
  }

  void cleanup() {
    renderer_->cleanup();
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
  Time last_frame_time_;
  // Time in ms since the last draw, as a float
  float time_delta_ms_ = 0.0f;
  // Used to calculate FPS
  uint64_t last_fps_frame_ = 0;
  Time next_fps_time_;
  // Last 1s of frame times.
  std::vector<float> frame_times_;

  FrameState frame_state_;
  std::unique_ptr<Renderer> renderer_;
  World world_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
