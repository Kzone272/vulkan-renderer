#pragma once

#include <memory>
#include <span>
#include <vector>

#include "buffers.h"
#include "descriptors.h"
#include "fbo.h"
#include "images.h"
#include "pass.h"
#include "pipelines.h"
#include "render-objects.h"
#include "scene-data.h"
#include "vulkan-include.h"

struct SDL_Window;
struct SDL_Surface;

class Renderer {
 public:
  Renderer(SDL_Window* window, uint32_t width, uint32_t height) {
    window_ = window;
    width_ = width;
    height_ = height;
  }
  ~Renderer();

  void init(FrameState* frame_state);
  void drawFrame(FrameState* frame_state);
  void resizeWindow(uint32_t width, uint32_t height);
  MaterialId useMaterial(const MaterialInfo& mat_info);
  void useMesh(ModelId model_id, const Mesh& mesh, MaterialId mat_id);
  void setModelMaterial(ModelId model_id, MaterialId material_id);
  // TODO: Return a TextureId instead.
  Texture* getDrawingTexture();
  Texture* getVoronoiTexture();
  void imguiNewFrame();

 private:
  struct QueueFamilyIndices {
    uint32_t gfx_family = -1;
    uint32_t present_family = -1;

    bool isComplete() {
      return gfx_family != -1 && present_family != -1;
    }
  };
  struct SwapchainSupportDetails {
    vk::SurfaceCapabilitiesKHR caps;
    std::vector<vk::SurfaceFormatKHR> formats;
    std::vector<vk::PresentModeKHR> present_modes;
  };

  void initVulkan();
  void initImgui();
  void drawFrame();
  void createInstance();
  std::vector<const char*> getRequiredExtensions();
  void printSupportedExtensions();
  std::vector<const char*> getValidationLayers();
  vk::DebugUtilsMessengerCreateInfoEXT makeDbgMessengerCi();
  void setupDebugMessenger();
  void createSurface();
  void pickPhysicalDevice();
  vk::SampleCountFlagBits getMaxSampleCount();
  QueueFamilyIndices findQueueFamilies(vk::PhysicalDevice device);
  bool checkDeviceExtensionSupport(vk::PhysicalDevice device);
  SwapchainSupportDetails querySwapchainSupport(vk::PhysicalDevice device);
  void createLogicalDevice();
  vk::SurfaceFormatKHR chooseSwapSurfaceFormat(
      const std::vector<vk::SurfaceFormatKHR>& formats);
  vk::PresentModeKHR chooseSwapPresentMode(
      const std::vector<vk::PresentModeKHR>& present_modes);
  vk::Extent2D chooseSwapExtent(const vk::SurfaceCapabilitiesKHR& caps);
  void createSwapchain();

  void createCommandPool();
  void createCommandBuffers();

  void findDepthFormat();
  bool hasStencilComponent(vk::Format format);
  vk::Format findSupportedFormat(
      const std::vector<vk::Format>& formats, vk::ImageTiling tiling,
      vk::FormatFeatureFlags features);

  vk::UniqueShaderModule createShaderModule(std::string filename);
  void createShaders();

  void createSamplers();
  void initSdlImage();
  SDL_Surface* loadImage(std::string texture_path);
  Texture* createTexture(void* texture_data, uint32_t width, uint32_t height);
  void transitionImageLayout(
      vk::Image img, vk::Format format, uint32_t mip_levels,
      vk::ImageLayout old_layout, vk::ImageLayout new_layout);
  void copyBufferToImage(
      vk::Buffer buf, vk::Image img, uint32_t width, uint32_t height);
  void generateMipmaps(
      vk::Image img, int32_t width, int32_t height, vk::Format format,
      uint32_t mip_levels);

  Material* loadMaterial(const MaterialInfo& mat_info);
  Texture* loadTexture(std::string path);
  std::unique_ptr<Model> loadMesh(const Mesh& mesh);
  Texture* getColorTexture(uint32_t color);
  void stageVertices(const std::vector<Vertex>& vertices, Model& model);
  void stageIndices(const std::vector<uint32_t>& indices, Model& model);

  // Copy data to a CPU staging buffer, create a GPU buffer, and submit a copy
  // from the staging_buf to dst_buf.
  void stageBuffer(
      vk::DeviceSize size, void* data, vk::BufferUsageFlags usage,
      vk::UniqueBuffer& dst_buf, vk::UniqueDeviceMemory& dst_buf_mem);

  vk::CommandBuffer beginSingleTimeCommands();
  void endSingleTimeCommands(vk::CommandBuffer cmd_buf);

  void createDescriptorPool();
  void createImguiDescriptorPool();

  void renderScene();

  void recordCommandBuffer();
  void createSyncObjects();
  void recreateSwapchain();

  // The window should only be used in createSurface().
  SDL_Window* window_ = nullptr;
  bool window_resized_ = false;
  uint32_t width_ = 100;
  uint32_t height_ = 100;

  vk::UniqueInstance instance_;
  vk::DispatchLoaderDynamic dldi_;
  vk::UniqueSurfaceKHR surface_;
  vk::UniqueHandle<vk::DebugUtilsMessengerEXT, vk::DispatchLoaderDynamic>
      dbg_messenger_;
  vk::PhysicalDevice physical_device_;
  // Indices of queue families for the selected |physical_device_|
  QueueFamilyIndices q_indices_;
  vk::UniqueDevice device_;
  vk::Queue gfx_q_;
  vk::Queue present_q_;
  SwapchainSupportDetails swapchain_support_;
  vk::UniqueSwapchainKHR swapchain_;
  std::vector<vk::UniqueImageView> swapchain_views_;
  vk::UniqueDescriptorPool desc_pool_;
  vk::UniqueDescriptorPool imgui_desc_pool_;
  vk::UniqueCommandPool cmd_pool_;
  std::vector<vk::UniqueCommandBuffer> cmd_bufs_;
  std::vector<vk::UniqueSemaphore> img_sems_;
  std::vector<vk::UniqueSemaphore> render_sems_;
  std::vector<vk::UniqueFence> in_flight_fences_;
  // TODO: Make some factory that creates and caches samplers by properties.
  vk::UniqueSampler linear_sampler_;
  vk::UniqueSampler nearest_sampler_;
  vk::UniqueSampler clamp_sampler_;
  vk::SampleCountFlagBits msaa_samples_ = vk::SampleCountFlagBits::e1;
  vk::SampleCountFlagBits scene_samples_ = vk::SampleCountFlagBits::e4;

  // The members in this struct should never change after handing it out, or
  // problems are likely to occur.
  VulkanState vs_{.kMaxFramesInFlight = 2};

  FrameState* frame_state_ = nullptr;
  DrawState ds_{};

  std::map<ModelId, std::unique_ptr<Model>> loaded_models_;
  std::vector<std::unique_ptr<Material>> loaded_materials_;
  std::vector<std::unique_ptr<Texture>> loaded_textures_;
  std::map<uint32_t, Texture*> color_textures_;

  Drawing drawing_;
  Voronoi voronoi_;
  Scene scene_;
  Edges edges_;
  JumpFlood jf_;
  Swap swap_;
  Resolve resolve_;
  SampleQuery sample_query_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
