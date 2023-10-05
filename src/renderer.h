#pragma once

#include <memory>
#include <vector>

#include "descriptors.h"
#include "frame-state.h"
#include "pipelines.h"
#include "render-objects.h"
#include "vulkan-include.h"

struct Texture {
  // Inputs
  vk::Extent2D size;
  vk::Format format;
  uint32_t mip_levels = 1;
  vk::SampleCountFlagBits samples = vk::SampleCountFlagBits::e1;
  // Outputs
  vk::UniqueImage image;
  vk::UniqueDeviceMemory image_mem;
  vk::UniqueImageView image_view;
  vk::DescriptorImageInfo info;
};

struct Fbo {
  // Inputs
  vk::Extent2D size;
  std::vector<vk::Format> color_fmts;
  vk::SampleCountFlagBits samples = vk::SampleCountFlagBits::e1;
  bool depth_test = false;
  bool resolve = false;
  bool swap = false;
  std::vector<vk::ImageView> swap_views;
  // Outputs
  std::vector<Texture> colors;
  std::vector<Texture> resolves;
  Texture depth;
  vk::UniqueRenderPass rp;
  std::vector<vk::ClearValue> clears;
  std::vector<vk::UniqueFramebuffer> fbs;
};

struct Ubo {
  vk::UniqueBuffer buf;
  vk::UniqueDeviceMemory buf_mem;
  void* buf_mapped;
  vk::DescriptorBufferInfo info;
};

struct Material {
  Texture* diffuse;
  Ubo ubo;
  vk::DescriptorSet desc_set;

  constexpr static vk::DeviceSize size =
      sizeof(MaterialInfo::UniformBufferObject);
};

struct Model {
  vk::UniqueBuffer vert_buf;
  vk::UniqueDeviceMemory vert_buf_mem;
  vk::UniqueBuffer ind_buf;
  vk::UniqueDeviceMemory ind_buf_mem;
  uint32_t index_count;
  Material* material;
};

// TODO: This probably shouldn't exist. I think DescLayouts should just move
// into Pipeline.
struct Pipe {
  std::vector<DescLayout> desc_los;
  Pipeline pl;
};

// An offscreen render target that can be sampled.
struct Canvas {
  Fbo fbo;
  std::vector<Pipe> pipes;
};

struct SDL_Window;
struct SDL_Surface;

class Renderer {
 public:
  Renderer(SDL_Window* window, uint32_t width, uint32_t height) {
    window_ = window;
    width_ = width;
    height_ = height;
  }

  void init(FrameState* frame_state);
  void drawFrame(FrameState* frame_state);
  void cleanup();
  void resizeWindow(uint32_t width, uint32_t height);
  void useModel(ModelId model_id, const ModelInfo& model_info);
  MaterialId useMaterial(const MaterialInfo& mat_info);
  void useMesh(ModelId model_id, const Mesh& mesh, MaterialId mat_id);
  // TODO: Return a TextureId instead.
  Texture* getDrawingTexture();
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
  void updateUboData(int frame);
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

  void createImage(
      Texture& texture, vk::ImageTiling tiling, vk::ImageUsageFlags usage,
      vk::MemoryPropertyFlags props, vk::ImageAspectFlags aspect);
  vk::UniqueImageView createImageView(
      vk::Image img, vk::Format format, uint32_t mip_levels,
      vk::ImageAspectFlags aspect_flags);
  void createCommandPool();
  void createCommandBuffers();

  vk::Format findDepthFormat();
  bool hasStencilComponent(vk::Format format);
  vk::Format findSupportedFormat(
      const std::vector<vk::Format>& formats, vk::ImageTiling tiling,
      vk::FormatFeatureFlags features);

  void createFbos();
  void createDescriptorSetLayouts();
  vk::UniqueShaderModule createShaderModule(std::string filename);
  void createShaders();
  void createGraphicsPipelines();

  void initFbo(Fbo& fbo);
  void createCanvas(Canvas& canvas);
  void createCanvasPipe(Canvas& canvas);

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
  void createTextureSampler();

  std::unique_ptr<Model> loadModel(const ModelInfo& model_info);
  Material* loadMaterial(const MaterialInfo& mat_info);
  Texture* loadTexture(std::string path);
  std::unique_ptr<Model> loadMesh(const Mesh& mesh);
  Texture* getColorTexture(uint32_t color);
  Mesh loadObj(std::string obj_path);
  void stageVertices(const std::vector<Vertex>& vertices, Model& model);
  void stageIndices(const std::vector<uint32_t>& indices, Model& model);

  void createInFlightBuffers();
  Ubo createMappedBuf(vk::DeviceSize size);
  // Copy data to a CPU staging buffer, create a GPU buffer, and submit a copy
  // from the staging_buf to dst_buf.
  void stageBuffer(
      vk::DeviceSize size, void* data, vk::BufferUsageFlags usage,
      vk::UniqueBuffer& dst_buf, vk::UniqueDeviceMemory& dst_buf_mem);
  void copyBuffer(vk::Buffer src_buf, vk::Buffer dst_buf, vk::DeviceSize size);
  void createBuffer(
      vk::DeviceSize size, vk::BufferUsageFlags usage,
      vk::MemoryPropertyFlags props, vk::UniqueBuffer& buf,
      vk::UniqueDeviceMemory& buf_mem);
  uint32_t findMemoryType(uint32_t type_filter, vk::MemoryPropertyFlags props);

  vk::CommandBuffer beginSingleTimeCommands();
  void endSingleTimeCommands(vk::CommandBuffer cmd_buf);

  void createDescriptorPool();
  void createImguiDescriptorPool();
  void createInFlightDescSets();
  void updateResizedDescSets();

  void beginRp(const vk::CommandBuffer& cmd_buf, const Fbo& fbo, int fb_ind);
  void renderCanvas(
      const vk::CommandBuffer& cmd_buf, int frame, const Canvas& canvas);
  void recordCommandBuffer(int frame, uint32_t img_ind);
  void createSyncObjects();
  void recreateSwapchain();
  void cleanupSwapchain();

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
  vk::PhysicalDeviceProperties device_props_;
  // Indices of queue families for the selected |physical_device_|
  QueueFamilyIndices q_indices_;
  vk::UniqueDevice device_;
  vk::Queue gfx_q_;
  vk::Queue present_q_;
  SwapchainSupportDetails swapchain_support_;
  vk::UniqueSwapchainKHR swapchain_;
  vk::Format color_fmt_ = vk::Format::eB8G8R8A8Unorm;
  vk::Format swapchain_format_;
  vk::Extent2D swapchain_extent_;
  std::vector<vk::UniqueImageView> swapchain_views_;
  Fbo scene_fbo_;
  Fbo post_fbo_;
  vk::UniqueDescriptorPool desc_pool_;
  vk::UniqueDescriptorPool imgui_desc_pool_;
  vk::UniqueShaderModule scene_vert_;
  vk::UniqueShaderModule scene_frag_;
  vk::UniqueShaderModule fullscreen_vert_;
  vk::UniqueShaderModule post_frag_;
  vk::UniqueShaderModule circle_frag_;
  Pipeline scene_pl_;
  Pipeline post_pl_;
  vk::UniqueCommandPool cmd_pool_;
  std::vector<vk::UniqueCommandBuffer> cmd_bufs_;
  std::vector<vk::UniqueSemaphore> img_sems_;
  std::vector<vk::UniqueSemaphore> render_sems_;
  std::vector<vk::UniqueFence> in_flight_fences_;
  vk::UniqueSampler texture_sampler_;

  vk::SampleCountFlagBits msaa_samples_ = vk::SampleCountFlagBits::e1;

  FrameState* frame_state_ = nullptr;

  struct InFlightState {
    std::vector<Ubo> global;
    std::vector<Ubo> post;
  } in_flight_;

  std::map<ModelId, std::unique_ptr<Model>> loaded_models_;
  std::vector<std::unique_ptr<Material>> loaded_materials_;
  std::vector<std::unique_ptr<Texture>> loaded_textures_;
  std::map<uint32_t, Texture*> color_textures_;

  // Bound per frame.
  DescLayout global_dl_{
      .binds = {{.type = vk::DescriptorType::eUniformBuffer}},
      .stages =
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
  };
  // Bound per material
  DescLayout material_dl_{
      .binds =
          {{.type = vk::DescriptorType::eCombinedImageSampler},
           {.type = vk::DescriptorType::eUniformBuffer}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };
  // Bound in post processing step.
  DescLayout post_dl_{
      .binds =
          {{.type = vk::DescriptorType::eUniformBuffer},
           {.type = vk::DescriptorType::eCombinedImageSampler}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  Canvas drawing_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
