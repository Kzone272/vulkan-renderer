#pragma once

#include <memory>
#include <span>
#include <vector>

#include "descriptors.h"
#include "fbo.h"
#include "images.h"
#include "pass.h"
#include "pipelines.h"
#include "render-objects.h"
#include "vulkan-include.h"

struct Buffer {
  vk::UniqueBuffer buf;
  vk::UniqueDeviceMemory mem;
  void* mapped = nullptr;
  vk::DescriptorBufferInfo info;
};

struct DynamicBuf {
  Buffer staging;
  Buffer device;
};

struct Material {
  Texture* diffuse;
  Buffer ubo;
  vk::DescriptorSet desc_set;
};

struct Model {
  vk::UniqueBuffer vert_buf;
  vk::UniqueDeviceMemory vert_buf_mem;
  vk::UniqueBuffer ind_buf;
  vk::UniqueDeviceMemory ind_buf_mem;
  uint32_t index_count;
  uint32_t vertex_count;
  Material* material;
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

  vk::Format findDepthFormat();
  bool hasStencilComponent(vk::Format format);
  vk::Format findSupportedFormat(
      const std::vector<vk::Format>& formats, vk::ImageTiling tiling,
      vk::FormatFeatureFlags features);

  vk::UniqueShaderModule createShaderModule(std::string filename);
  void createShaders();

  void initPass(Pass& pass);
  void createDrawing();
  void createVoronoi();
  void createScene();
  void createPost();
  void createSwap();

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
  void createBuffer(
      vk::DeviceSize size, vk::BufferUsageFlags usage,
      vk::MemoryPropertyFlags props, vk::UniqueBuffer& buf,
      vk::UniqueDeviceMemory& buf_mem);
  DynamicBuf createDynamicBuffer(
      vk::DeviceSize size, vk::BufferUsageFlags usage);

  vk::CommandBuffer beginSingleTimeCommands();
  void endSingleTimeCommands(vk::CommandBuffer cmd_buf);

  void createDescriptorPool();
  void createImguiDescriptorPool();

  void beginRp(const Fbo& fbo, int fb_ind);
  void renderDrawing();
  void renderVoronoi();
  void renderScene();
  void renderPost();
  void renderSwap();

  void recordCommandBuffer();
  void createSyncObjects();
  void recreateSwapchain();

  // The window should only be used in createSurface().
  SDL_Window* window_ = nullptr;
  bool window_resized_ = false;
  uint32_t width_ = 100;
  uint32_t height_ = 100;

  // The members in this struct should never change after handing it out, or
  // problems are likely to occur.
  VulkanState vs_;

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
  vk::Format color_fmt_ = vk::Format::eB8G8R8A8Unorm;
  vk::Format swapchain_format_;
  vk::Extent2D swapchain_extent_;
  std::vector<vk::UniqueImageView> swapchain_views_;
  vk::UniqueDescriptorPool desc_pool_;
  vk::UniqueDescriptorPool imgui_desc_pool_;
  vk::UniqueShaderModule scene_vert_;
  vk::UniqueShaderModule scene_frag_;
  vk::UniqueShaderModule fullscreen_vert_;
  vk::UniqueShaderModule post_frag_;
  vk::UniqueShaderModule circle_frag_;
  vk::UniqueShaderModule sample_frag_;
  vk::UniqueShaderModule voronoi_vert_;
  vk::UniqueShaderModule voronoi_frag_;
  vk::UniqueCommandPool cmd_pool_;
  std::vector<vk::UniqueCommandBuffer> cmd_bufs_;
  std::vector<vk::UniqueSemaphore> img_sems_;
  std::vector<vk::UniqueSemaphore> render_sems_;
  std::vector<vk::UniqueFence> in_flight_fences_;
  vk::UniqueSampler linear_sampler_;
  vk::UniqueSampler nearest_sampler_;

  vk::SampleCountFlagBits msaa_samples_ = vk::SampleCountFlagBits::e1;

  FrameState* frame_state_ = nullptr;

  struct DrawState {
    vk::CommandBuffer cmd = {};
    int frame = -1;
    uint32_t img_ind = 0;
  } ds_;

  std::map<ModelId, std::unique_ptr<Model>> loaded_models_;
  std::vector<std::unique_ptr<Material>> loaded_materials_;
  std::vector<std::unique_ptr<Texture>> loaded_textures_;
  std::map<uint32_t, Texture*> color_textures_;

  struct Drawing {
    Pass pass;
    Pipeline* draw;
    std::vector<DynamicBuf> debugs;
    DescLayout* inputs;

    void update(const DrawState& ds, const DebugData& debug);
  } drawing_;

  struct Voronoi {
    Pass pass;
    Pipeline* draw;
    std::vector<DynamicBuf> verts;
  } voronoi_;

  struct Scene {
    Pass pass;
    std::vector<DynamicBuf> globals;
    DescLayout* global;
    DescLayout* material;
    Pipeline* draw;

    void update(const DrawState& ds, const FrameState& fs);
  } scene_;

  struct Post {
    Pass pass;
    std::vector<DynamicBuf> debugs;
    DescLayout* inputs;
    Pipeline* draw;

    void update(const DrawState& ds, const DebugData& debug);
  } post_;

  struct Swap {
    Pass pass;
    DescLayout* sampler;
    Pipeline* draw;
  } swap_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};

// TODO: Move to separate file buffers.h.
template <class T>
void updateDynamicBuf(
    vk::CommandBuffer cmd, DynamicBuf& dbuf, std::span<T> data,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access);

// TODO: Move to separate file. Maybe descriptors.h?
template <class T>
static vk::VertexInputBindingDescription getBindingDesc() {
  return {
      .binding = 0,
      .stride = sizeof(T),
      .inputRate = vk::VertexInputRate::eVertex,
  };
}

template <class T>
static std::vector<vk::VertexInputAttributeDescription> getAttrDescs();

template <>
static std::vector<vk::VertexInputAttributeDescription> getAttrDescs<Vertex>() {
  std::vector<vk::VertexInputAttributeDescription> attrs = {
      {
          .location = 0,
          .binding = 0,
          .format = vk::Format::eR32G32B32Sfloat,  // vec3
          .offset = offsetof(Vertex, pos),
      },
      {
          .location = 1,
          .binding = 0,
          .format = vk::Format::eR32G32B32Sfloat,  // vec3
          .offset = offsetof(Vertex, normal),
      },
      {
          .location = 2,
          .binding = 0,
          .format = vk::Format::eR32G32B32Sfloat,  // vec3
          .offset = offsetof(Vertex, color),
      },
      {
          .location = 3,
          .binding = 0,
          .format = vk::Format::eR32G32Sfloat,  // vec2
          .offset = offsetof(Vertex, uv),
      },
  };
  return attrs;
}

template <>
static std::vector<vk::VertexInputAttributeDescription>
getAttrDescs<Vertex2d>() {
  std::vector<vk::VertexInputAttributeDescription> attrs = {
      {
          .location = 0,
          .binding = 0,
          .format = vk::Format::eR32G32Sfloat,  // vec2
          .offset = offsetof(Vertex2d, pos),
      },
      {
          .location = 1,
          .binding = 0,
          .format = vk::Format::eR32G32B32Sfloat,  // vec3
          .offset = offsetof(Vertex2d, color),
      },
  };
  return attrs;
}
