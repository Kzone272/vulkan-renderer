# The main app for this project
add_executable(main
  main.cc
  app.h
  app.cc
  renderer.h
  renderer.cc
  asserts.h
  asserts.cc
  defines.h
  input.h
  input.cc
  vulkan-include.h
  maths.h
  maths.cc
  glm-include.h
  # object.h
  # object.cc
  render-objects.h
  animation.h
  time-include.h
  time-utils.h
  time-utils.cc
  primitives.h
  skelly.h
  skelly.cc
  skelly-presets.h
  descriptors.h
  descriptors.cc
  files.h
  pipelines.h
  pipelines.cc
  assets.h
  assets.cc
  images.h
  images.cc
  fbo.h
  fbo.cc
  pass.h
  pass.cc
  render-state.h
  buffers.h
  buffers.cc
  shaders.h
  shaders.cc
  scene-data.h
  second-order.h
  skeleton.h
  biped-rig.h
  biped-rig.cc
  transform.h
  transform.cc
  pose.h
  pose.cc
  imgui-helpers.h
  imgui-helpers.cc
  vma-usage.h
  vma-usage.cc
  materials.cc
  command-buffers.cc
  entities.cc
)

target_compile_features(main PRIVATE cxx_std_23)
# Disable C++ modules because it breaks precompiled headers
set(CMAKE_EXPERIMENTAL_CXX_MODULE_DYNDEP 0)

add_dependencies(main shaders)

target_include_directories(main PRIVATE
  ${PROJECT_SOURCE_DIR}/deps
  ${PROJECT_SOURCE_DIR}/deps/VulkanMemoryAllocator/include
  ${PROJECT_SOURCE_DIR}/deps/VulkanMemoryAllocator-Hpp/include
  # Needed because Imgui itself includes headers relative to this folder.
  ${IMGUI_DIR} 
  ${Vulkan_INCLUDE_DIRS}
  ${TINYOBJLOADER_INCLUDE_DIRS}
  ${COMMON_INCLUDE_DIRS}
)

target_precompile_headers(main
  PRIVATE
    vulkan-include.h
)

target_link_libraries(main PRIVATE
  SDL2::SDL2
  SDL2_image::SDL2_image
  ${Vulkan_LIBRARIES}
  imgui
)

# Copy assets directory to build directory.
add_custom_target(assets)
add_custom_command(TARGET main POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:main>/assets"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/assets"
    "$<TARGET_FILE_DIR:main>/assets"
)
add_dependencies(main assets)

# Copy dlls to build directory.
add_custom_target(dlls)
add_custom_command(TARGET main POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${DEPS_DIR}/bin"
    "$<TARGET_FILE_DIR:main>"
)
add_dependencies(main dlls)

add_subdirectory(shaders)
