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
  object.h
  object.cc
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
)

target_compile_features(main PRIVATE cxx_std_23)
# Disable C++ modules because it breaks precompiled headers
set(CMAKE_EXPERIMENTAL_CXX_MODULE_DYNDEP 0)

add_dependencies(main shaders)

target_include_directories(main PRIVATE
  ${DEPS_INCLUDE_DIR}
  ${DEPS_INCLUDE_DIR}/imgui
  ${Vulkan_INCLUDE_DIRS}
  ${TINYOBJLOADER_INCLUDE_DIRS}
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

add_subdirectory(shaders)
