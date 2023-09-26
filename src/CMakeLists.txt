# The main app for this project
add_executable(main
  main.cc
  app.h
  renderer.h
  asserts.h
  asserts.cc
  defines.h
  vulkan-include.h
  maths.h
  maths.cc
  strings.h
  glm-include.h
  object.h
  render-objects.h
  animation.h
  time-include.h
  primitives.h
  skelly.h
  descriptors.h
  files.h
  pipelines.h
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
