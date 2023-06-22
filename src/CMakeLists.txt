# The main app for this project
add_executable(main
  main.cc
  hello_triangle_app.h
  asserts.h
  asserts.cc
  defines.h
  vulkan-include.h
  utils.h
)
target_compile_features(main PRIVATE cxx_std_20)
add_dependencies(main shaders)

target_include_directories(main PRIVATE
  ${SDL2_INCLUDE_DIRS}
  ${Vulkan_INCLUDE_DIRS}
  ${TINYOBJLOADER_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Needed to satisfy boost-stacktrace
set(CMAKE_CXX_FLAGS "-D_GNU_SOURCE")

target_precompile_headers(main
  PRIVATE
    vulkan-include.h
)

target_link_libraries(main PRIVATE
  SDL2::SDL2
  SDL2_image::SDL2_image
  glm::glm
  ${Vulkan_LIBRARIES}
  ${Boost_LIBRARIES}
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


# App for testing that the project setup works.
add_executable(setup
  setup.cc
)
target_compile_features(setup PRIVATE cxx_std_17)

target_include_directories(setup PRIVATE
  ${SDL2_INCLUDE_DIRS}
  ${Vulkan_INCLUDE_DIRS}
)

target_link_libraries(setup PRIVATE
  SDL2::SDL2
  glm::glm
  ${Vulkan_LIBRARIES}
)

add_subdirectory(shaders)
