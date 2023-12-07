cmake_minimum_required(VERSION 3.25)

add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/MP>)

project(VulkanRenderer
  VERSION 1.0
  DESCRIPTION "Vulkan Renderer learning project"
  LANGUAGES CXX
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

find_package(glm 0.9.9.9 CONFIG REQUIRED)
find_package(SDL2 2.28.0 CONFIG REQUIRED)
find_package(SDL2_image 2.6.3 CONFIG REQUIRED)
find_package(Vulkan REQUIRED)
find_package(tinyobjloader 2.0.0 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_subdirectory(src)
