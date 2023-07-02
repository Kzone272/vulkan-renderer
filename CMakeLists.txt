cmake_minimum_required(VERSION 3.25)

project(VulkanTutorial
  VERSION 1.0
  DESCRIPTION "Vulkan Tutorial learning project"
  LANGUAGES CXX
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

find_package(glm 0.9.9.9 CONFIG REQUIRED)
find_package(SDL2 CONFIG REQUIRED)
find_package(SDL2_image CONFIG REQUIRED)
find_package(Vulkan REQUIRED)
find_package(tinyobjloader CONFIG REQUIRED)

add_subdirectory(src)
