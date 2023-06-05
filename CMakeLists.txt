cmake_minimum_required(VERSION 3.25)

project(VulkanTutorial
  VERSION 1.0
  DESCRIPTION "Vulkan Tutorial learning project"
  LANGUAGES CXX
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

find_package(glm REQUIRED)
find_package(SDL2 2.26.2 REQUIRED)
find_package(SDL2_image 2.6.2 REQUIRED)
find_package(Vulkan REQUIRED)

add_subdirectory(src)
