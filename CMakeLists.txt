cmake_minimum_required(VERSION 3.25)

project(VulkanTutorial
  VERSION 1.0
  DESCRIPTION "Vulkan Tutorial learning project"
  LANGUAGES CXX
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

find_package(Eigen3 3.4.0 REQUIRED)
find_package(SDL2 2.26.2 REQUIRED)
find_package(Vulkan REQUIRED)

add_subdirectory(src)
