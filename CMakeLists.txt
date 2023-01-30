cmake_minimum_required(VERSION 3.25)

project(VulkanTutorial
  VERSION 1.0
  DESCRIPTION "Vulkan Tutorial learning project"
  LANGUAGES CXX
)

find_package(Eigen3 CONFIG 3.4.0 REQUIRED)
find_package(SDL2 CONFIG 2.26.2 REQUIRED)

add_subdirectory(src)
