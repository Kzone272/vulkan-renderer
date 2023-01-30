cmake_minimum_required(VERSION 3.25)

project(VulkanTutorial
  VERSION 1.0
  DESCRIPTION "Vulkan Tutorial learning project"
  LANGUAGES CXX
)

include(FetchContent)

FetchContent_Declare(
  eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen3)

find_package(Eigen3 CONFIG REQUIRED)

add_subdirectory(src)
