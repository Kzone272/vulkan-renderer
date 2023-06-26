cmake_minimum_required(VERSION 3.25)

project(VulkanTutorial
  VERSION 1.0
  DESCRIPTION "Vulkan Tutorial learning project"
  LANGUAGES CXX
)

add_subdirectory(deps)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

find_package(SDL2_image REQUIRED)
find_package(Vulkan REQUIRED)
find_package(tinyobjloader REQUIRED)
# This list of components is maybe questionable.
find_package(Boost
  REQUIRED
    stacktrace_noop
  OPTIONAL_COMPONENTS
    stacktrace_windbg
    stacktrace_basic
)

add_subdirectory(src)
