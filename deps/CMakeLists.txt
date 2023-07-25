project(Deps)
cmake_minimum_required(VERSION 3.25)

include(ExternalProject)

set(CMAKE_ARGS 
    "-DCMAKE_BUILD_TYPE:PATH=${CMAKE_BUILD_TYPE}"
    "-DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}"
    "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
    "-DBUILD_TESTING=OFF"
)

ExternalProject_Add(glm
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glm
  CMAKE_ARGS ${CMAKE_ARGS}
  BINARY_DIR glm
)

ExternalProject_Add(SDL
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDL
  CMAKE_ARGS ${CMAKE_ARGS}
  BINARY_DIR SDL
)

ExternalProject_Add(SDL_image
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDL_image
  CMAKE_ARGS ${CMAKE_ARGS}
  BINARY_DIR SDL_image
)

ExternalProject_Add(tinyobjloader
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tinyobjloader
  CMAKE_ARGS ${CMAKE_ARGS}
  BINARY_DIR tinyobjloader
)

find_package(SDL2 CONFIG REQUIRED)
find_package(Vulkan REQUIRED)

set (IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui")
file(GLOB imgui_headers
  ${IMGUI_DIR}/*.h
  ${IMGUI_DIR}/backends/imgui_impl_sdl2.h
  ${IMGUI_DIR}/backends/imgui_impl_vulkan.h
)
file(GLOB imgui_cpps
  ${IMGUI_DIR}/*.cpp
  ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
  ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp
)
add_library(imgui STATIC ${imgui_cpps})
set_target_properties(imgui PROPERTIES CXX_STANDARD 11)

target_include_directories(imgui PRIVATE
  ${IMGUI_DIR}
  ${Vulkan_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
)
target_link_libraries(imgui PRIVATE
  ${Vulkan_LIBRARIES}
  ${SDL2_LIBRARIES}
)
target_sources(imgui PUBLIC
  FILE_SET HEADERS
  FILES ${imgui_headers}
)

install(TARGETS imgui
  EXPORT imguiTargets
  FILE_SET HEADERS
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include
  INCLUDES DESTINATION include
)

install(EXPORT imguiTargets
  FILE imguiConfig.cmake
  DESTINATION cmake
)
