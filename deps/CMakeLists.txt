cmake_minimum_required(VERSION 3.25)
project(Deps)

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

ExternalProject_Add(googletest
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/googletest
  CMAKE_ARGS ${CMAKE_ARGS}
  BINARY_DIR googletest
)
