# App for testing that the project setup works.
add_executable(setup
  setup.cc
  utils.h
)
target_compile_features(setup PRIVATE cxx_std_17)

target_include_directories(setup PRIVATE
  ${SDL2_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Vulkan_INCLUDE_DIRS}
)

target_link_libraries(setup PRIVATE
  SDL2::SDL2
  ${Vulkan_LIBRARIES}
)
