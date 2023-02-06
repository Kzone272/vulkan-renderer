add_executable(app
  main.cc
  utils.h
)
target_compile_features(app PRIVATE cxx_std_17)

target_include_directories(app PRIVATE
  ${SDL2_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Vulkan_INCLUDE_DIRS}
)

target_link_libraries(app PRIVATE
  SDL2::SDL2
  ${Vulkan_LIBRARIES}
)
