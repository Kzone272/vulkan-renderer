# The main app for this project
add_executable(main
  main.cc
  hello_triangle_app.h
  asserts.h
  defines.h
)
target_compile_features(main PRIVATE cxx_std_17)

target_include_directories(main PRIVATE
  ${SDL2_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Vulkan_INCLUDE_DIRS}
)

target_link_libraries(main PRIVATE
  SDL2::SDL2
  ${Vulkan_LIBRARIES}
)


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
