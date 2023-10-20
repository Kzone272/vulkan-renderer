find_program(glslc_executable NAMES glslc HINTS Vulkan::glslc)

file(GLOB shader_files
  shader.vert
  shader.frag
  fullscreen.vert
  post.frag
  sample.frag
  circle.frag
  voronoi.vert
  voronoi.frag
)

set(shader_binary_dir "${PROJECT_BINARY_DIR}/shaders")

foreach(shader_path IN LISTS shader_files)
  get_filename_component(shader_name ${shader_path} NAME)
  set(spirv_path "${shader_binary_dir}/${shader_name}.spv")

  add_custom_command(
    OUTPUT ${spirv_path}
    COMMAND ${CMAKE_COMMAND} -E make_directory "${shader_binary_dir}/"
    COMMAND ${glslc_executable} ${shader_path} -o ${spirv_path}
    DEPENDS ${shader_path}
  )
  list(APPEND spirv_shaders ${spirv_path})
endforeach()

add_custom_target(shaders ALL DEPENDS ${spirv_shaders})

add_custom_command(TARGET shaders POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:main>/shaders"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${shader_binary_dir}"
      "$<TARGET_FILE_DIR:main>/shaders"
)
