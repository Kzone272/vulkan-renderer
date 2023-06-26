set (IMGUI_DIR "${CMAKE_CURRENT_LIST_DIR}/imgui")
if (EXISTS ${IMGUI_DIR})

  file(GLOB imgui_headers
    ${IMGUI_DIR}/*.h
  )
  file(GLOB imgui_cpps
    ${IMGUI_DIR}/*.cpp
  )

  add_library(imgui ${imgui_cpps})
  target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_LIST_DIR})

  set(imgui_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/imgui")
  set(imgui_LIBRARIES imgui)
  add_library(imgui::imgui ALIAS imgui)

else()
  message("imgui submodule not cloned.")
  # set (imgui_FOUND 0)

endif()
