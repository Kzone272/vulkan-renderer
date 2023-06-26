set(DEPS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

set (IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui")

file(GLOB imgui_headers
  ${IMGUI_DIR}/*.h
)
file(GLOB imgui_cpps
  ${IMGUI_DIR}/*.cpp
)

add_library(imgui ${imgui_cpps})
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(imgui::imgui ALIAS imgui)

