#pragma once

#include <imgui/imgui.h>

// Same as ImGui::SliderFloat but right clicking on the slider will reset value
// to v_default.
bool SliderFloatDefault(
    const char *label, float *v, float v_min, float v_max, float v_default,
    const char *format = "%.3f", ImGuiSliderFlags flags = 0);
