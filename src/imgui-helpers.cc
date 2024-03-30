#include "imgui-helpers.h"

bool SliderFloatDefault(
    const char *label, float *v, float v_min, float v_max, float v_default,
    const char *format, ImGuiSliderFlags flags) {
  bool r = ImGui::SliderFloat(label, v, v_min, v_max, format, flags);

  if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
    *v = v_default;
  }

  return r;
}
