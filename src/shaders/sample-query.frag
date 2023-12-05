#version 450

layout(location = 0) in vec2 fragUv;

layout(location = 0) out vec2 outColor;

void main() {
  outColor = gl_SamplePosition;
}
