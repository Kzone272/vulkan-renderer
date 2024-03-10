#pragma once

#include <vector>

#include "render-objects.h"

// Make a cube with origin at bottom middle.
constexpr Mesh makeCube() {
  Mesh m;

  // Vertices:
  // top x {back, front} x {left, right}
  vec3 tbl{-0.5, 1, 0.5};
  vec3 tbr{0.5, 1, 0.5};
  vec3 tfr{0.5, 1, -0.5};
  vec3 tfl{-0.5, 1, -0.5};
  // bottom x {back, front} x {left, right}
  vec3 bbl{-0.5, 0, 0.5};
  vec3 bbr{0.5, 0, 0.5};
  vec3 bfr{0.5, 0, -0.5};
  vec3 bfl{-0.5, 0, -0.5};
  // Normals:
  vec3 up{0, 1, 0};         // up
  vec3 down{0, -1, 0};      // down
  vec3 left{-1, 0, 0};      // left
  vec3 right{1, 0, 0};      // right
  vec3 forward{0, 0, 1};    // forward
  vec3 backward{0, 0, -1};  // backward

  m.vertices = {
      // Top
      {.pos{tbl}, .normal{up}},
      {.pos{tbr}, .normal{up}},
      {.pos{tfl}, .normal{up}},
      {.pos{tfr}, .normal{up}},
      // Bottom
      {.pos{bfl}, .normal{down}},
      {.pos{bfr}, .normal{down}},
      {.pos{bbl}, .normal{down}},
      {.pos{bbr}, .normal{down}},
      // Left
      {.pos{tbl}, .normal{left}},
      {.pos{tfl}, .normal{left}},
      {.pos{bbl}, .normal{left}},
      {.pos{bfl}, .normal{left}},
      // Right
      {.pos{tfr}, .normal{right}},
      {.pos{tbr}, .normal{right}},
      {.pos{bfr}, .normal{right}},
      {.pos{bbr}, .normal{right}},
      // Front
      {.pos{tfl}, .normal{backward}},
      {.pos{tfr}, .normal{backward}},
      {.pos{bfl}, .normal{backward}},
      {.pos{bfr}, .normal{backward}},
      // Back
      {.pos{tbr}, .normal{forward}},
      {.pos{tbl}, .normal{forward}},
      {.pos{bbr}, .normal{forward}},
      {.pos{bbl}, .normal{forward}},
  };
  m.indices = {
      0,  1,  2,  2,  1,  3,   // top
      4,  5,  6,  6,  5,  7,   // bottom
      8,  9,  10, 10, 9,  11,  // left
      12, 13, 14, 14, 13, 15,  // right
      16, 17, 18, 18, 17, 19,  // front
      20, 21, 22, 22, 21, 23,  // back
  };
  return std::move(m);
}

inline Mesh makePlane(float x_size, float z_size) {
  Mesh m;
  // repeat texture every 1m
  float x_reps = x_size / 100;
  float y_reps = z_size / 100;
  float hw = x_size / 2;  // half width
  float hd = z_size / 2;  // half depth
  vec3 up{0, 1, 0};
  m.vertices = {
      {.pos{-hw, 0, hd}, .normal{up}, .uv{0, 0}},            // bl
      {.pos{hw, 0, hd}, .normal{up}, .uv{x_reps, 0}},        // br
      {.pos{-hw, 0, -hd}, .normal{up}, .uv{0, y_reps}},      // fl
      {.pos{hw, 0, -hd}, .normal{up}, .uv{x_reps, y_reps}},  // fr
  };
  m.indices = {0, 1, 2, 2, 1, 3};
  return std::move(m);
}

namespace {
void iter(Mesh& mesh, int steps, bool in);
}  // namespace

inline Mesh tetrahedron(int steps, bool in) {
  vec3 a(0, 0, 2.f / 3.f);
  glm::quat r120 = glm::angleAxis(glm::radians(120.f), vec3(0, 1, 0));
  vec3 b = r120 * a;
  vec3 c = r120 * b;

  float lift = sqrt(8.f / 9.f);
  vec3 d(0, lift, 0);

  vec3 ad = a - d;
  vec3 bd = b - d;
  vec3 cd = c - d;
  vec3 abdN = glm::normalize(glm::cross(ad, bd));
  vec3 acdN = glm::normalize(glm::cross(cd, ad));
  vec3 bcdN = glm::normalize(glm::cross(bd, cd));
  vec3 abcN = {0, -1, 0};

  Mesh tetra;
  vec3 color = {0.1, 0.8, 1};

  tetra.vertices = {
      {.pos{a}, .normal{abcN}, .color{color}},
      {.pos{c}, .normal{abcN}, .color{color}},
      {.pos{b}, .normal{abcN}, .color{color}},
      {.pos{a}, .normal{abdN}, .color{color}},
      {.pos{b}, .normal{abdN}, .color{color}},
      {.pos{d}, .normal{abdN}, .color{color}},
      {.pos{b}, .normal{bcdN}, .color{color}},
      {.pos{c}, .normal{bcdN}, .color{color}},
      {.pos{d}, .normal{bcdN}, .color{color}},
      {.pos{c}, .normal{acdN}, .color{color}},
      {.pos{a}, .normal{acdN}, .color{color}},
      {.pos{d}, .normal{acdN}, .color{color}},
  };

  iter(tetra, steps, in);

  return std::move(tetra);
};

namespace {

void iter(Mesh& mesh, int steps, bool in) {
  for (int i = 0; i < steps; i++) {
    ASSERT(mesh.vertices.size() % 3 == 0);
    std::vector<Vertex> new_verts;
    for (int j = 0; j < mesh.vertices.size(); j += 3) {
      auto& av = mesh.vertices[j];
      auto& bv = mesh.vertices[j + 1];
      auto& cv = mesh.vertices[j + 2];
      vec3 a = av.pos;
      vec3 b = bv.pos;
      vec3 c = cv.pos;
      vec3 upN = av.normal;  // ABC should all have the same normal

      vec3 d = (b + c) / 2.f;
      vec3 dcol = (bv.color + cv.color) / 2.f;

      vec3 e = (a + c) / 2.f;
      vec3 ecol = (av.color + cv.color) / 2.f;

      vec3 f = (a + b) / 2.f;
      vec3 fcol = (av.color + bv.color) / 2.f;

      new_verts.insert(
          new_verts.end(), {av,
                            {.pos{f}, .normal{upN}, .color{fcol}},
                            {.pos{e}, .normal{upN}, .color{ecol}},
                            {.pos{f}, .normal{upN}, .color{fcol}},
                            bv,
                            {.pos{d}, .normal{upN}, .color{dcol}},
                            {.pos{d}, .normal{upN}, .color{dcol}},
                            cv,
                            {.pos{e}, .normal{upN}, .color{ecol}}});

      // point on abc plane:
      vec3 gprime = (a + b + c) / 3.f;
      vec3 ab = b - a;
      vec3 ac = c - a;
      vec3 normal = glm::normalize(glm::cross(ab, ac));

      float lift = 0.23;
      if (in) {
        lift *= -1;
      }
      float height = glm::length(d - a) * lift;
      vec3 g = gprime + height * normal;

      vec3 gcol = (av.color + bv.color + cv.color) / 3.f;
      gcol.r *= 0.5;
      gcol.g *= 0.5;

      vec3 dg = d - g;
      vec3 eg = e - g;
      vec3 fg = f - g;
      vec3 degN = glm::normalize(glm::cross(dg, eg));
      vec3 dfgN = glm::normalize(glm::cross(fg, dg));
      vec3 efgN = glm::normalize(glm::cross(eg, fg));

      new_verts.insert(
          new_verts.end(), {{.pos{e}, .normal{efgN}, .color{ecol}},
                            {.pos{f}, .normal{efgN}, .color{fcol}},
                            {.pos{g}, .normal{efgN}, .color{gcol}},
                            {.pos{f}, .normal{dfgN}, .color{fcol}},
                            {.pos{d}, .normal{dfgN}, .color{dcol}},
                            {.pos{g}, .normal{dfgN}, .color{gcol}},
                            {.pos{d}, .normal{degN}, .color{dcol}},
                            {.pos{e}, .normal{degN}, .color{ecol}},
                            {.pos{g}, .normal{degN}, .color{gcol}}});
    }
    mesh.vertices = std::move(new_verts);
  }
}

}  // namespace

inline Mesh makeSphere(int rows) {
  DASSERT(rows >= 3);
  const int slices = 2 * rows;

  Mesh sphere;
  glm::quat rY = glm::angleAxis(glm::radians(360.f / slices), vec3(0, 1, 0));
  glm::quat rZ = glm::angleAxis(glm::radians(180.f / rows), vec3(0, 0, 1));

  vec3 up(0, 1, 0);
  sphere.vertices.push_back({.pos{up}, .normal{up}});

  vec3 point = up;
  for (int i = 1; i < rows; i++) {
    point = rZ * point;
    for (int j = 0; j < slices; j++) {
      sphere.vertices.push_back({.pos{point}, .normal{point}});
      point = rY * point;
    }
  }
  sphere.vertices.push_back({.pos{-up}, .normal{-up}});

  // Top and bottom caps.
  const uint32_t last = sphere.vertices.size() - 1;
  for (uint32_t i = 0; i < slices; i++) {
    // Top cap
    uint32_t start = 1;
    uint32_t next_i = (i + 1) % slices;
    sphere.indices.insert(sphere.indices.end(), {0, start + next_i, start + i});
    // Bottom cap
    start = last - slices;
    sphere.indices.insert(
        sphere.indices.end(), {last, start + i, start + next_i});
  }

  // Middle rows
  for (int i = 0; i < rows - 2; i++) {
    for (int j = 0; j < slices; j++) {
      uint32_t tl = 1 + i * slices + j;
      uint32_t tr = 1 + i * slices + ((j + 1) % slices);
      uint32_t bl = tl + slices;
      uint32_t br = tr + slices;
      sphere.indices.insert(sphere.indices.end(), {tl, tr, bl, bl, tr, br});
    }
  }

  return std::move(sphere);
};
