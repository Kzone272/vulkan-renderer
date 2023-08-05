#pragma once

#include <vector>

#include "render-objects.h"

// Make a cube with origin at bottom middle.
Mesh makeCube() {
  Mesh m;
  m.vertices = {
      // Bottom
      {.pos{-0.5, 0, 0.5}},   // bl
      {.pos{0.5, 0, 0.5}},    // br
      {.pos{0.5, 0, -0.5}},   // fr
      {.pos{-0.5, 0, -0.5}},  // fl
      // Top
      {.pos{-0.5, 1, 0.5}},   // bl
      {.pos{0.5, 1, 0.5}},    // br
      {.pos{0.5, 1, -0.5}},   // fr
      {.pos{-0.5, 1, -0.5}},  // fl
  };
  m.indices = {
      3, 2, 1, 3, 1, 0,  // bot
      4, 5, 6, 4, 6, 7,  // top
      4, 7, 3, 4, 3, 0,  // left
      6, 5, 1, 6, 1, 2,  // right
      7, 6, 2, 7, 2, 3,  // front
      5, 4, 0, 5, 0, 1,  // back
  };
  return std::move(m);
}

Mesh makePlane(float x_size, float z_size) {
  Mesh m;
  // repeat texture every 1m
  float x_reps = x_size / 100;
  float y_reps = z_size / 100;
  m.vertices = {
      {.pos{-x_size / 2, 0, z_size / 2}, .uv{0, 0}},            // bl
      {.pos{x_size / 2, 0, z_size / 2}, .uv{x_reps, 0}},        // br
      {.pos{x_size / 2, 0, -z_size / 2}, .uv{x_reps, y_reps}},  // fr
      {.pos{-x_size / 2, 0, -z_size / 2}, .uv{0, y_reps}},      // fl
  };
  m.indices = {0, 1, 2, 0, 2, 3};
  return std::move(m);
}

void iter(Mesh& mesh, int steps, bool in);

Mesh tetrahedron(int steps, bool in) {
  vec3 a(0, 0, 2.f / 3.f);
  glm::quat r120 = glm::angleAxis(glm::radians(120.f), vec3(0, 1, 0));
  vec3 b = r120 * a;
  vec3 c = r120 * b;

  float lift = sqrt(8.f / 9.f);
  vec3 d(0, lift, 0);

  Mesh tetra;
  vec3 color = {0.1, 0.8, 1};
  tetra.vertices.insert(
      tetra.vertices.begin(), {{.pos{a}, .color{color}},
                               {.pos{b}, .color{color}},
                               {.pos{c}, .color{color}},
                               {.pos{d}, .color{color}}});
  uint32_t ai = 0, bi = 1, ci = 2, di = 3;

  tetra.indices.insert(
      tetra.indices.end(), {ai, ci, bi, ai, bi, di, bi, ci, di, ci, ai, di});

  iter(tetra, steps, in);

  return std::move(tetra);
};

void iter(Mesh& mesh, int steps, bool in) {
  for (int i = 0; i < steps; i++) {
    ASSERT(mesh.indices.size() % 3 == 0);
    std::vector<uint32_t> new_indices;
    for (int j = 0; j < mesh.indices.size(); j += 3) {
      uint32_t ai = mesh.indices[j];
      uint32_t bi = mesh.indices[j + 1];
      uint32_t ci = mesh.indices[j + 2];
      auto av = mesh.vertices[ai];
      auto bv = mesh.vertices[bi];
      auto cv = mesh.vertices[ci];
      const vec3 a = av.pos;
      const vec3 b = bv.pos;
      const vec3 c = cv.pos;

      vec3 d = (b + c) / 2.f;
      vec3 dcol = (bv.color + cv.color) / 2.f;
      uint32_t di = mesh.vertices.size();
      mesh.vertices.push_back({.pos{d}, .color{dcol}});

      vec3 e = (a + c) / 2.f;
      vec3 ecol = (av.color + cv.color) / 2.f;
      uint32_t ei = mesh.vertices.size();
      mesh.vertices.push_back({.pos{e}, .color{ecol}});

      vec3 f = (a + b) / 2.f;
      vec3 fcol = (av.color + bv.color) / 2.f;
      uint32_t fi = mesh.vertices.size();
      mesh.vertices.push_back({.pos{f}, .color{fcol}});

      new_indices.insert(
          new_indices.end(), {ai, fi, ei, fi, bi, di, di, ci, ei});

      // point on abc plane:
      vec3 gprime = (a + b + c) / 3.f;
      vec3 ab = b - a;
      vec3 ac = c - a;
      vec3 normal = glm::normalize(glm::cross(ab, ac));

      float lift = 1.f / 4.f;
      if (in) {
        lift *= -1;
      }
      float height = glm::length(d - a) * lift;
      vec3 g = gprime + height * normal;

      vec3 gcol = (av.color + bv.color + cv.color) / 3.f;
      gcol.r *= 0.5;
      gcol.g *= 0.5;

      uint32_t gi = mesh.vertices.size();
      mesh.vertices.push_back({.pos{g}, .color{gcol}});

      new_indices.insert(
          new_indices.end(), {ei, fi, gi, fi, di, gi, di, ei, gi});
    }
    mesh.indices = std::move(new_indices);
  }
}
