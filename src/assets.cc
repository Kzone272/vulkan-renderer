#include "assets.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include "asserts.h"

Mesh loadObj(std::string obj_path) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn;
  std::string err;
  ASSERT(tinyobj::LoadObj(
      &attrib, &shapes, &materials, &warn, &err, obj_path.c_str()));

  Mesh mesh;
  std::unordered_map<Vertex, uint32_t> uniq_verts;
  for (const auto& shape : shapes) {
    for (const auto& index : shape.mesh.indices) {
      Vertex vert{};
      vert.pos = {
          attrib.vertices[3 * index.vertex_index],
          attrib.vertices[3 * index.vertex_index + 1],
          attrib.vertices[3 * index.vertex_index + 2],
      };
      vert.uv = {
          attrib.texcoords[2 * index.texcoord_index],
          1.f - attrib.texcoords[2 * index.texcoord_index + 1],  // Flip v
      };
      vert.normal = {
          attrib.normals[3 * index.normal_index],
          attrib.normals[3 * index.normal_index + 1],
          attrib.normals[3 * index.normal_index + 2],
      };

      auto it = uniq_verts.find(vert);
      if (it == uniq_verts.end()) {
        it = uniq_verts.insert({vert, static_cast<uint32_t>(uniq_verts.size())})
                 .first;
        mesh.vertices.push_back(vert);
      }
      mesh.indices.push_back(it->second);
    }
  }
  printf(
      "loaded %zd vertices, %zd indices\n", mesh.vertices.size(),
      mesh.indices.size());

  return std::move(mesh);
}
