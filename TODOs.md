Things I Should Do Soon:

Short-term ideas:
- Make prettier
  - Add shadow mapping for directional lights
  - Add cascaded shadow mapping
  - Add screen-space gradient material
  - Use reverse-z depth buffer for better accuracy
  - Setup cube map for background and indirect lighting
  - Normal maps
  - Ambient occlusion shader?
  - HDR + tonemapping?
  - Bloom shader
- Rendering performance
  - Per-draw uniforms in an actual buffer
  - Support instance rendering
  - Support draw indirection. One draw call per material maybe?
  - Consider interleaved vertex buffers
  - Consider bindless descriptors
  - Forward+ / tiled forward rendering
- Improve asset loading
  - Use some library? assimp?
  - Import complex meshes with mutltiple materials and textures
  - Import animation rigs with skinned meshes
- Add quick and simple debug rendering
- Improve walk animation
  - Adjust position of in-swing step if velocity changes significantly
- Add catmull rom splines
- Choose voronoi points with poisson disk sampling

Medium-term ideas:
- Collisions, physics
- Clicking/selection, probably based on ray cast collider intersections
- Try doing PBR rendering
- Setup a compute shader
- Add scene selector
- Vulkan refactors
  - Consider separating samplers from images in shaders and VkDescriptorImageInfos
    - Simpler decision when creating descriptors for images and samplers
    - Less combinatorial explosion when different shaders want different samplers
  - I think passes should own their descriptor set inputs
    - provide a set of ImageViews/infos as inputs to a pass or pipeline

Long-term ideas:
- Speed up build
  - Maybe add RenderInterface to move renderer.h out of app.h
