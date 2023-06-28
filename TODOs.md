Things I Should Do Soon:

Short-term ideas:
- Render multiple objects
  - Changing position per object probably needs push constants
  - Move model matrix to push constant
  - Make UBO contain single view_proj matrix
- Render objects with different textures
  - Multiple ways this could be done
  - I like the idea of having a DescriptorSet per texture
  - bind that texture descriptor set per model/material

Medium-term ideas:
- Setup imgui
- Setup a compute shader
- Setup blinn-phong rendering

Long-term ideas:
- Setup cube map BG
- Try doing PBR rendering
