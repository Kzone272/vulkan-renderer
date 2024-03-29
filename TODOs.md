Things I Should Do Soon:

Short-term ideas:
- fbo.output_set isn't useful if the input set wants more than a single sampler
  - What I really need is to provide a set of ImageViews/infos to an input
    pipeline for it to create the sets with it's layout.
    - This is actually pretty similar to what making Materials would need.
- Speed up build
  - Add RenderInterface to move renderer.h out of app.h
  - make app.cc
- Simplify creating Ubo DynamicBufs and attaching to descriptors.
  - Not totally clear what that interface should look like.
- Simplify adding new shaders. Probably stored in a map keyed by filename.
- Add shadow mapping for some lights.
- Improve outline filter AA by rendering edges with MSAA.
- Choose voronoi points with poisson disk sampling.
- Fix memory access rendering bug when maximizing window
  - May have crept in when using Fbo for swaps? -> nope.
- Reconsider when we set image and buffer infos
- Add hand forward offset
- Adjust position of in-swing step if velocity changes significantly
- Make prettier
  - Add scene selector
  - Setup cube map BG
  - Setup blinn-phong rendering
- Use obj materials info to allow multiple materials per "model"
- Catmull rom splines

Medium-term ideas:
- Fix Mac build
- Try doing PBR rendering
- Setup a compute shader

Long-term ideas:
- Physics
