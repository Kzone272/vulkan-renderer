Vulkan Renderer
==================

Trying to learn some Vulkan, and implement some cool renering techniques while I'm at it.
I started by following along with vulkan-tutorial.com and have diverged from there.

Video of procedural walk cycle: https://youtu.be/W4CaiJ9Zog8

*Latest screenshot:*
![](screenshot6.png)

### Building

- Fetch dependencies with:
```
git submodule update --init
```
- Build all deps (see deps/README.md)

- From this directory, configure using one of the provided presets:
```
cmake --preset Windows-Debug
cmake --preset Mac-Debug
```
Then build:
```
cmake --build build
```

Currently builds on MSVC >= 1937 on Windows, and Clang >= 17 on Mac.

### Dependencies

These should be installed using instructions in deps/README.md, or globally installed:
- VulkanSDK @ 1.3.290.0
- sdl2 @ 2.28.0
- SDL2_image[core,libjpeg-turbo] @ 2.6.3
- glm @ 0.9.9.9
- tinyobjloader @ 2.0.0-rc10
- imgui @ 1.89.6

Exact versions may not matter, but this is what I'm using.
