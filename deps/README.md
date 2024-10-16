## Building

Based on the strategy described here: https://www.acarg.ch/posts/cmake-deps/

- To clone all submodules, run from root dir:
```
git submodule update --init
```

- Run from this dir:
```
cmake --preset Debug
cmake --build --preset Debug
cmake --install build/Debug --config Debug
```
Repeat the above replacing "Debug" with "Release" for release builds.

This will build all deps into deps/build/(Debug|Release) dir and install them
locally in the deps/install/(Debug|Release) dir.
