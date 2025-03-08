## Building

Based on the strategy described here: https://www.acarg.ch/posts/cmake-deps/

- To clone all submodules, run from root dir:
```
git submodule update --init
```

- Run from this dir for Debug builds:
```
cmake --preset Debug
cmake --build --preset Debug
cmake --install build/Debug --config Debug
```
And for Release builds:
```
cmake --preset Release
cmake --build --preset Release
cmake --install build/Release --config Release
```

This will build all deps into deps/build/(Debug|Release) dir and install them
locally in the deps/install/(Debug|Release) dir.
