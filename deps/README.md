## Building

Based on the strategy described here: https://www.acarg.ch/posts/cmake-deps/

- To clone all submodules, run from root dir:
```
git submodule update --init
```

- Run from this dir:
```
cmake --preset default
cmake --build --preset debug
cmake --install build
```

This will build all deps into deps/build dir and install them locally in the
deps/install dir.
