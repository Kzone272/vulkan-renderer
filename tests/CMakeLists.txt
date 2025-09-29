add_executable(unit-tests
  aabb-tests.cc
  ../src/aabb.cc
)
target_compile_features(unit-tests PRIVATE cxx_std_23)

target_link_libraries(unit-tests PRIVATE
  GTest::gtest
  GTest::gtest_main
)

# Google Test specific CTest integration
include(GoogleTest)
gtest_discover_tests(unit-tests)


