#include <gtest/gtest.h>

#include <format>

#include "../src/aabb.h"
#include "../src/glm-include.h"

TEST(AabbTests, merge) {
  AABB x(vec3(0), vec3(1));
  AABB y(vec3(0), vec3(2));

  auto merged = x.merge(y);
  auto merged2 = y.merge(x);

  EXPECT_EQ(merged, merged2);
  EXPECT_EQ(merged.min_, vec3(0));
  EXPECT_EQ(merged.max_, vec3(2));
  EXPECT_EQ(merged, AABB(vec3(0), vec3(2)));

  AABB z(vec3(-1), vec3(0));
  merged = x.merge(z);

  EXPECT_EQ(merged, AABB(vec3(-1), vec3(1)));

  AABB neg(vec3(-3), vec3(-2));
  merged = z.merge(neg);

  EXPECT_EQ(merged, AABB(vec3(-3), vec3(0)));

  AABB mixed(vec3(1, 3, 5), vec3(2, 4, 6));
  AABB mixed2(vec3(0, 5, 3), vec3(1, 6, 4));
  merged = mixed.merge(mixed2);

  EXPECT_EQ(merged, AABB(vec3(0, 3, 3), vec3(2, 6, 6)));
}

void expectVecNear(const vec3& a, const vec3& b, float epsilon = 1e-3) {
  SCOPED_TRACE(std::format("a: {} b: {}", toStr(a), toStr(b)));
  EXPECT_NEAR(a.x, b.x, epsilon);
  EXPECT_NEAR(a.y, b.y, epsilon);
  EXPECT_NEAR(a.z, b.z, epsilon);
}

TEST(AabbTests, transform) {
  AABB x(vec3(-1), vec3(1));
  mat4 scale = glm::scale(vec3(2));

  auto trans = x.transform(scale);

  EXPECT_EQ(trans, AABB(vec3(-2), vec3(2)));

  mat4 rotate = glm::toMat4(glm::angleAxis(glm::radians(45.f), vec3(0, 1, 0)));
  trans = x.transform(rotate);

  expectVecNear(trans.min_, vec3(-1.414, -1, -1.414));
  expectVecNear(trans.max_, vec3(1.414, 1, 1.414));

  mat4 move = glm::translate(vec3(-3, 2, 5));
  trans = x.transform(move);

  expectVecNear(trans.min_, vec3(-4, 1, 4));
  expectVecNear(trans.max_, vec3(-2, 3, 6));
  
  trans = x.transform(move * rotate * scale);

  expectVecNear(trans.min_, vec3(-5.828, 0, 2.172));
  expectVecNear(trans.max_, vec3(-0.172, 4, 7.828));
}
