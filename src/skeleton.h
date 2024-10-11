#pragma once

#include <vector>

#include "glm-include.h"

constexpr size_t kNoParent = -1;

class Skeleton {
 public:
  Skeleton(size_t count) : count_(count) {
  }

  inline void addBone(size_t b, size_t p) {
    DASSERT(b == parents_.size());
    parents_.push_back(p);
  }

  inline size_t parent(size_t i) {
    return parents_[i];
  }

  inline size_t count() const {
    return count_;
  }

 private:
  size_t count_;
  std::vector<size_t> parents_;
};
