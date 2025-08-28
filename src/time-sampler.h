#pragma once

class TimeSampler {
 public:
  void addTime(float time) {
    total_ += time;
    count_++;
  }

  float avg() {
    return total_ / count_;
  }

  void reset() {
    total_ = 0;
    count_ = 0;
  }

 private:
  float total_ = 0;
  int count_ = 0;
};
