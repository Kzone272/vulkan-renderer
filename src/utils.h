#pragma once

#include <iostream>
#include <Eigen/Dense>

namespace utils {

using Eigen::MatrixXd;

// Only exists to prove the Eigen works.
void PrintMatrix() {
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
}

}  // namespace utils
