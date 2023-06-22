#include "asserts.h"

#include <boost/stacktrace.hpp>
#include <exception>
#include <iostream>

#include "utils.h"

void ASSERT(bool x) {
  if (!(x)) {
    auto str = strFmt("Assertion failed! Printing stack trace:");
    printf("%s\n", str.c_str());
    std::cout << boost::stacktrace::stacktrace();
    throw std::runtime_error(str);
  }
}
