#include "asserts.h"

#include <exception>
#include <iostream>

#ifdef __cpp_lib_stacktrace
#include <stacktrace>
#endif

#include "utils.h"

void ASSERTfn(bool x, std::string assertion) {
  if (!(x)) {
    auto str = strFmt("ASSERT(%s) failed!", assertion.c_str());
    printf("%s\n", str.c_str());

#ifdef __cpp_lib_stacktrace
    std::cout << std::stacktrace::current() << '\n';
#else
    printf("Can't print stacktrace :/\n");
#endif

    throw std::runtime_error(str);
  }
}
