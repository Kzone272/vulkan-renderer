#include "asserts.h"

#include <exception>
#include <print>

#ifdef __cpp_lib_stacktrace
#include <stacktrace>
#endif

void ASSERTfn(bool x, std::string assertion) {
  if (!(x)) {
    std::string error_str = std::format("ASSERT({}) failed!", assertion);
    std::println("{}", error_str);

#ifdef __cpp_lib_stacktrace
    std::println("{}", std::stacktrace::current().to_string());
#else
    std::println("Can't print stacktrace :/");
#endif

    throw std::runtime_error(error_str);
  }
}
