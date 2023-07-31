#pragma once

#include <cstdio>
#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>

// Snippet source:
// https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
template <typename... Args>
std::string strFmt(const std::string& format, Args... args) {
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) +
               1;  // Extra space for '\0'
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(
      buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}

template <class... Args>
void log(const std::format_string<Args...> fmt, Args&&... args) {
  std::cout << std::format(fmt, std::forward<Args>(args)...) << std::endl;
}
