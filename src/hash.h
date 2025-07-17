#pragma once

// Hashing method from:
// https://stackoverflow.com/questions/16340/how-do-i-generate-a-hashcode-from-a-byte-array-in-c

inline uint32_t hashBytes(void* data, size_t size_bytes) {
  char* byteData = static_cast<char*>(data);
  uint32_t hash = 2166136261;
  for (size_t i = 0; i < size_bytes; i++) {
    hash = (hash * 16777619) ^ byteData[i];
  }
  return hash;
}

inline uint32_t hashBytes(auto span) {
  return hashBytes(span.data(), span.size_bytes());
}
