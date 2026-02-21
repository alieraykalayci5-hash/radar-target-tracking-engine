#pragma once
#include <cstdint>
#include <string_view>

struct Fnv1a64 {
  uint64_t h = 14695981039346656037ull;

  void add_byte(uint8_t b) {
    h ^= b;
    h *= 1099511628211ull;
  }

  void add(std::string_view sv) {
    for (unsigned char c : sv) add_byte(static_cast<uint8_t>(c));
  }

  void add_u64(uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      add_byte(static_cast<uint8_t>((v >> (i * 8)) & 0xFF));
    }
  }
};