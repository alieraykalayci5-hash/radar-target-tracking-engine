#pragma once
#include <cstdint>
#include <cmath>
#include <limits>

struct Rng {
  uint64_t s;

  explicit Rng(uint64_t seed) : s(seed ? seed : 0x9E3779B97F4A7C15ull) {}

  // xorshift64*
  uint64_t next_u64() {
    uint64_t x = s;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    s = x;
    return x * 2685821657736338717ull;
  }

  // [0,1)
  double uniform01() {
    // 53-bit precision
    const uint64_t u = next_u64();
    const uint64_t mant = (u >> 11);
    return static_cast<double>(mant) * (1.0 / 9007199254740992.0); // 2^53
  }

  // Standard normal N(0,1) via Box-Muller
  double normal01() {
    // Avoid log(0)
    double u1 = uniform01();
    double u2 = uniform01();
    if (u1 < 1e-15) u1 = 1e-15;
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double th = 2.0 * 3.14159265358979323846 * u2;
    return r * std::cos(th);
  }

  // N(mean, std)
  double normal(double mean, double stddev) {
    return mean + stddev * normal01();
  }
};