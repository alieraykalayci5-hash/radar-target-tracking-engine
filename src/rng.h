#pragma once
#include <random>
#include <cstdint>

struct Rng {
  std::mt19937_64 eng;

  explicit Rng(uint64_t seed) : eng(seed) {}

  // [0,1)
  double uniform01() {
    std::uniform_real_distribution<double> d(0.0, 1.0);
    return d(eng);
  }

  // [a,b)
  double uniform(double a, double b) {
    std::uniform_real_distribution<double> d(a, b);
    return d(eng);
  }

  // Normal(mu, sigma)
  double normal(double mu, double sigma) {
    std::normal_distribution<double> d(mu, sigma);
    return d(eng);
  }

  // Integer in [a,b]
  int uniform_int(int a, int b) {
    std::uniform_int_distribution<int> d(a, b);
    return d(eng);
  }
};