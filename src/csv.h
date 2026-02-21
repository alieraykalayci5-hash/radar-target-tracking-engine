#pragma once
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

struct Csv {
  std::ofstream out;

  explicit Csv(const std::string& path) : out(path, std::ios::binary) {}

  void header(const std::string& h) {
    out << h << "\n";
  }

  template <typename... Ts>
  void row(const Ts&... xs) {
    bool first = true;
    auto emit = [&](const auto& v) {
      if (!first) out << ",";
      first = false;
      out << v;
    };
    (emit(xs), ...);
    out << "\n";
  }

  void row2d(int step, uint32_t id, double x, double y) {
    out << step << "," << id << ","
        << std::setprecision(17) << x << ","
        << std::setprecision(17) << y << "\n";
  }
};