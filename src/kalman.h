#pragma once
#include "math_types.h"

struct KalmanCV2D {
  // State: [x, y, vx, vy]
  Vec4 x = Vec4::Zero();
  Mat4 P = Mat4::Identity();

  double dt = 0.05;

  // Process noise (acceleration)
  double sigma_a = 1.5;

  // Measurement noise (position)
  double sigma_z = 3.0;

  KalmanCV2D() = default;
  KalmanCV2D(double dt_, double sigma_a_, double sigma_z_);

  void predict();
  // z = [x_meas, y_meas]
  void update(const Vec2& z, Vec2* out_innovation = nullptr, Mat2* out_S = nullptr);
};