#include "kalman.h"

KalmanCV2D::KalmanCV2D(double dt_, double sigma_a_, double sigma_z_)
  : dt(dt_), sigma_a(sigma_a_), sigma_z(sigma_z_) {}

void KalmanCV2D::predict() {
  Mat4 F = Mat4::Identity();
  F(0,2) = dt;
  F(1,3) = dt;

  // Continuous white-noise acceleration model discretized
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  Mat4 Q = Mat4::Zero();
  const double q = sigma_a * sigma_a;
  Q(0,0) = dt4/4.0 * q; Q(0,2) = dt3/2.0 * q;
  Q(1,1) = dt4/4.0 * q; Q(1,3) = dt3/2.0 * q;
  Q(2,0) = dt3/2.0 * q; Q(2,2) = dt2 * q;
  Q(3,1) = dt3/2.0 * q; Q(3,3) = dt2 * q;

  x = F * x;
  P = F * P * F.transpose() + Q;
}

void KalmanCV2D::update(const Vec2& z, Vec2* out_innovation, Mat2* out_S) {
  Mat2x4 H;
  H.setZero();
  H(0,0) = 1.0;
  H(1,1) = 1.0;

  Mat2 R = Mat2::Identity() * (sigma_z * sigma_z);

  Vec2 y = z - (H * x); // innovation
  Mat2 S = H * P * H.transpose() + R;

  // Kalman gain
  Mat4x2 K = P * H.transpose() * S.inverse();

  x = x + K * y;
  Mat4 I = Mat4::Identity();
  P = (I - K * H) * P;

  if (out_innovation) *out_innovation = y;
  if (out_S) *out_S = S;
}