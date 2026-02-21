#include "sim.h"
#include <cmath>

static Vec2 polar(double r, double ang) {
  return Vec2(r * std::cos(ang), r * std::sin(ang));
}

TargetSim2D::TargetSim2D(uint64_t seed, SimConfig cfg)
  : rng_(seed), cfg_(cfg) {
  init_targets();
}

void TargetSim2D::init_targets() {
  truth_.clear();
  truth_.reserve(cfg_.num_targets);

  for (int i = 0; i < cfg_.num_targets; ++i) {
    const double ang = 2.0 * 3.14159265358979323846 * rng_.uniform01();
    const double r = cfg_.spawn_radius * (0.3 + 0.7 * rng_.uniform01());
    Vec2 p = polar(r, ang);

    const double dir = 2.0 * 3.14159265358979323846 * rng_.uniform01();
    const double sp = cfg_.speed_min + (cfg_.speed_max - cfg_.speed_min) * rng_.uniform01();
    Vec2 v = polar(sp, dir);

    TrueTarget t;
    t.id = static_cast<uint32_t>(i + 1);
    t.pos = p;
    t.vel = v;
    truth_.push_back(t);
  }
}

void TargetSim2D::step() {
  meas_.clear();

  // propagate truth (constant velocity)
  for (auto& t : truth_) {
    t.pos += t.vel * cfg_.dt;

    // detection
    const double u = rng_.uniform01();
    if (u > cfg_.p_detect) {
      continue; // missed detection
    }

    // radar "position measurement" with gaussian noise
    Vec2 z;
    z.x() = rng_.normal(t.pos.x(), cfg_.sigma_z);
    z.y() = rng_.normal(t.pos.y(), cfg_.sigma_z);

    Measurement m;
    m.step = step_;
    m.true_id = t.id;
    m.z = z;
    meas_.push_back(m);
  }

  step_ += 1;
}