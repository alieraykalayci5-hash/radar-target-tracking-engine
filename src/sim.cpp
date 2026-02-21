#include "sim.h"

TargetSim2D::TargetSim2D(uint64_t seed, const SimConfig& cfg)
  : cfg_(cfg), rng_(seed) {
  step_idx_ = 0;
  truth_.clear();
  last_meas_.clear();

  if (cfg_.scenario_cross) init_cross();
  else init_random();
}

void TargetSim2D::init_random() {
  truth_.reserve(cfg_.num_targets);
  for (int i = 0; i < cfg_.num_targets; ++i) {
    TruthTarget t;
    t.id = i + 1;
    t.pos = Vec2(rng_.uniform(-120.0, 120.0), rng_.uniform(-120.0, 120.0));
    t.vel = Vec2(rng_.uniform(-8.0, 8.0), rng_.uniform(-8.0, 8.0));
    truth_.push_back(t);
  }
}

void TargetSim2D::init_cross() {
  // Two targets cross near origin to create association ambiguity.
  truth_.reserve(2);

  TruthTarget a;
  a.id = 1;
  a.pos = Vec2(-80.0, 0.0);
  a.vel = Vec2(+6.0, 0.0);

  TruthTarget b;
  b.id = 2;
  b.pos = Vec2(+80.0, 0.0);
  b.vel = Vec2(-6.0, 0.0);

  truth_.push_back(a);
  truth_.push_back(b);
}

void TargetSim2D::gen_measurements() {
  last_meas_.clear();

  // true detections
  for (const auto& t : truth_) {
    if (rng_.uniform01() > cfg_.p_detect) continue;

    Measurement m;
    m.true_id = t.id;
    const double nx = rng_.normal(0.0, cfg_.sigma_z);
    const double ny = rng_.normal(0.0, cfg_.sigma_z);
    m.z = t.pos + Vec2(nx, ny);
    last_meas_.push_back(m);
  }

  // clutter
  if (cfg_.enable_clutter) {
    for (int i = 0; i < cfg_.clutter_per_step; ++i) {
      Measurement m;
      m.true_id = 0;
      const double x = rng_.uniform(-cfg_.clutter_area_half, cfg_.clutter_area_half);
      const double y = rng_.uniform(-cfg_.clutter_area_half, cfg_.clutter_area_half);
      m.z = Vec2(x, y);
      last_meas_.push_back(m);
    }
  }
}

void TargetSim2D::step() {
  for (auto& t : truth_) {
    t.pos = t.pos + t.vel * cfg_.dt;
  }
  gen_measurements();
  step_idx_++;
}