#pragma once
#include <vector>
#include <cstdint>
#include "math_types.h"
#include "rng.h"

struct TruthTarget {
  int id = 0;
  Vec2 pos = Vec2::Zero();
  Vec2 vel = Vec2::Zero();
};

struct Measurement {
  int true_id = 0;   // 0 = clutter / false alarm
  Vec2 z = Vec2::Zero();
};

struct SimConfig {
  int num_targets = 3;
  double dt = 0.05;
  int steps = 400;

  double sigma_z = 3.0;
  double p_detect = 0.90;

  bool enable_clutter = true;
  int clutter_per_step = 6;
  double clutter_area_half = 300.0;

  // scenario selection
  bool scenario_cross = false;
};

class TargetSim2D {
public:
  TargetSim2D(uint64_t seed, const SimConfig& cfg);

  void step();

  const std::vector<TruthTarget>& truth() const { return truth_; }
  const std::vector<Measurement>& last_measurements() const { return last_meas_; }

private:
  SimConfig cfg_;
  Rng rng_;
  int step_idx_ = 0;

  std::vector<TruthTarget> truth_;
  std::vector<Measurement> last_meas_;

  void init_random();
  void init_cross();
  void gen_measurements();
};