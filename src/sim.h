#pragma once
#include <vector>
#include <cstdint>
#include "math_types.h"
#include "rng.h"

struct TrueTarget {
  uint32_t id;
  Vec2 pos;
  Vec2 vel;
};

struct SimConfig {
  int num_targets = 3;
  double dt = 0.05;
  int steps = 400;

  // measurement model
  double sigma_z = 3.0;     // position measurement std (m)
  double p_detect = 0.90;   // detection probability (miss logic)

  // initial world
  double spawn_radius = 200.0;
  double speed_min = 5.0;
  double speed_max = 20.0;
};

struct Measurement {
  int step;
  uint32_t true_id; // 0 if unknown/clutter (we won’t generate clutter in v1)
  Vec2 z;
};

class TargetSim2D {
public:
  TargetSim2D(uint64_t seed, SimConfig cfg);

  void step();

  int step_index() const { return step_; }
  const std::vector<TrueTarget>& truth() const { return truth_; }
  const std::vector<Measurement>& last_measurements() const { return meas_; }

private:
  Rng rng_;
  SimConfig cfg_;
  int step_ = 0;
  std::vector<TrueTarget> truth_;
  std::vector<Measurement> meas_;

  void init_targets();
};