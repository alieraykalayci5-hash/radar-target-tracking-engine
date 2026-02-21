#pragma once
#include <vector>
#include <cstdint>
#include "kalman.h"

struct Track {
  uint32_t id = 0;
  KalmanCV2D kf;
  int age = 0;          // total steps alive
  int hits = 0;         // measurement updates
  int misses = 0;       // consecutive misses
  bool confirmed = false;

  // last association stats
  double last_maha2 = 0.0;

  Track(uint32_t id_, const KalmanCV2D& model, const Vec2& z_init);
};

struct TrackerConfig {
  double gate_maha2 = 9.21; // ~ chi-square 2DOF, 99% ~= 9.21
  int confirm_hits = 3;     // hits required to confirm
  int max_misses = 8;       // drop after this many consecutive misses
  double init_vel_sigma = 30.0; // initial velocity uncertainty
};

struct AssocResult {
  // index of measurement assigned to each track (-1 if none)
  std::vector<int> track_to_meas;
  // track index assigned to each measurement (-1 if none)
  std::vector<int> meas_to_track;
};

class MultiTargetTracker {
public:
  explicit MultiTargetTracker(TrackerConfig cfg) : cfg_(cfg) {}

  void step(const std::vector<Vec2>& measurements, double dt, double sigma_a, double sigma_z);

  const std::vector<Track>& tracks() const { return tracks_; }

  // diagnostics
  const std::vector<Vec2>& last_innovations() const { return last_innovs_; }
  const std::vector<Mat2>& last_S() const { return last_S_; }

private:
  TrackerConfig cfg_;
  uint32_t next_id_ = 1;
  std::vector<Track> tracks_;

  // last step logging vectors aligned with tracks_ after update
  std::vector<Vec2> last_innovs_;
  std::vector<Mat2> last_S_;

  static double maha2_for(const Track& t, const Vec2& z, Mat2* out_S, Vec2* out_innov);
  AssocResult associate_greedy(const std::vector<Vec2>& meas);

  void initiate_from_unassigned(const std::vector<Vec2>& meas, const AssocResult& ar,
                                double dt, double sigma_a, double sigma_z);
  void prune_and_confirm();
};