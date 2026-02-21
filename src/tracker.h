#pragma once
#include <vector>
#include <cstdint>
#include <numeric>
#include "kalman.h"

// Track lifecycle config
struct TrackerConfig {
  // gating threshold (chi-square 2 dof)
  double gate_maha2 = 9.21;

  int max_misses = 8;

  // M-of-N confirmation
  int confirm_M = 3;
  int confirm_N = 5;

  // Track initiation (anti-clutter)
  double init_gate_dist = 12.0;
  int init_required_hits = 2;
  int init_max_age = 2;
  double init_vel_sigma = 40.0;

  // Association strategy
  bool use_hungarian = true;
};

struct Track {
  uint32_t id = 0;
  KalmanCV2D kf;

  int age = 0;
  int misses = 0;

  bool confirmed = false;
  double last_maha2 = 0.0;

  // hit history for M-of-N
  std::vector<uint8_t> hit_hist;

  Track(uint32_t id_, const KalmanCV2D& model, const Vec2& z_init, int confirm_N);
  int hits_in_window() const {
    int s = 0;
    for (uint8_t v : hit_hist) s += (v ? 1 : 0);
    return s;
  }
};

struct AssocResult {
  std::vector<int> track_to_meas; // size = tracks
  std::vector<int> meas_to_track; // size = meas
};

class MultiTargetTracker {
public:
  explicit MultiTargetTracker(TrackerConfig cfg) : cfg_(cfg) {}

  void step(const std::vector<Vec2>& measurements, double dt, double sigma_a, double sigma_z);

  const std::vector<Track>& tracks() const { return tracks_; }
  const std::vector<Vec2>& last_innovations() const { return last_innovs_; }
  const std::vector<Mat2>& last_S() const { return last_S_; }

private:
  struct Candidate {
    Vec2 z = Vec2::Zero();
    int hits = 0;
    int age = 0;
  };

  TrackerConfig cfg_;
  uint32_t next_id_ = 1;

  std::vector<Track> tracks_;
  std::vector<Vec2> last_innovs_;
  std::vector<Mat2> last_S_;

  // anti-clutter initiation candidates
  std::vector<Candidate> cands_;

  double maha2_for(const Track& t, const Vec2& z, Mat2* out_S, Vec2* out_innov);

  AssocResult associate(const std::vector<Vec2>& meas);
  AssocResult associate_greedy(const std::vector<Vec2>& meas);
  AssocResult associate_hungarian(const std::vector<Vec2>& meas);

  void initiate_from_unassigned_candidates(const std::vector<Vec2>& meas,
                                          const AssocResult& ar,
                                          double dt, double sigma_a, double sigma_z);

  void prune_and_confirm();
};