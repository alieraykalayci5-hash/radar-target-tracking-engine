#include "tracker.h"
#include <limits>
#include <algorithm>

Track::Track(uint32_t id_, const KalmanCV2D& model, const Vec2& z_init)
  : id(id_), kf(model) {
  kf.x.setZero();
  kf.x(0) = z_init.x();
  kf.x(1) = z_init.y();
  // vx, vy start at 0
  kf.P = Mat4::Identity();
}

double MultiTargetTracker::maha2_for(const Track& t, const Vec2& z, Mat2* out_S, Vec2* out_innov) {
  // Build H and S same as in KF update
  Mat2x4 H;
  H.setZero();
  H(0,0) = 1.0; H(1,1) = 1.0;

  Mat2 R = Mat2::Identity() * (t.kf.sigma_z * t.kf.sigma_z);

  Vec2 innov = z - (H * t.kf.x);
  Mat2 S = H * t.kf.P * H.transpose() + R;

  const double m2 = innov.transpose() * S.inverse() * innov;
  if (out_S) *out_S = S;
  if (out_innov) *out_innov = innov;
  return m2;
}

AssocResult MultiTargetTracker::associate_greedy(const std::vector<Vec2>& meas) {
  AssocResult ar;
  ar.track_to_meas.assign(tracks_.size(), -1);
  ar.meas_to_track.assign(meas.size(), -1);

  struct Edge { int ti; int mi; double m2; };
  std::vector<Edge> edges;
  edges.reserve(tracks_.size() * meas.size());

  for (int ti = 0; ti < (int)tracks_.size(); ++ti) {
    for (int mi = 0; mi < (int)meas.size(); ++mi) {
      double m2 = maha2_for(tracks_[ti], meas[mi], nullptr, nullptr);
      if (m2 <= cfg_.gate_maha2) {
        edges.push_back({ti, mi, m2});
      }
    }
  }

  std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b){
    return a.m2 < b.m2;
  });

  for (const auto& e : edges) {
    if (ar.track_to_meas[e.ti] != -1) continue;
    if (ar.meas_to_track[e.mi] != -1) continue;
    ar.track_to_meas[e.ti] = e.mi;
    ar.meas_to_track[e.mi] = e.ti;
    tracks_[e.ti].last_maha2 = e.m2;
  }

  return ar;
}

void MultiTargetTracker::initiate_from_unassigned(const std::vector<Vec2>& meas,
                                                  const AssocResult& ar,
                                                  double dt, double sigma_a, double sigma_z) {
  KalmanCV2D model(dt, sigma_a, sigma_z);

  for (int mi = 0; mi < (int)meas.size(); ++mi) {
    if (ar.meas_to_track[mi] != -1) continue;

    Track t(next_id_++, model, meas[mi]);

    // Large initial uncertainty in velocity, smaller in position
    t.kf.P.setZero();
    t.kf.P(0,0) = sigma_z*sigma_z;
    t.kf.P(1,1) = sigma_z*sigma_z;
    t.kf.P(2,2) = cfg_.init_vel_sigma * cfg_.init_vel_sigma;
    t.kf.P(3,3) = cfg_.init_vel_sigma * cfg_.init_vel_sigma;

    t.age = 1;
    t.hits = 1;     // we spawned it from a measurement
    t.misses = 0;
    t.confirmed = (t.hits >= cfg_.confirm_hits);
    tracks_.push_back(t);
  }
}

void MultiTargetTracker::prune_and_confirm() {
  // confirm logic already updated in step; prune dead tracks
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& t){
    return t.misses > cfg_.max_misses;
  }), tracks_.end());
}

void MultiTargetTracker::step(const std::vector<Vec2>& measurements, double dt, double sigma_a, double sigma_z) {
  // 1) predict all
  for (auto& t : tracks_) {
    t.kf.dt = dt;
    t.kf.sigma_a = sigma_a;
    t.kf.sigma_z = sigma_z;
    t.kf.predict();
    t.age += 1;
    // clear last_maha2 if no assoc
    t.last_maha2 = 0.0;
  }

  // 2) association
  AssocResult ar = associate_greedy(measurements);

  // Prepare last innovations aligned with tracks_ (before any new tracks are spawned)
  last_innovs_.assign(tracks_.size(), Vec2::Zero());
  last_S_.assign(tracks_.size(), Mat2::Zero());

  // 3) update associated tracks
  for (int ti = 0; ti < (int)tracks_.size(); ++ti) {
    int mi = ar.track_to_meas[ti];
    if (mi == -1) {
      tracks_[ti].misses += 1;
      continue;
    }

    Vec2 innov;
    Mat2 S;
    tracks_[ti].kf.update(measurements[mi], &innov, &S);

    last_innovs_[ti] = innov;
    last_S_[ti] = S;

    tracks_[ti].hits += 1;
    tracks_[ti].misses = 0;
    tracks_[ti].confirmed = (tracks_[ti].hits >= cfg_.confirm_hits);
  }

  // 4) initiate new tracks for unassigned measurements
  const size_t before_tracks = tracks_.size();
  initiate_from_unassigned(measurements, ar, dt, sigma_a, sigma_z);

  // Extend diagnostics arrays for newly created tracks (prevents out-of-range in main logging)
  if (tracks_.size() > before_tracks) {
    last_innovs_.resize(tracks_.size(), Vec2::Zero());
    last_S_.resize(tracks_.size(), Mat2::Zero());
  }

  // 5) prune dead
  prune_and_confirm();
}