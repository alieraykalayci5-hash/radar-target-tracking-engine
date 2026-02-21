#include "tracker.h"
#include "hungarian.h"
#include <limits>
#include <algorithm>

Track::Track(uint32_t id_, const KalmanCV2D& model, const Vec2& z_init, int confirm_N)
  : id(id_), kf(model) {
  kf.x.setZero();
  kf.x(0) = z_init.x();
  kf.x(1) = z_init.y();
  kf.P = Mat4::Identity();

  hit_hist.assign(std::max(1, confirm_N), 0);
}

double MultiTargetTracker::maha2_for(const Track& t, const Vec2& z, Mat2* out_S, Vec2* out_innov) {
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

AssocResult MultiTargetTracker::associate(const std::vector<Vec2>& meas) {
  return cfg_.use_hungarian ? associate_hungarian(meas) : associate_greedy(meas);
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

AssocResult MultiTargetTracker::associate_hungarian(const std::vector<Vec2>& meas) {
  AssocResult ar;
  ar.track_to_meas.assign(tracks_.size(), -1);
  ar.meas_to_track.assign(meas.size(), -1);

  const int T = (int)tracks_.size();
  const int M = (int)meas.size();
  if (T == 0 || M == 0) return ar;

  // Build cost matrix = maha2, but gate-out becomes huge cost.
  // We'll allow unassigned by letting Hungarian pick expensive matches; we then post-filter by gate.
  const double BIG = 1e9;

  std::vector<std::vector<double>> cost((size_t)T, std::vector<double>((size_t)M, BIG));
  for (int ti = 0; ti < T; ++ti) {
    for (int mi = 0; mi < M; ++mi) {
      double m2 = maha2_for(tracks_[ti], meas[mi], nullptr, nullptr);
      if (m2 <= cfg_.gate_maha2) cost[ti][mi] = m2;
      else cost[ti][mi] = BIG;
    }
  }

  // Solve assignment (row=track -> col=measurement)
  std::vector<int> assign = hungarian_min_cost(cost);

  // Apply assignment with gate post-check (BIG means invalid)
  for (int ti = 0; ti < T; ++ti) {
    int mi = assign[ti];
    if (mi < 0 || mi >= M) continue;
    double c = cost[ti][mi];
    if (c >= BIG * 0.5) continue; // invalid
    if (ar.meas_to_track[mi] != -1) continue; // safety
    ar.track_to_meas[ti] = mi;
    ar.meas_to_track[mi] = ti;
    tracks_[ti].last_maha2 = c;
  }

  return ar;
}

void MultiTargetTracker::initiate_from_unassigned_candidates(const std::vector<Vec2>& meas,
                                                            const AssocResult& ar,
                                                            double dt, double sigma_a, double sigma_z) {
  const double gate2 = cfg_.init_gate_dist * cfg_.init_gate_dist;

  std::vector<char> cand_used(cands_.size(), 0);

  for (int mi = 0; mi < (int)meas.size(); ++mi) {
    if (ar.meas_to_track[mi] != -1) continue;

    const Vec2 z = meas[mi];

    int best_ci = -1;
    double best_d2 = std::numeric_limits<double>::infinity();

    for (int ci = 0; ci < (int)cands_.size(); ++ci) {
      if (cand_used[ci]) continue;
      const Vec2 d = z - cands_[ci].z;
      const double d2 = d.squaredNorm();
      if (d2 <= gate2 && d2 < best_d2) {
        best_d2 = d2;
        best_ci = ci;
      }
    }

    if (best_ci != -1) {
      cand_used[best_ci] = 1;
      cands_[best_ci].z = z;
      cands_[best_ci].hits += 1;
      cands_[best_ci].age = 0;
    } else {
      Candidate c;
      c.z = z;
      c.hits = 1;
      c.age = 0;
      cands_.push_back(c);
      cand_used.push_back(1);
    }
  }

  for (int ci = 0; ci < (int)cands_.size(); ++ci) {
    if (!cand_used[ci]) cands_[ci].age += 1;
  }

  cands_.erase(std::remove_if(cands_.begin(), cands_.end(), [&](const Candidate& c){
    return c.age > cfg_.init_max_age;
  }), cands_.end());

  KalmanCV2D model(dt, sigma_a, sigma_z);

  std::vector<Candidate> keep;
  keep.reserve(cands_.size());

  for (const auto& c : cands_) {
    if (c.hits >= cfg_.init_required_hits) {
      Track t(next_id_++, model, c.z, cfg_.confirm_N);

      t.kf.P.setZero();
      t.kf.P(0,0) = sigma_z*sigma_z;
      t.kf.P(1,1) = sigma_z*sigma_z;
      t.kf.P(2,2) = cfg_.init_vel_sigma * cfg_.init_vel_sigma;
      t.kf.P(3,3) = cfg_.init_vel_sigma * cfg_.init_vel_sigma;

      t.age = 1;
      t.misses = 0;

      for (int i = 0; i < (int)t.hit_hist.size() && i < c.hits; ++i) t.hit_hist[i] = 1;

      t.confirmed = (t.hits_in_window() >= cfg_.confirm_M);
      tracks_.push_back(t);
    } else {
      keep.push_back(c);
    }
  }

  cands_.swap(keep);
}

void MultiTargetTracker::prune_and_confirm() {
  for (auto& t : tracks_) {
    t.confirmed = (t.hits_in_window() >= cfg_.confirm_M);
  }

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
    t.last_maha2 = 0.0;
  }

  // 2) association (greedy or hungarian)
  AssocResult ar = associate(measurements);

  last_innovs_.assign(tracks_.size(), Vec2::Zero());
  last_S_.assign(tracks_.size(), Mat2::Zero());

  // 3) update associated tracks
  for (int ti = 0; ti < (int)tracks_.size(); ++ti) {
    int mi = ar.track_to_meas[ti];

    // slide hit window
    if (!tracks_[ti].hit_hist.empty()) {
      std::rotate(tracks_[ti].hit_hist.begin(), tracks_[ti].hit_hist.begin() + 1, tracks_[ti].hit_hist.end());
      tracks_[ti].hit_hist.back() = (mi != -1) ? 1 : 0;
    }

    if (mi == -1) {
      tracks_[ti].misses += 1;
      continue;
    }

    Vec2 innov;
    Mat2 S;
    tracks_[ti].kf.update(measurements[mi], &innov, &S);

    last_innovs_[ti] = innov;
    last_S_[ti] = S;

    tracks_[ti].misses = 0;
  }

  // 4) initiate via candidates
  const size_t before_tracks = tracks_.size();
  initiate_from_unassigned_candidates(measurements, ar, dt, sigma_a, sigma_z);

  if (tracks_.size() > before_tracks) {
    last_innovs_.resize(tracks_.size(), Vec2::Zero());
    last_S_.resize(tracks_.size(), Mat2::Zero());
  }

  // 5) confirm + prune
  prune_and_confirm();
}