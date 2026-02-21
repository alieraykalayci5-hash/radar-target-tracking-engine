#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <cstdlib>

#include "sim.h"
#include "tracker.h"
#include "csv.h"
#include "fnv1a.h"

static bool arg_eq(const char* a, const char* b) {
  return std::string(a) == std::string(b);
}

static uint64_t parse_u64(const char* s) {
  return static_cast<uint64_t>(std::strtoull(s, nullptr, 10));
}

static int parse_i(const char* s) {
  return std::atoi(s);
}

static double parse_d(const char* s) {
  return std::atof(s);
}

int main(int argc, char** argv) {
  // Defaults (sane v1)
  uint64_t seed = 12345;
  int steps = 400;
  double dt = 0.05;

  int num_targets = 3;
  double sigma_z = 3.0;
  double p_detect = 0.90;

  // KF process model
  double sigma_a = 1.5;

  // gating/track management
  double gate_maha2 = 9.21;
  int confirm_hits = 3;
  int max_misses = 8;

  std::string out_dir = "out";

  for (int i = 1; i < argc; ++i) {
    if (arg_eq(argv[i], "--seed") && i + 1 < argc) seed = parse_u64(argv[++i]);
    else if (arg_eq(argv[i], "--steps") && i + 1 < argc) steps = parse_i(argv[++i]);
    else if (arg_eq(argv[i], "--dt") && i + 1 < argc) dt = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--targets") && i + 1 < argc) num_targets = parse_i(argv[++i]);
    else if (arg_eq(argv[i], "--sigma_z") && i + 1 < argc) sigma_z = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--p_detect") && i + 1 < argc) p_detect = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--sigma_a") && i + 1 < argc) sigma_a = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--gate_maha2") && i + 1 < argc) gate_maha2 = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--confirm_hits") && i + 1 < argc) confirm_hits = parse_i(argv[++i]);
    else if (arg_eq(argv[i], "--max_misses") && i + 1 < argc) max_misses = parse_i(argv[++i]);
    else if (arg_eq(argv[i], "--out") && i + 1 < argc) out_dir = argv[++i];
    else if (arg_eq(argv[i], "--help")) {
      std::cout <<
        "radar_tracker options:\n"
        "  --seed N\n"
        "  --steps N\n"
        "  --dt SEC\n"
        "  --targets N\n"
        "  --sigma_z METERS\n"
        "  --p_detect P\n"
        "  --sigma_a\n"
        "  --gate_maha2\n"
        "  --confirm_hits\n"
        "  --max_misses\n"
        "  --out DIR\n";
      return 0;
    }
  }

  std::filesystem::create_directories(out_dir);

  SimConfig scfg;
  scfg.num_targets = num_targets;
  scfg.dt = dt;
  scfg.steps = steps;
  scfg.sigma_z = sigma_z;
  scfg.p_detect = p_detect;

  TargetSim2D sim(seed, scfg);

  TrackerConfig tcfg;
  tcfg.gate_maha2 = gate_maha2;
  tcfg.confirm_hits = confirm_hits;
  tcfg.max_misses = max_misses;

  MultiTargetTracker tracker(tcfg);

  Csv truth_csv(out_dir + "/truth.csv");
  Csv meas_csv(out_dir + "/meas.csv");
  Csv tracks_csv(out_dir + "/tracks.csv");
  Csv resid_csv(out_dir + "/residuals.csv");

  truth_csv.header("step,true_id,x,y,vx,vy");
  meas_csv.header("step,true_id,zx,zy");
  tracks_csv.header("step,track_id,confirmed,x,y,vx,vy,misses,hits,maha2");
  resid_csv.header("step,track_id,innov_x,innov_y,S00,S01,S10,S11");

  Fnv1a64 fnv;
  fnv.add("RADAR_TRACKING_V1\n");
  fnv.add_u64(seed);

  for (int step = 0; step < steps; ++step) {
    sim.step();

    // log truth
    for (const auto& t : sim.truth()) {
      truth_csv.out << step << "," << t.id << ","
                    << std::setprecision(17) << t.pos.x() << ","
                    << std::setprecision(17) << t.pos.y() << ","
                    << std::setprecision(17) << t.vel.x() << ","
                    << std::setprecision(17) << t.vel.y() << "\n";
    }

    // collect measurements
    std::vector<Vec2> z;
    z.reserve(sim.last_measurements().size());
    for (const auto& m : sim.last_measurements()) {
      z.push_back(m.z);
      meas_csv.out << step << "," << m.true_id << ","
                   << std::setprecision(17) << m.z.x() << ","
                   << std::setprecision(17) << m.z.y() << "\n";
    }

    // tracker step
    tracker.step(z, dt, sigma_a, sigma_z);

    // log tracks + residuals
    const auto& tracks = tracker.tracks();
    const auto& innovs = tracker.last_innovations();
    const auto& Ss = tracker.last_S();

    for (size_t i = 0; i < tracks.size(); ++i) {
      const auto& tr = tracks[i];

      tracks_csv.out << step << "," << tr.id << "," << (tr.confirmed ? 1 : 0) << ","
                     << std::setprecision(17) << tr.kf.x(0) << ","
                     << std::setprecision(17) << tr.kf.x(1) << ","
                     << std::setprecision(17) << tr.kf.x(2) << ","
                     << std::setprecision(17) << tr.kf.x(3) << ","
                     << tr.misses << "," << tr.hits << ","
                     << std::setprecision(17) << tr.last_maha2
                     << "\n";

      resid_csv.out << step << "," << tr.id << ","
                    << std::setprecision(17) << innovs[i].x() << ","
                    << std::setprecision(17) << innovs[i].y() << ","
                    << std::setprecision(17) << Ss[i](0,0) << ","
                    << std::setprecision(17) << Ss[i](0,1) << ","
                    << std::setprecision(17) << Ss[i](1,0) << ","
                    << std::setprecision(17) << Ss[i](1,1)
                    << "\n";
    }

    // determinism hash: hash a subset of track output lines (stable)
    // (This is a "golden-style" quick check; later smoke.sh gibi script ekleyeceğiz.)
    for (const auto& tr : tracks) {
      // Quantization-free but text-based stable because we control precision.
      std::string line =
        std::to_string(step) + "," + std::to_string(tr.id) + "," + std::to_string(tr.confirmed ? 1 : 0) + "," +
        std::to_string(tr.kf.x(0)) + "," + std::to_string(tr.kf.x(1)) + "," +
        std::to_string(tr.kf.x(2)) + "," + std::to_string(tr.kf.x(3)) + "\n";
      fnv.add(line);
    }
  }

  std::cerr << "FNV1A64=" << std::hex << fnv.h << std::dec << "\n";
  std::cout << "Wrote logs to: " << out_dir << "\n";
  std::cout << "Files: truth.csv, meas.csv, tracks.csv, residuals.csv\n";
  return 0;
}