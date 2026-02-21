#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <cstdlib>
#include <iomanip>
#include <cstdint>
#include <algorithm>
#include <chrono>

#include "sim.h"
#include "tracker.h"
#include "csv.h"
#include "fnv1a.h"
#include "hungarian.h"

static bool arg_eq(const char* a, const char* b) { return std::string(a) == std::string(b); }
static uint64_t parse_u64(const char* s) { return static_cast<uint64_t>(std::strtoull(s, nullptr, 10)); }
static int parse_i(const char* s) { return std::atoi(s); }
static double parse_d(const char* s) { return std::atof(s); }
static int parse_b(const char* s) { return std::atoi(s) ? 1 : 0; }

static std::vector<int> greedy_min_cost(const std::vector<std::vector<double>>& cost) {
  const int T = (int)cost.size();
  const int M = (T > 0) ? (int)cost[0].size() : 0;
  std::vector<int> row_to_col(T, -1);
  if (T == 0 || M == 0) return row_to_col;

  struct Edge { int r; int c; double w; };
  std::vector<Edge> edges;
  edges.reserve((size_t)T * (size_t)M);

  for (int r = 0; r < T; ++r)
    for (int c = 0; c < M; ++c)
      edges.push_back({r, c, cost[r][c]});

  std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b){
    if (a.w != b.w) return a.w < b.w;
    if (a.r != b.r) return a.r < b.r;
    return a.c < b.c;
  });

  std::vector<int> col_used(M, 0);
  for (const auto& e : edges) {
    if (row_to_col[e.r] != -1) continue;
    if (col_used[e.c]) continue;
    row_to_col[e.r] = e.c;
    col_used[e.c] = 1;
  }
  return row_to_col;
}

static double assignment_cost(const std::vector<std::vector<double>>& cost, const std::vector<int>& a) {
  double s = 0.0;
  for (int r = 0; r < (int)a.size(); ++r) {
    int c = a[r];
    if (c >= 0 && c < (int)cost[r].size()) s += cost[r][c];
  }
  return s;
}

static void run_assoc_demo() {
  std::vector<std::vector<double>> cost = {
    {1.0, 2.0},
    {2.0, 100.0}
  };

  auto g = greedy_min_cost(cost);
  auto h = hungarian_min_cost(cost);

  std::cout << "=== ASSOC DEMO (Greedy vs Hungarian) ===\n";
  std::cout << "cost matrix:\n";
  for (int r = 0; r < (int)cost.size(); ++r) {
    std::cout << "  row " << r << ": ";
    for (int c = 0; c < (int)cost[r].size(); ++c) {
      std::cout << cost[r][c] << (c + 1 < (int)cost[r].size() ? ", " : "");
    }
    std::cout << "\n";
  }

  std::cout << "greedy assignment: ";
  for (int r = 0; r < (int)g.size(); ++r) std::cout << r << "->" << g[r] << (r + 1 < (int)g.size() ? ", " : "");
  std::cout << "  total_cost=" << assignment_cost(cost, g) << "\n";

  std::cout << "hungarian assignment: ";
  for (int r = 0; r < (int)h.size(); ++r) std::cout << r << "->" << h[r] << (r + 1 < (int)h.size() ? ", " : "");
  std::cout << "  total_cost=" << assignment_cost(cost, h) << "\n";
}

int main(int argc, char** argv) {
  uint64_t seed = 12345;
  int steps = 400;
  double dt = 0.05;

  int num_targets = 3;
  double sigma_z = 3.0;
  double p_detect = 0.90;

  int enable_clutter = 1;
  int clutter_per_step = 6;
  double clutter_area_half = 300.0;

  double sigma_a = 1.5;

  double gate_maha2 = 9.21;
  int max_misses = 8;

  int confirm_M = 3;
  int confirm_N = 5;

  int use_hungarian = 1;

  // demo
  int assoc_demo = 0;

  // scenario
  bool scenario_cross = false;

  std::string out_dir = "out";

  for (int i = 1; i < argc; ++i) {
    if (arg_eq(argv[i], "--seed") && i + 1 < argc) seed = parse_u64(argv[++i]);
    else if (arg_eq(argv[i], "--steps") && i + 1 < argc) steps = parse_i(argv[++i]);
    else if (arg_eq(argv[i], "--dt") && i + 1 < argc) dt = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--targets") && i + 1 < argc) num_targets = parse_i(argv[++i]);
    else if (arg_eq(argv[i], "--sigma_z") && i + 1 < argc) sigma_z = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--p_detect") && i + 1 < argc) p_detect = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--sigma_a") && i + 1 < argc) sigma_a = parse_d(argv[++i]);

    else if (arg_eq(argv[i], "--clutter") && i + 1 < argc) enable_clutter = parse_b(argv[++i]);
    else if (arg_eq(argv[i], "--clutter_n") && i + 1 < argc) clutter_per_step = parse_i(argv[++i]);
    else if (arg_eq(argv[i], "--clutter_A") && i + 1 < argc) clutter_area_half = parse_d(argv[++i]);

    else if (arg_eq(argv[i], "--gate_maha2") && i + 1 < argc) gate_maha2 = parse_d(argv[++i]);
    else if (arg_eq(argv[i], "--max_misses") && i + 1 < argc) max_misses = parse_i(argv[++i]);

    else if (arg_eq(argv[i], "--confirm_M") && i + 1 < argc) confirm_M = parse_i(argv[++i]);
    else if (arg_eq(argv[i], "--confirm_N") && i + 1 < argc) confirm_N = parse_i(argv[++i]);

    else if (arg_eq(argv[i], "--hungarian") && i + 1 < argc) use_hungarian = parse_b(argv[++i]);
    else if (arg_eq(argv[i], "--assoc_demo") && i + 1 < argc) assoc_demo = parse_b(argv[++i]);

    else if (arg_eq(argv[i], "--scenario") && i + 1 < argc) {
      std::string s = argv[++i];
      scenario_cross = (s == "cross");
    }

    else if (arg_eq(argv[i], "--out") && i + 1 < argc) out_dir = argv[++i];
    else if (arg_eq(argv[i], "--help")) {
      std::cout
        << "radar_tracker options:\n"
        << "  --seed N\n"
        << "  --steps N\n"
        << "  --dt SEC\n"
        << "  --targets N\n"
        << "  --sigma_z METERS\n"
        << "  --p_detect P\n"
        << "  --sigma_a\n"
        << "  --clutter 0|1\n"
        << "  --clutter_n N\n"
        << "  --clutter_A METERS\n"
        << "  --gate_maha2\n"
        << "  --max_misses\n"
        << "  --confirm_M M\n"
        << "  --confirm_N N\n"
        << "  --hungarian 0|1\n"
        << "  --assoc_demo 0|1\n"
        << "  --scenario random|cross\n"
        << "  --out DIR\n";
      return 0;
    }
  }

  if (assoc_demo) {
    run_assoc_demo();
    return 0;
  }

  if (confirm_N < 1) confirm_N = 1;
  if (confirm_M < 1) confirm_M = 1;
  if (confirm_M > confirm_N) confirm_M = confirm_N;

  std::filesystem::create_directories(out_dir);

  SimConfig scfg;
  scfg.num_targets = num_targets;
  scfg.dt = dt;
  scfg.steps = steps;
  scfg.sigma_z = sigma_z;
  scfg.p_detect = p_detect;
  scfg.enable_clutter = (enable_clutter != 0);
  scfg.clutter_per_step = clutter_per_step;
  scfg.clutter_area_half = clutter_area_half;
  scfg.scenario_cross = scenario_cross;

  TargetSim2D sim(seed, scfg);

  TrackerConfig tcfg;
  tcfg.gate_maha2 = gate_maha2;
  tcfg.max_misses = max_misses;
  tcfg.confirm_M = confirm_M;
  tcfg.confirm_N = confirm_N;
  tcfg.use_hungarian = (use_hungarian != 0);

  MultiTargetTracker tracker(tcfg);

  Csv truth_csv(out_dir + "/truth.csv");
  Csv meas_csv(out_dir + "/meas.csv");
  Csv tracks_csv(out_dir + "/tracks.csv");
  Csv resid_csv(out_dir + "/residuals.csv");

  truth_csv.header("step,true_id,x,y,vx,vy");
  meas_csv.header("step,true_id,zx,zy");
  tracks_csv.header("step,track_id,confirmed,x,y,vx,vy,misses,maha2,hits_window");
  resid_csv.header("step,track_id,innov_x,innov_y,S00,S01,S10,S11");

  Fnv1a64 fnv;
  fnv.add("RADAR_TRACKING_V8\n");
  fnv.add_u64(seed);

  uint64_t total_meas = 0;
  uint64_t total_clutter = 0;

  uint32_t max_track_id_seen = 0;
  uint64_t assoc_updates = 0;
  double maha2_sum = 0.0;

  const auto t0 = std::chrono::steady_clock::now();

  for (int step = 0; step < steps; ++step) {
    sim.step();

    for (const auto& t : sim.truth()) {
      truth_csv.out << step << "," << t.id << ","
                    << std::setprecision(17) << t.pos.x() << ","
                    << std::setprecision(17) << t.pos.y() << ","
                    << std::setprecision(17) << t.vel.x() << ","
                    << std::setprecision(17) << t.vel.y() << "\n";
    }

    std::vector<Vec2> z;
    z.reserve(sim.last_measurements().size());

    total_meas += sim.last_measurements().size();
    for (const auto& m : sim.last_measurements()) {
      if (m.true_id == 0) total_clutter++;
      z.push_back(m.z);

      meas_csv.out << step << "," << m.true_id << ","
                   << std::setprecision(17) << m.z.x() << ","
                   << std::setprecision(17) << m.z.y() << "\n";
    }

    tracker.step(z, dt, sigma_a, sigma_z);

    const auto& tracks = tracker.tracks();
    const auto& innovs = tracker.last_innovations();
    const auto& Ss = tracker.last_S();

    for (size_t i = 0; i < tracks.size(); ++i) {
      const auto& tr = tracks[i];
      max_track_id_seen = std::max(max_track_id_seen, tr.id);

      const int hits_window = tr.hits_in_window();

      tracks_csv.out << step << "," << tr.id << "," << (tr.confirmed ? 1 : 0) << ","
                     << std::setprecision(17) << tr.kf.x(0) << ","
                     << std::setprecision(17) << tr.kf.x(1) << ","
                     << std::setprecision(17) << tr.kf.x(2) << ","
                     << std::setprecision(17) << tr.kf.x(3) << ","
                     << tr.misses << ","
                     << std::setprecision(17) << tr.last_maha2 << ","
                     << hits_window
                     << "\n";

      resid_csv.out << step << "," << tr.id << ","
                    << std::setprecision(17) << innovs[i].x() << ","
                    << std::setprecision(17) << innovs[i].y() << ","
                    << std::setprecision(17) << Ss[i](0,0) << ","
                    << std::setprecision(17) << Ss[i](0,1) << ","
                    << std::setprecision(17) << Ss[i](1,0) << ","
                    << std::setprecision(17) << Ss[i](1,1)
                    << "\n";

      if (tr.last_maha2 > 0.0) {
        assoc_updates++;
        maha2_sum += tr.last_maha2;
      }
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  const double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double ms_per_step = (steps > 0) ? (elapsed_ms / (double)steps) : 0.0;
  const double steps_per_sec = (ms_per_step > 0.0) ? (1000.0 / ms_per_step) : 0.0;

  int confirmed_final = 0;
  for (const auto& tr : tracker.tracks()) if (tr.confirmed) confirmed_final++;

  const double maha2_avg = (assoc_updates > 0) ? (maha2_sum / (double)assoc_updates) : 0.0;

  std::cerr << "FNV1A64=" << std::hex << fnv.h << std::dec << "\n";
  std::cout << "Wrote logs to: " << out_dir << "\n";
  std::cout << "Files: truth.csv, meas.csv, tracks.csv, residuals.csv\n";

  std::cout << "\n=== RUN SUMMARY ===\n";
  std::cout << "scenario=" << (scenario_cross ? "cross" : "random") << "\n";
  std::cout << "hungarian=" << (tcfg.use_hungarian ? 1 : 0) << "\n";
  std::cout << "steps=" << steps
            << " dt=" << dt
            << " targets=" << (scenario_cross ? 2 : num_targets)
            << " sigma_z=" << sigma_z
            << " p_detect=" << p_detect
            << " clutter=" << (scfg.enable_clutter ? 1 : 0)
            << " clutter_n=" << scfg.clutter_per_step
            << " clutter_A=" << scfg.clutter_area_half
            << "\n";
  std::cout << "confirm_M=" << confirm_M << " confirm_N=" << confirm_N << "\n";
  std::cout << "measurements_total=" << total_meas
            << " clutter_total=" << total_clutter
            << "\n";
  std::cout << "tracks_created_estimate=" << max_track_id_seen
            << " tracks_alive_final=" << tracker.tracks().size()
            << " confirmed_final=" << confirmed_final
            << "\n";
  std::cout << "assoc_updates=" << assoc_updates
            << " maha2_avg=" << std::setprecision(6) << maha2_avg
            << "\n";
  std::cout << "elapsed_ms=" << std::setprecision(3) << elapsed_ms
            << " ms_per_step=" << std::setprecision(6) << ms_per_step
            << " steps_per_sec=" << std::setprecision(3) << steps_per_sec
            << "\n";

  return 0;
}