#include "hungarian.h"
#include <algorithm>
#include <limits>

// Classic Hungarian algorithm implementation for rectangular matrices (minimization).
// Uses potentials (u, v) and p/way arrays (1-indexed) on padded square matrix.
std::vector<int> hungarian_min_cost(const std::vector<std::vector<double>>& cost) {
  const int n = (int)cost.size();
  const int m = (n > 0) ? (int)cost[0].size() : 0;

  if (n == 0) return {};
  if (m == 0) return std::vector<int>(n, -1);

  const int N = std::max(n, m);
  const double INF = 1e100;

  // Build padded square matrix a[1..N][1..N]
  std::vector<std::vector<double>> a(N + 1, std::vector<double>(N + 1, 0.0));
  for (int i = 1; i <= N; ++i) {
    for (int j = 1; j <= N; ++j) {
      if (i <= n && j <= m) a[i][j] = cost[i - 1][j - 1];
      else a[i][j] = 0.0; // padding cost; caller controls "validity" via large costs in original
    }
  }

  // Potentials and matching
  std::vector<double> u(N + 1, 0.0), v(N + 1, 0.0);
  std::vector<int> p(N + 1, 0), way(N + 1, 0);

  // p[j] = matched row for column j
  for (int i = 1; i <= N; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(N + 1, INF);
    std::vector<char> used(N + 1, false);

    do {
      used[j0] = true;
      int i0 = p[j0];
      int j1 = 0;
      double delta = INF;

      for (int j = 1; j <= N; ++j) {
        if (used[j]) continue;
        double cur = a[i0][j] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }

      for (int j = 0; j <= N; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    // Augment
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  // p[j] gives row matched to column j in padded square.
  // Convert to assignment for original rows (size n), with -1 for unassigned.
  std::vector<int> row_to_col(n, -1);

  // Build inverse: row->col
  for (int j = 1; j <= N; ++j) {
    int i = p[j];
    if (i >= 1 && i <= n) {
      int col = j;
      if (col >= 1 && col <= m) row_to_col[i - 1] = col - 1;
      else row_to_col[i - 1] = -1;
    }
  }

  return row_to_col;
}