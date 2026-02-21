#pragma once
#include <vector>

// Solve minimum-cost assignment using Hungarian algorithm.
// Input: cost matrix with size rows x cols (rows=tracks, cols=measurements).
// Output: assignment vector of size rows, where assignment[i] = j means row i assigned to col j,
// or -1 means unassigned (when cols < rows or if caller uses large costs to represent invalid).
//
// Deterministic, O(n^3). Works for rectangular matrices by padding internally.
std::vector<int> hungarian_min_cost(const std::vector<std::vector<double>>& cost);