// Equivalence + comparison test: original explore_lite FrontierSearch (namespace frontier_exploration, compiled from the
// untouched original source) versus explore_relaxed FrontierSearch (namespace frontier_exploration_relaxed).
//
// 1) 60 random synthetic costmaps: with bfs_max_traverse_cost = 0 the two must return identical frontier lists.
// 2) For each costmap dump given on the command line (W H res ox oy rx ry, then W*H OccupancyGrid values), print the
//    frontier counts, BFS statistics and reachability for several bfs_max_traverse_cost values, and assert that the BFS
//    never entered a cell at or above INSCRIBED (253).
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include <explore/frontier_search.h>            // original
#include <explore_relaxed/frontier_search.h>    // variant
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

static unsigned char translate(int v)
{
  // identical to explore's costmap_client.cpp init_translation_table()
  if (v == 0) return 0;
  if (v == 99) return 253;
  if (v == 100) return 254;
  if (v < 0) return 255;
  return static_cast<unsigned char>(1 + (251 * (v - 1)) / 97);
}

static bool same(const std::vector<frontier_exploration::Frontier>& a,
                 const std::vector<frontier_exploration_relaxed::Frontier>& b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].size != b[i].size || a[i].cost != b[i].cost || a[i].min_distance != b[i].min_distance ||
        a[i].middle.x != b[i].middle.x || a[i].middle.y != b[i].middle.y || a[i].centroid.x != b[i].centroid.x ||
        a[i].centroid.y != b[i].centroid.y || a[i].points.size() != b[i].points.size())
      return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("equivalence_test");
  int failures = 0;

  // ---- 1) random synthetic maps, original vs relaxed(cap=0)
  std::mt19937 rng(12345);
  for (int t = 0; t < 60; ++t) {
    const unsigned W = 60 + rng() % 40, H = 60 + rng() % 40;
    nav2_costmap_2d::Costmap2D cm(W, H, 0.05, 0.0, 0.0);
    unsigned char* d = cm.getCharMap();
    for (unsigned i = 0; i < W * H; ++i) {
      const int r = rng() % 100;
      d[i] = r < 55 ? 0 : (r < 75 ? 255 : (r < 85 ? 1 + rng() % 252 : (r < 92 ? 253 : 254)));
    }
    // smooth a bit by blocks so that structure exists
    for (unsigned y = 0; y + 3 < H; y += 3)
      for (unsigned x = 0; x + 3 < W; x += 3) {
        const unsigned char v = d[y * W + x];
        for (unsigned dy = 0; dy < 3; ++dy)
          for (unsigned dx = 0; dx < 3; ++dx) d[(y + dy) * W + x + dx] = v;
      }
    geometry_msgs::msg::Point p;
    p.x = (W / 2) * 0.05; p.y = (H / 2) * 0.05;
    frontier_exploration::FrontierSearch orig(&cm, 3.0, 1.0, 0.35, logger);
    frontier_exploration_relaxed::FrontierSearch rel(&cm, 3.0, 1.0, 0.35, logger, 0);
    const auto a = orig.searchFrom(p);
    const auto b = rel.searchFrom(p);
    if (!same(a, b)) {
      std::printf("EQUIVALENCE FAIL on random map %d (orig %zu vs relaxed %zu frontiers)\n", t, a.size(), b.size());
      ++failures;
    }
  }
  std::printf("random-map equivalence (cap=0): %s\n", failures == 0 ? "PASS 60/60" : "FAIL");

  // ---- 2) real snapshot dumps
  for (int ai = 1; ai < argc; ++ai) {
    std::ifstream f(argv[ai]);
    unsigned W, H; double res, ox, oy, rx, ry;
    if (!(f >> W >> H >> res >> ox >> oy >> rx >> ry)) { std::printf("cannot read %s\n", argv[ai]); continue; }
    nav2_costmap_2d::Costmap2D cm(W, H, res, ox, oy);
    unsigned char* d = cm.getCharMap();
    for (unsigned i = 0; i < W * H; ++i) { int v; f >> v; d[i] = translate(v); }
    geometry_msgs::msg::Point p; p.x = rx; p.y = ry;
    std::printf("== %s  robot=(%.2f,%.2f)\n", argv[ai], rx, ry);
    frontier_exploration::FrontierSearch orig(&cm, 3.0, 1.0, 0.35, logger);
    const auto a0 = orig.searchFrom(p);
    frontier_exploration_relaxed::FrontierSearch rel0(&cm, 3.0, 1.0, 0.35, logger, 0);
    const auto b0 = rel0.searchFrom(p);
    const bool eq = same(a0, b0);
    if (!eq) ++failures;
    std::printf("   original: kept=%zu | relaxed(cap 0): kept=%zu identical=%s\n", a0.size(), b0.size(), eq ? "YES" : "NO");
    for (int cap : {0, 20, 40, 45, 50, 55, 60, 70, 80, 98}) {
      frontier_exploration_relaxed::FrontierSearch rel(&cm, 3.0, 1.0, 0.35, logger, cap);
      const auto fr = rel.searchFrom(p);
      const auto& st = rel.lastStats();
      size_t reach = 0, ne = 0;
      for (const auto& fo : fr) {
        if (rel.reachable(p, fo.middle)) ++reach;
        // map frame -> world with the default sim spawn (-1.10,-0.60, yaw 1.5708); NE corridor box x 3.0..5.8, y 1.8..4.3
        const double wx = std::cos(1.5708) * fo.middle.x - std::sin(1.5708) * fo.middle.y - 1.10;
        const double wy = std::sin(1.5708) * fo.middle.x + std::cos(1.5708) * fo.middle.y - 0.60;
        if (wx >= 3.0 && wx <= 5.8 && wy >= 1.8 && wy <= 4.3) ++ne;
      }
      const bool safe = st.max_cost_visited < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      if (!safe) ++failures;
      std::printf("   cap=%2d (internal %3d): found=%zu kept=%zu reachable_kept=%zu NE_box_frontiers=%zu bfs_visited=%llu traversed_nonzero=%llu max_cost_visited=%u %s\n",
                  cap, rel.bfsCapInternal(), st.frontiers_found, fr.size(), reach, ne, (unsigned long long)st.visited,
                  (unsigned long long)st.traversed_nonzero, st.max_cost_visited, safe ? "" : "UNSAFE(>=253 entered)");
    }
  }
  std::printf("TOTAL failures: %d\n", failures);
  rclcpp::shutdown();
  return failures == 0 ? 0 : 1;
}
