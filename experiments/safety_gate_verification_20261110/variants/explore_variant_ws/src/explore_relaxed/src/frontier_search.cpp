#include <explore_relaxed/costmap_tools.h>
#include <explore_relaxed/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <queue>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration_relaxed
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

// OccupancyGrid value (1..98) -> internal cost, same formula as explore's cost_translation_table__ (1 + 251*(v-1)/97).
static unsigned char msgCostToInternal(int v)
{
  if (v <= 0) {
    return 0;
  }
  if (v > 98) {
    v = 98;  // never reach INSCRIBED (OccupancyGrid 99 -> 253)
  }
  return static_cast<unsigned char>(1 + (251 * (v - 1)) / 97);
}

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size, rclcpp::Logger logger,
                               int bfs_max_traverse_cost)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , bfs_cap_(msgCostToInternal(bfs_max_traverse_cost))
  , logger_(logger)
{
}

bool FrontierSearch::reachable(const geometry_msgs::msg::Point& from,
                               const geometry_msgs::msg::Point& goal)
{
  unsigned int fx, fy, gx, gy;
  if (!costmap_->worldToMap(from.x, from.y, fx, fy) ||
      !costmap_->worldToMap(goal.x, goal.y, gx, gy)) {
    return false;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  const unsigned char* map = costmap_->getCharMap();
  const unsigned int sx = costmap_->getSizeInCellsX(), sy = costmap_->getSizeInCellsY();
  std::vector<bool> seen(static_cast<size_t>(sx) * sy, false);
  std::queue<unsigned int> q;
  const unsigned int start = costmap_->getIndex(fx, fy);
  q.push(start);
  seen[start] = true;
  while (!q.empty()) {
    const unsigned int idx = q.front();
    q.pop();
    unsigned int cx, cy;
    costmap_->indexToCells(idx, cx, cy);
    if (std::abs(static_cast<int>(cx) - static_cast<int>(gx)) <= 2 &&
        std::abs(static_cast<int>(cy) - static_cast<int>(gy)) <= 2) {
      return true;
    }
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = static_cast<int>(cx) + dx, ny = static_cast<int>(cy) + dy;
        if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 || nx >= static_cast<int>(sx) || ny >= static_cast<int>(sy)) {
          continue;
        }
        const unsigned int n = costmap_->getIndex(nx, ny);
        // known and below INSCRIBED (253): same threshold NavFn uses for "obstacle"
        if (!seen[n] && map[n] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          seen[n] = true;
          q.push(n);
        }
      }
    }
  }
  return false;
}

std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(logger_, "[FrontierSearch] Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(logger_, "[FrontierSearch] Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;
  stats_ = SearchStats();
  stats_.start_found = (map_[bfs.front()] == FREE_SPACE);
  {
    unsigned int sx0, sy0;
    costmap_->indexToCells(bfs.front(), sx0, sy0);
    stats_.start_dist_m = std::hypot(static_cast<double>(sx0) - mx, static_cast<double>(sy0) - my) * costmap_->getResolution();
  }

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      // Original rule: only move to cells whose cost is <= the current cell's (descending search).
      // Relaxed rule (bfs_cap_ > 0): additionally enter cells with cost <= bfs_cap_ (<= 252, i.e. never inscribed,
      // lethal or unknown). With bfs_cap_ == 0 this is exactly the original condition.
      const bool descend = map_[nbr] <= map_[idx];
      const bool relaxed = bfs_cap_ > 0 && map_[nbr] <= bfs_cap_;
      if ((descend || relaxed) && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        ++stats_.visited;
        if (map_[nbr] > 0) {
          ++stats_.traversed_nonzero;
        }
        if (map_[nbr] > stats_.max_cost_visited) {
          stats_.max_cost_visited = map_[nbr];
        }
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        ++stats_.frontiers_found;
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  stats_.frontiers_kept = frontier_list.size();
  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}  // namespace frontier_exploration_relaxed
