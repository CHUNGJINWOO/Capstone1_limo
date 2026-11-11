#ifndef FRONTIER_SEARCH_RELAXED_H_
#define FRONTIER_SEARCH_RELAXED_H_

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace frontier_exploration_relaxed
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double cost;
  geometry_msgs::msg::Point initial;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point middle;
  std::vector<geometry_msgs::msg::Point> points;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
// Statistics of the last searchFrom() call (for logging / offline comparison).
struct SearchStats {
  bool start_found{false};
  double start_dist_m{0.0};
  std::uint64_t visited{0};             // cells visited by the BFS
  std::uint64_t traversed_nonzero{0};   // visited cells whose cost is > 0 (inflated) -- 0 with the original rule
  unsigned int max_cost_visited{0};     // highest internal cost the BFS entered (must stay < 253)
  std::size_t frontiers_found{0};       // before the min_frontier_size filter
  std::size_t frontiers_kept{0};        // after the filter
};

class FrontierSearch
{
public:
  FrontierSearch() : logger_(rclcpp::get_logger("frontier_search")) {} // Default constructor for the logger

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  // bfs_max_traverse_cost is in OccupancyGrid units (0..98), converted with explore's cost translation table.
  // 0 (default) = original explore_lite rule (the BFS only moves to neighbours whose cost <= the current cell's).
  // >0 additionally lets the BFS enter cells whose cost is <= that value; cells at or above INSCRIBED (OccupancyGrid 99),
  // lethal cells and unknown cells are never entered, so the unknown/frontier rule is unchanged.
  FrontierSearch(nav2_costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale, double min_frontier_size, rclcpp::Logger logger,
                 int bfs_max_traverse_cost = 0);

  // Discovery and drivability are separate: true if a path of known, non-inscribed (< INSCRIBED) cells connects `from`
  // (the start cell is always allowed, like NavFn's cleared robot cell) to a cell within 2 cells of `goal`.
  // Conservative: unknown cells are not traversed. Does not call Nav2; offline/online comparison with the planner is done
  // by the experiment scripts.
  bool reachable(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& goal);

  const SearchStats& lastStats() const { return stats_; }
  unsigned char bfsCapInternal() const { return bfs_cap_; }

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::msg::Point position);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

private:
  nav2_costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
  unsigned char bfs_cap_{0};
  SearchStats stats_;
  rclcpp::Logger logger_;
};
}  // namespace frontier_exploration_relaxed
#endif
