#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <unordered_set>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size, rclcpp::Logger logger,
                               double information_gain_scale)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , information_gain_scale_(information_gain_scale)
  , min_frontier_size_(min_frontier_size)
  , logger_(logger)
{
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

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // Calculate information gain and set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.information_gain = calculateInformationGain(frontier);
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
  /*
   * Frontier Cost Function (Lower cost = Higher priority)
   *
   * Formula: cost = α×distance - β×information_gain - γ×size
   *
   * Components:
   * 1. Distance penalty: Prefer nearby frontiers (reduces travel time)
   * 2. Information gain reward: Prefer frontiers with high IG (more unknown cells visible)
   * 3. Size reward: Prefer larger frontiers (backward compatibility)
   *
   * The negative signs make IG and size into rewards (higher values → lower cost → higher priority)
   */

  double distance_cost = potential_scale_ * frontier.min_distance * costmap_->getResolution();
  double ig_reward = information_gain_scale_ * frontier.information_gain;
  double size_reward = gain_scale_ * frontier.size * costmap_->getResolution();

  return distance_cost - ig_reward - size_reward;
}

double FrontierSearch::calculateInformationGain(const Frontier& frontier)
{
  /*
   * Information Gain Calculation via Raycasting
   *
   * Simulates LiDAR sensing from frontier centroid to estimate how many
   * unknown cells would be revealed if robot moved to this frontier.
   *
   * Algorithm:
   * 1. Cast rays in 360° around frontier centroid (simulating LiDAR)
   * 2. For each ray up to LIDAR_MAX_RANGE (3.5m):
   *    - Trace line using Bresenham's algorithm
   *    - Count unknown (NO_INFORMATION) cells encountered
   *    - Stop at obstacles or max range
   * 3. Return total count of unique unknown cells visible
   *
   * Higher IG = more unexplored area visible = better exploration target
   */

  double wx = frontier.centroid.x;
  double wy = frontier.centroid.y;

  // Convert centroid to map coordinates
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    // Centroid outside map bounds, return zero gain
    return 0.0;
  }

  // Track which cells we've already counted (avoid double-counting)
  std::unordered_set<unsigned int> counted_cells;

  double resolution = costmap_->getResolution();
  double max_range_cells = LIDAR_MAX_RANGE / resolution;

  // Cast rays in 360 degrees
  double angle_increment = 2.0 * M_PI / LIDAR_NUM_RAYS;

  for (int ray = 0; ray < LIDAR_NUM_RAYS; ++ray) {
    double angle = ray * angle_increment;
    double ray_dx = cos(angle);
    double ray_dy = sin(angle);

    // Bresenham-style ray tracing
    for (double range = 0.0; range < max_range_cells; range += 0.5) {
      int test_x = static_cast<int>(mx + range * ray_dx);
      int test_y = static_cast<int>(my + range * ray_dy);

      // Check bounds
      if (test_x < 0 || test_x >= static_cast<int>(size_x_) ||
          test_y < 0 || test_y >= static_cast<int>(size_y_)) {
        break;  // Ray left map bounds
      }

      unsigned int test_idx = costmap_->getIndex(test_x, test_y);
      unsigned char cost = map_[test_idx];

      // Stop ray at lethal obstacles (can't see through walls)
      if (cost >= LETHAL_OBSTACLE) {
        break;
      }

      // Count unknown cells
      if (cost == NO_INFORMATION) {
        counted_cells.insert(test_idx);
      }
    }
  }

  // Information gain = number of unique unknown cells visible
  double ig = static_cast<double>(counted_cells.size());

  // Apply frontier size weighting (larger frontiers have more potential)
  // This matches the proposal formula: IG(f) = Σ(unknown_cells) × frontier_size × occlusion_weight
  double size_weight = std::sqrt(static_cast<double>(frontier.size));  // Square root to avoid over-weighting
  double occlusion_weight = 1.0;  // Could be enhanced with line-of-sight analysis

  return ig * size_weight * occlusion_weight;
}
}  // namespace frontier_exploration
