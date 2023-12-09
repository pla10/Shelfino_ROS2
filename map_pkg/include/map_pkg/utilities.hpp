#pragma once 

#include "rclcpp/rclcpp.hpp"

#include "map_pkg/obstacle_struct.hpp"

#include "geometry/homog2d.hpp"

static const rmw_qos_profile_t rmw_qos_profile_custom =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

/**
 * @brief Checks if the time elapsed since start_time is greater than max_timeout
 * 
 * @param clock The clock to use
 * @param start_time The initial time
 * @param max_timeout The maximum time to wait
 * @return true If the time elapsed is greater than max_timeout
 * @return false If the time elapsed is not greater than max_timeout
 */
inline bool overTime (
  const rclcpp::Clock::SharedPtr& clock, 
  const rclcpp::Time& start_time, 
  int max_timeout) 
{
  return (clock->now() - start_time).seconds() > max_timeout;
}

/**
 * @brief Function that checks that two obstacles do not overlap
 * 
 * @param o1 Obstacle 1
 * @param o2 Obstacle 2
 * @return true If the obstacles overlap
 * @return false If the obstacles do not overlap
 */
bool overlaps(obstacle o1, obstacle o2);

/**
 * @brief Function that checks if an obstacles overlaps with any of the obstacles in a vector
 * 
 * @param o1 The obstacle to check
 * @param obstacles The vector of obstacles to check against
 * @return true If the obstacle overlaps with any of the obstacles in the vector
 * @return false If the obstacle does not overlap with any of the obstacles in the vector
 */
bool overlaps(obstacle o1, std::vector<obstacle> obstacles);

/**
 * @brief It checks if the obstacle is inside the map
 * @details For the hexagon map, it checks only the main square plus the rectangles 
 * above, below, left and right. It does not check the corners, which are consider
 * out of the map.
 * 
 * @param obs The obstacle to check
 * @param map The map to use
 * @param dx The x dimension of the map
 * @param dy The y dimension of the map
 * @return true If the obstacle is inside the map
 * @return false If the obstacle is not inside the map
 */
bool is_inside_map(obstacle obs, std::string map, double dx, double dy);

