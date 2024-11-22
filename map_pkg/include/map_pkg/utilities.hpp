#pragma once 

#include "geometry/homog2d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
using lifecycle_msgs::srv::ChangeState;

#include "map_pkg/obstacle_struct.hpp"

#include <chrono>
using namespace std::chrono_literals;
#include <thread>
#include <future>

static const rmw_qos_profile_t rmw_qos_profile_custom =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  100,
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
bool overlaps(const Obstacle& o1, const Obstacle& o2);

/**
 * @brief Function that checks if an obstacles overlaps with any of the obstacles in a vector
 * 
 * @param o1 The Obstacle to check
 * @param obstacles The vector of obstacles to check against
 * @return true If the Obstacle overlaps with any of the obstacles in the vector
 * @return false If the Obstacle does not overlap with any of the obstacles in the vector
 */
bool overlaps(const Obstacle & o1, const std::vector<Obstacle> & obstacles);

/**
 * @brief It checks if the Obstacle is inside the map
 * @details For the hexagon map, it checks only the main square plus the rectangles 
 * above, below, left and right. It does not check the corners, which are consider
 * out of the map.
 * 
 * @param obs The Obstacle to check
 * @param map The map to use
 * @param dx The x dimension of the map
 * @param dy The y dimension of the map
 * @return true If the Obstacle is inside the map
 * @return false If the Obstacle is not inside the map
 */
bool is_inside_map(Obstacle obs, std::string map, double dx, double dy);

std::vector<h2d::Point2d> create_hexagon_v(double dx);
geometry_msgs::msg::Polygon create_hexagon(double dx);

std::vector<h2d::Point2d> create_rectangle_v(double dx, double dy, double x = 0.0, double y = 0.0, double yaw = 0.0);
geometry_msgs::msg::Polygon create_rectangle(double dx, double dy, double x = 0.0, double y = 0.0, double yaw = 0.0);

static const std::vector<Obstacle> default_victims = std::vector<Obstacle>();

/**
 * @brief Checks if the Obstacle overlaps with any other Obstacle in the vector
 * 
 * @param obs The Obstacle to check
 * @param obstacles The vector of obstacles
 * @param map The type of the map (rectangle or hexagon)
 * @param dx x dimension of the map
 * @param dy y dimension of the map
 * @return true If the Obstacle does not overlap and is inside the map
 * @return false If the Obstacle overlaps or is outside the map
 */
bool valid_position(
  std::string map, double dx, double dy,
  const Obstacle & obs, std::vector<std::vector<Obstacle>> others
);

inline bool equal_sizes (std::vector<size_t> sizes){
  for (uint i=0; i<sizes.size()-1; i++){
    for (uint j=i+1; j<sizes.size(); j++){
      if (sizes[i] != sizes[j])
        return false;
    }
  }
  return true;
}

template<typename T = double>
inline std::string to_string(std::vector<T> vect){
  std::stringstream ss;
  ss << "[";
  for (const auto& v : vect){ ss << v << ", "; }
  ss << "]";
  return ss.str();
}

/**
 * @brief Function to wait for a future to be ready
 * 
 * @tparam FutureT The type of the future
 * @tparam WaitTimeT The type of the time to wait
 * @param future The future to wait for
 * @param time_to_wait The maximum time to wait
 * @return std::future_status The status of the future
 */
template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

/**
 * @brief Function to invoke the service to change the state of a managed node
 * 
 * @param node The node from which the change state service is called
 * @param change_state_srv Service listener to change the state, it must be subscribed to /node/change_state
 * @param new_state The id of the new state to change to
 * @param time_out The maximum time to wait for the service to respond
 * @return true The service was called successfully
 * @return false The service was not called successfully
 */
template<class NodeType>
bool change_state(NodeType node, rclcpp::Client<ChangeState>::SharedPtr change_state_srv, std::uint8_t new_state, 
  std::chrono::seconds time_out = std::chrono::seconds(0)
)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = new_state;

  if(!change_state_srv->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Service %s is not available.",
      change_state_srv->get_service_name());
    return false;
  }

  // We send the request with the transition we want to invoke.
  auto future_result = change_state_srv->async_send_request(request).future.share();

  return true;

  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      node->get_logger(), "Server time out while setting new state");
    return false;
  }

  // We have an answer, let's print our success.
  if (future_result.get()->success) {
    RCLCPP_INFO(
      node->get_logger(), "Transition %d successfully triggered.", static_cast<int>(new_state));
    return true;
  } else {
    RCLCPP_WARN(
      node->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(new_state));
    return false;
  }

}

