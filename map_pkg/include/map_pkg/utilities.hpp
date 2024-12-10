#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include <cmath>
#include <tuple>

#include "map_pkg/obstacle_struct.hpp"

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

using Point = std::tuple<double, double>;

std::vector<Point> create_hexagon_v(double dx);
geometry_msgs::msg::Polygon create_hexagon(double dx);
std::vector<Point> create_rectangle_v(double dx, double dy, double x = 0.0, double y = 0.0, double yaw = 0.0);
geometry_msgs::msg::Polygon create_rectangle(double dx, double dy, double x = 0.0, double y = 0.0, double yaw = 0.0);
