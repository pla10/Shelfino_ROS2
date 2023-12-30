#include "map_pkg/utilities.hpp"

h2d::CPolyline obs_to_cpoly (const obstacle& obs){
  h2d::CPolyline poly;
  if (obs.type == obstacle_type::BOX){
    poly = h2d::CPolyline(std::vector<h2d::Point2d> {
      {obs.x - obs.dx/2.0, obs.y + obs.dy/2.0},
      {obs.x - obs.dx/2.0, obs.y - obs.dy/2.0},
      {obs.x + obs.dx/2.0, obs.y - obs.dy/2.0},
      {obs.x + obs.dx/2.0, obs.y + obs.dy/2.0}
    });
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Obstacle type %d not supported by this function.", obs.type);
  }
  return poly;
}

h2d::Circle obs_to_circle (const obstacle& obs){
  h2d::Circle circle;
  if (obs.type == obstacle_type::CYLINDER){
    circle = h2d::Circle(h2d::Point2d(obs.x, obs.y), obs.radius);
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Obstacle type %d not supported by this function.", obs.type);
  }
  return circle;
}

/**
 * @brief Function that checks that two obstacles do not overlap
 * 
 * @param obs1 Obstacle 1
 * @param obs2 Obstacle 2
 * @return true If the obstacles overlap
 * @return false If the obstacles do not overlap
 */
bool overlaps(obstacle obs1, obstacle obs2) {
  bool ret = false;
  if (obs1.type == obstacle_type::CYLINDER && obs2.type == obstacle_type::CYLINDER) {
    h2d::Circle obs1_circle(obs_to_circle(obs1));
    h2d::Circle obs2_circle(obs_to_circle(obs2));

    ret = ( 
            obs1_circle == obs2_circle ||
            obs1_circle.intersects(obs2_circle).size() > 0 || 
            obs1_circle.isInside(obs2_circle) || 
            obs2_circle.isInside(obs1_circle)
    );
  } 
  else if (obs1.type == obstacle_type::BOX && obs2.type == obstacle_type::BOX) {
    h2d::CPolyline obs1_rect(obs_to_cpoly(obs1));
    h2d::CPolyline obs2_rect(obs_to_cpoly(obs2));

    ret = (
            obs1_rect == obs2_rect ||
            obs1_rect.intersects(obs2_rect).size() > 0 || 
            obs1_rect.isInside(obs2_rect) || 
            obs2_rect.isInside(obs1_rect)
    );
  } 
  else {
    if (obs1.type == obstacle_type::BOX) {
      obstacle temp = obs1;
      obs1 = obs2;
      obs2 = temp;
    }

    h2d::Circle obs1_circle(obs_to_circle(obs1));
    h2d::CPolyline obs2_rect(obs_to_cpoly(obs2));

    ret = (
            obs1_circle.intersects(obs2_rect).size() > 0 || 
            obs1_circle.isInside(obs2_rect) || 
            obs2_rect.isInside(obs1_circle)
    );
  }
  return ret;
}

/**
 * @brief Function that checks if an obstacles overlaps with any of the obstacles in a vector
 * 
 * @param obs1 The obstacle to check
 * @param obstacles The vector of obstacles to check against
 * @return true If the obstacle overlaps with any of the obstacles in the vector
 * @return false If the obstacle does not overlap with any of the obstacles in the vector
 */
bool overlaps(obstacle obs1, std::vector<obstacle> obstacles){
  for (auto obs2 : obstacles){
    if (overlaps(obs1, obs2)){
      return true;
    }
  }
  return false;
}


/**
 * @brief It checks if the obstacle is inside the map
 * 
 * @param obs The obstacle to check
 * @param map The map to use
 * @param dx The x dimension of the map
 * @param dy The y dimension of the map
 * @return true If the obstacle is inside the map
 * @return false If the obstacle is not inside the map
 */
bool is_inside_map(obstacle obs, std::string map, double dx, double dy){
  bool inside = false;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
    "Checking obstacle %f, %f, %f, %f, %f, in map %s dx: %f  dy: %f", 
    obs.x, obs.y, obs.radius, obs.dx, obs.dy, map.c_str(), dx, dy
  );

  h2d::CPolyline map_poly;
  if (map == "rectangle"){
    obstacle tmp = {0.0, 0.0, 0.0, dx, dy, obstacle_type::BOX};
    map_poly = obs_to_cpoly(tmp);
  }
  else if (map == "hexagon"){
    std::vector<h2d::Point2d> vertexes;
    vertexes.push_back(h2d::Point2d(            0.0, -dx));
    vertexes.push_back(h2d::Point2d(-sqrt(3)/2.0*dx, -dx/2.0));
    vertexes.push_back(h2d::Point2d(-sqrt(3)/2.0*dx,  dx/2.0));
    vertexes.push_back(h2d::Point2d(            0.0,  dx));
    vertexes.push_back(h2d::Point2d( sqrt(3)/2.0*dx,  dx/2.0));
    vertexes.push_back(h2d::Point2d( sqrt(3)/2.0*dx, -dx/2.0));
    map_poly = h2d::CPolyline(vertexes);
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Map %s not recognized.", map.c_str());
    inside = false;
  }

  if (obs.type == obstacle_type::CYLINDER){
    h2d::Circle obs_circle(obs_to_circle(obs));
    if (obs_circle.isInside(map_poly) && obs_circle.intersects(map_poly).size() == 0) {
      inside = true;
    }
  }
  else if (obs.type == obstacle_type::BOX){
    h2d::CPolyline obs_rect = obs_to_cpoly(obs);
    if (obs_rect.isInside(map_poly) && obs_rect.intersects(map_poly).size() == 0) {
      inside = true;
    }
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Obstacle type %d not recognized.", obs.type);
    inside = false;
  }  

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Result: %s", (inside ? "inside" : "NOT inside"));
  return inside;
}


geometry_msgs::msg::Polygon create_hexagon(double dx){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        double f = 0.866;  // fixed number for apothem of hexagon calculation
        point.x = -dx/2;
        point.y = dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = -dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx/2;
        point.y = -dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}


geometry_msgs::msg::Polygon create_rectangle(double dx, double dy){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = -dx/2; 
        point.y = -dy/2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx/2; 
        point.y = dy/2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = dy/2; 
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = -dy/2; 
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}

/**
 * @brief Checks if the obstacle overlaps with any other obstacle in the vector
 * 
 * @param obs The obstacle to check
 * @param obstacles The vector of obstacles
 * @param map The type of the map (rectangle or hexagon)
 * @param dx x dimension of the map
 * @param dy y dimension of the map
 * @return true If the obstacle does not overlap and is inside the map
 * @return false If the obstacle overlaps or is outside the map
 */
bool valid_position(
  std::string map, double dx, double dy,
  const obstacle & obs, std::vector<std::vector<obstacle>> others 
)
{
  bool res = is_inside_map(obs, map, dx, dy);
  for (auto other : others) {
    res &= !overlaps(obs, other);
  }
  return res;
}