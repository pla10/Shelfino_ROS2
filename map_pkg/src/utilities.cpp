#include "map_pkg/utilities.hpp"

/**
 * @brief Rotates a polygon by an angle clockwise
 * 
 * @param p The CPolyline to rotate
 * @param th The rotation angle in radians
 */
static void rotate(h2d::CPolyline& p, double th, double cx = std::numeric_limits<double>::quiet_NaN(), double cy = std::numeric_limits<double>::quiet_NaN()){
    // Check if cx and cy are NaN
    if (std::isnan(cx) || std::isnan(cy)){
        // Find center of polygon
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = -std::numeric_limits<double>::max();
        double max_y = -std::numeric_limits<double>::max();
        for (size_t i = 0; i<p.size(); i++) {
            auto pt = p.getPoint(i);
            min_x = std::min(min_x, pt.getX());
            min_y = std::min(min_y, pt.getY());
            max_x = std::max(max_x, pt.getX());
            max_y = std::max(max_y, pt.getY());
        }
        cx = (min_x+max_x)/2;
        cy = (min_y+max_y)/2;
    }
    h2d::Point2d center = {cx, cy};

    // Translate to the origin
    h2d::Homogr h;
    h.setTranslation(-center.getX(), -center.getY());        
    p = h * p;

    // Rotate around the origin
    std::vector<h2d::Point2d> p1_v = {};
    for (size_t i = 0; i< p.size(); i++) {
        auto point = p.getPoint(i);
        const double x = point.getX()*cos(th) - point.getY()*sin(th);
        const double y = point.getX()*sin(th) + point.getY()*cos(th);
        p1_v.push_back({x, y});
    }
    p = h2d::CPolyline(p1_v);
    
    // Translate to initial center
    h.setTranslation(center.getX(), center.getY());
    p = h * p;
}

h2d::CPolyline obs_to_cpoly (const Obstacle& obs){
  h2d::CPolyline poly;
  if (obs.type == OBSTACLE_TYPE::BOX){
    // The vertices of the rectangle as if there were no rotation
    std::vector<h2d::Point2d> norm_v = {
      {obs.x - obs.dx/2.0, obs.y + obs.dy/2.0},
      {obs.x - obs.dx/2.0, obs.y - obs.dy/2.0},
      {obs.x + obs.dx/2.0, obs.y - obs.dy/2.0},
      {obs.x + obs.dx/2.0, obs.y + obs.dy/2.0}
    };

    poly = h2d::CPolyline(norm_v);

    // The rotation angle is saved as clockwise, but the rotation matrix is counter-clockwise
    const double th = - obs.yaw;

    if (th != 0){
      rotate(poly, th);
    }
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Obstacle type %d not supported by this function.", obs.type);
  }
  return poly;
}

h2d::Circle obs_to_circle (const Obstacle& obs){
  h2d::Circle circle;
  if (obs.type == OBSTACLE_TYPE::CYLINDER){
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
bool overlaps(const Obstacle& obs1, const Obstacle& obs2) {
  std::cout << "Checking if overlap\n\t" << obs1 << "\n\t" << obs2 << std::endl;

  bool ret = false;
  if (obs1.type == OBSTACLE_TYPE::CYLINDER && obs2.type == OBSTACLE_TYPE::CYLINDER) {
    h2d::Circle obs1_circle(obs_to_circle(obs1));
    h2d::Circle obs2_circle(obs_to_circle(obs2));

    ret = ( 
            obs1_circle == obs2_circle ||
            obs1_circle.intersects(obs2_circle).size() > 0 || 
            obs1_circle.isInside(obs2_circle) || 
            obs2_circle.isInside(obs1_circle)
    );
  } 
  else if (obs1.type == OBSTACLE_TYPE::BOX && obs2.type == OBSTACLE_TYPE::BOX) {
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
    h2d::Circle obs_circle; // (obs_to_circle(new_obs1));
    h2d::CPolyline obs_rect; // (obs_to_cpoly(new_obs2));

    if (obs1.type == OBSTACLE_TYPE::BOX) {
      std::cout << "First is a box" << std::endl;
      obs_rect = obs_to_cpoly(obs1);
      obs_circle = obs_to_circle(obs2);
    }
    else {
      std::cout << "Second is a box" << std::endl;
      obs_rect = obs_to_cpoly(obs2);
      obs_circle = obs_to_circle(obs1);
    }

    ret = (
            obs_circle.intersects(obs_rect).size() > 0 || 
            obs_circle.isInside(obs_rect) || 
            obs_rect.isInside(obs_circle)
    );
  }
  std::cout << "Result: " << (ret ? "overlap" : "do not overlap") << std::endl;
  return ret;
}

/**
 * @brief Function that checks if an obstacles overlaps with any of the obstacles in a vector
 * 
 * @param obs1 The Obstacle to check
 * @param obstacles The vector of obstacles to check against
 * @return true If the Obstacle overlaps with any of the obstacles in the vector
 * @return false If the Obstacle does not overlap with any of the obstacles in the vector
 */
bool overlaps(const Obstacle& obs1, const std::vector<Obstacle>& obstacles){
  for (auto obs2 : obstacles){
    if (overlaps(obs1, obs2)){
      return true;
    }
  }
  return false;
}


/**
 * @brief It checks if the Obstacle is inside the map
 * 
 * @param obs The Obstacle to check
 * @param map The map to use
 * @param dx The x dimension of the map
 * @param dy The y dimension of the map
 * @return true If the Obstacle is inside the map
 * @return false If the Obstacle is not inside the map
 */
bool is_inside_map(Obstacle obs, std::string map, double dx, double dy){
  bool inside = false;

  printf(
    "Checking Obstacle %f, %f, %f, %f, %f, in map %s dx: %f  dy: %f\n", 
    obs.x, obs.y, obs.radius, obs.dx, obs.dy, map.c_str(), dx, dy
  );

  h2d::CPolyline map_poly;
  if (map == "rectangle"){
    Obstacle tmp = {0.0, 0.0, 0.0, dx, dy, 0.0, OBSTACLE_TYPE::BOX};
    map_poly = obs_to_cpoly(tmp);
  }
  else if (map == "hexagon"){
    std::vector<h2d::Point2d> vertexes;
    vertexes.push_back(h2d::Point2d(    -dx,             0.0));
    vertexes.push_back(h2d::Point2d(-dx/2.0, -sqrt(3)/2.0*dx));
    vertexes.push_back(h2d::Point2d( dx/2.0, -sqrt(3)/2.0*dx));
    vertexes.push_back(h2d::Point2d(     dx,             0.0));
    vertexes.push_back(h2d::Point2d( dx/2.0,  sqrt(3)/2.0*dx));
    vertexes.push_back(h2d::Point2d(-dx/2.0,  sqrt(3)/2.0*dx));
    map_poly = h2d::CPolyline(vertexes);
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Map %s not recognized.", map.c_str());
    inside = false;
  }

  if (obs.type == OBSTACLE_TYPE::CYLINDER){
    h2d::Circle obs_circle(obs_to_circle(obs));
    if (obs_circle.isInside(map_poly) && obs_circle.intersects(map_poly).size() == 0) {
      inside = true;
    }
  }
  else if (obs.type == OBSTACLE_TYPE::BOX){
    h2d::CPolyline obs_rect = obs_to_cpoly(obs);

    std::cout << "Is inside map? " << obs_rect.isInside(map_poly) << std::endl;
    std::cout << "Intersects with map? " << obs_rect.intersects(map_poly).size() << std::endl;

    if (obs_rect.isInside(map_poly) && obs_rect.intersects(map_poly).size() == 0) {
      inside = true;
    }
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Obstacle type %d not recognized.", obs.type);
    inside = false;
  }  

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Result: %s", (inside ? "inside" : "NOT inside"));

  std::cout << "Result: " << (inside ? "inside" : "NOT inside") << std::endl;

  return inside;
}

std::vector<h2d::Point2d> create_hexagon_v(double dx){
  std::vector<h2d::Point2d> vertexes;
  vertexes.push_back(h2d::Point2d(-dx/2.0,  sqrt(3)/2.0*dx));
  vertexes.push_back(h2d::Point2d( dx/2.0,  sqrt(3)/2.0*dx));
  vertexes.push_back(h2d::Point2d(     dx,             0.0));
  vertexes.push_back(h2d::Point2d( dx/2.0, -sqrt(3)/2.0*dx));
  vertexes.push_back(h2d::Point2d(-dx/2.0, -sqrt(3)/2.0*dx));
  vertexes.push_back(h2d::Point2d(    -dx,             0.0));
  return vertexes;
}

geometry_msgs::msg::Polygon create_hexagon(double dx){
  geometry_msgs::msg::Polygon pol;
  std::vector<geometry_msgs::msg::Point32> points_temp;
  
  std::vector<h2d::Point2d> vertexes = create_hexagon_v(dx);
  for (auto vertex : vertexes){
    geometry_msgs::msg::Point32 point;
    point.x = vertex.getX();
    point.y = vertex.getY();
    point.z = 0;
    points_temp.push_back(point);
  }
  
  pol.points = points_temp;
  return pol;
}


std::vector<h2d::Point2d> create_rectangle_v(double dx, double dy, double x, double y, double yaw){
  // The vertices of the rectangle as if there were no rotation
  std::vector<h2d::Point2d> vertexes = {
    h2d::Point2d(x - dx/2.0, y + dy/2.0),
    h2d::Point2d(x - dx/2.0, y - dy/2.0),
    h2d::Point2d(x + dx/2.0, y - dy/2.0),
    h2d::Point2d(x + dx/2.0, y + dy/2.0)
  };

  // If there is a rotation, rotate
  if (yaw != 0){
    h2d::CPolyline poly = h2d::CPolyline(vertexes);
    rotate(poly, yaw, x, y);
    for (size_t i = 0; i<vertexes.size(); i++){
      vertexes[i] = poly.getPoint(i);
    }
  }
  
  return vertexes;
}

geometry_msgs::msg::Polygon create_rectangle(double dx, double dy, double x, double y, double yaw){
  geometry_msgs::msg::Polygon pol;
  std::vector<geometry_msgs::msg::Point32> points_temp;

  std::vector<h2d::Point2d> vertexes = create_rectangle_v(dx, dy, x, y, yaw);
  for (auto vertex : vertexes){
    geometry_msgs::msg::Point32 point;
    point.x = vertex.getX();
    point.y = vertex.getY();
    point.z = 0;
    points_temp.push_back(point);
  }

  pol.points = points_temp;
  return pol;
}

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
)
{
  bool res = is_inside_map(obs, map, dx, dy);
  std::cout << "[valid_position] Is inside map? " << res << std::endl;

  for (size_t i=0; i<others.size() && res; i++){
    for (size_t j=0; j<others[i].size() && res; j++){
      res &= !overlaps(obs, others[i][j]);
      std::cout << "[valid_position] Overlaps with " << i << ", " << j << "? " << !res << std::endl;
      std::cout << "\t" << obs << "\n\t" << others[i][j] << std::endl;
    }
  }
  return res;
}

