#include "map_pkg/utilities.hpp"

std::vector<Point> create_hexagon_v(double dx){
  std::vector<Point> vertexes;
  vertexes.push_back(std::make_tuple(-dx/2.0,  sqrt(3)/2.0*dx));
  vertexes.push_back(std::make_tuple( dx/2.0,  sqrt(3)/2.0*dx));
  vertexes.push_back(std::make_tuple(     dx,             0.0));
  vertexes.push_back(std::make_tuple( dx/2.0, -sqrt(3)/2.0*dx));
  vertexes.push_back(std::make_tuple(-dx/2.0, -sqrt(3)/2.0*dx));
  vertexes.push_back(std::make_tuple(    -dx,             0.0));
  return vertexes;
}

geometry_msgs::msg::Polygon create_hexagon(double dx){
  geometry_msgs::msg::Polygon pol;
  std::vector<geometry_msgs::msg::Point32> points_temp;

  std::vector<Point> vertexes = create_hexagon_v(dx);
  for (auto vertex : vertexes){
    geometry_msgs::msg::Point32 point;
    point.x = std::get<0>(vertex);
    point.y = std::get<1>(vertex);
    point.z = 0;
    points_temp.push_back(point);
  }
  
  pol.points = points_temp;
  return pol;
}


std::vector<Point> create_rectangle_v(double dx, double dy, double x, double y, double yaw){
  // The vertices of the rectangle as if there were no rotation
  std::vector<Point> vertexes = {
    std::make_tuple(x - dx/2.0, y + dy/2.0),
    std::make_tuple(x - dx/2.0, y - dy/2.0),
    std::make_tuple(x + dx/2.0, y - dy/2.0),
    std::make_tuple(x + dx/2.0, y + dy/2.0)
  };

  // If there is a rotation, rotate w.r.t. the center of the rectangle
  if (yaw != 0){
    double x_center = x;
    double y_center = y;
    for (auto& vertex : vertexes){
      double x_temp = std::get<0>(vertex);
      double y_temp = std::get<1>(vertex);
      double x_rot = (x_temp - x_center)*cos(yaw) - (y_temp - y_center)*sin(yaw) + x_center;
      double y_rot = (x_temp - x_center)*sin(yaw) + (y_temp - y_center)*cos(yaw) + y_center;
      vertex = std::make_tuple(x_rot, y_rot);
    }
  }
  
  return vertexes;
}

geometry_msgs::msg::Polygon create_rectangle(double dx, double dy, double x, double y, double yaw){
  geometry_msgs::msg::Polygon pol;
  std::vector<geometry_msgs::msg::Point32> points_temp;

  std::vector<Point> vertexes = create_rectangle_v(dx, dy, x, y, yaw);
  for (auto vertex : vertexes){
    geometry_msgs::msg::Point32 point;
    point.x = std::get<0>(vertex);
    point.y = std::get<1>(vertex);
    point.z = 0;
    points_temp.push_back(point);
  }

  pol.points = points_temp;
  return pol;
}