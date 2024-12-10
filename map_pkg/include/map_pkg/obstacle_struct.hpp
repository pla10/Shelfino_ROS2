#pragma once

#include <ostream>
#include <string> 
#include <vector>

enum OBSTACLE_TYPE {
  CYLINDER,
  BOX
};

struct Obstacle {
  double radius;
  double x, y;
  double dx, dy;
  double yaw;
  OBSTACLE_TYPE type;
  std::string xml_file = "";
 
  friend std::ostream& operator<<(std::ostream& os, const Obstacle& obs){
    os << "Obstacle: ";
    if (obs.type == OBSTACLE_TYPE::CYLINDER){
      os << "Cylinder: ";
      os << "x: " << obs.x << " y: " << obs.y << " radius: " << obs.radius;
    }
    else if (obs.type == OBSTACLE_TYPE::BOX){
      os << "Box: ";
      os << "x: " << obs.x << " y: " << obs.y << " dx: " << obs.dx << " dy: " << obs.dy << " yaw: " << obs.yaw;
    }
    else{
      os << "Unknown type";
    }
    return os;
  }
};

struct Victim : public Obstacle {
  Victim(double _x, double _y, double _r = 0.5) : Obstacle() {
    this->x = _x;
    this->y = _y;
    this->radius = _r;
    this->yaw = 0.0;
    this->type = OBSTACLE_TYPE::CYLINDER;
  }
};
