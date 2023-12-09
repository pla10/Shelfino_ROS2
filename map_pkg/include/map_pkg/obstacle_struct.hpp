#pragma once

#include <string> 
#include <vector>

enum obstacle_type {
  CYLINDER,
  BOX
};

struct obstacle {
  double radius;
  double x, y;
  double dx, dy;
  obstacle_type type;
  std::string xml_file = "";
};

struct victim : public obstacle {
  victim(double _x, double _y) : obstacle() {
    this->x = _x;
    this->y = _y;
    this->radius = 0.5;
    this->type = obstacle_type::CYLINDER;
  }
};
