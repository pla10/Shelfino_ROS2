#ifndef PERSONITEM_H
#define PERSONITEM_H

#include <json.hpp>

//struct Pose{
//  double x = 0;
//  double y = 0;
//  double theta = 0;
//  double v = 0;
//  double omega = 0;
//  double t = 0;
//};

struct LidarScans{
  double x = 0;
  double y = 0;
  bool isSafe = true;
  int zoneAssociated = -1;
};

class Cone
{
public:
  double min_angle;
  double max_angle;
  std::string type;

  Cone(){
    min_angle = 0;
    max_angle = 0;
    type = "";
  }

  Cone(const Cone& pi){
    min_angle = pi.min_angle;
    max_angle = pi.max_angle;
    type = pi.type;
  }
};

class PersonItem
{

public:
  double x;
  double y;
  double theta;
  int id;
  bool isTracking;
  bool is_person;
  std::string type;
  int probability;

  PersonItem(){
    id = 0;
    x=0;
    y = 0;
    theta = 0;
    isTracking = false;
    is_person = 0;
    type = "";
    probability = 0;
  }


  //! Class constructor element by element
  PersonItem(const PersonItem& pi){
    id = pi.id;
    x = pi.x;
    y = pi.y;
    theta = pi.theta;
    isTracking = pi.isTracking;
    is_person = pi.is_person;
    type = pi.type;
    probability = pi.probability;
  }

  void fromJson(const nlohmann::json &j){
    x = j.at("x");
    y = j.at("y");
    theta = j.at("theta");
    id    = j.at("id");
    isTracking = false;
    type = j.at("type").get<std::string>();
    probability = j.at("probability");
  }


};

#endif // PERSONITEM_H
