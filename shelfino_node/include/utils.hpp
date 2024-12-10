#pragma once

#include <vector>
// #include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include "robotstatus.h"
#include "json.hpp"

typedef double real_type;

static const int CAM_WIDTH_SMALL  = 800;
static const int CAM_HEIGHT_SMALL = CAM_WIDTH_SMALL*9/16;
static const int CAM_FPS_SMALL    = 30;
static bool ROBOT_CONNECTED = true;


struct Point2d {
  Point2d() {}
  Point2d(real_type x, real_type y): x(x), y(y) {}

  friend bool operator==(Point2d const & p1, Point2d const & p2) { return p1.x==p2.x && p1.y==p2.y; }
  friend bool operator!=(Point2d const & p1, Point2d const & p2) { return !(p1==p2); }

  real_type x, y;
};

struct Pose {
  real_type x, y, theta;
};

typedef Pose State2d;

typedef std::vector<Point2d> Obstacle;

struct ScanData
{
  std::vector<RobotStatus::LidarData> data;
  Pose pose;
};


struct Wayline {
  real_type xc, yc;
  real_type theta;
  real_type L;
  size_t npts;

  //vector<real_type> _x, _y;

//public:
  Wayline(real_type xc, 
          real_type yc,
          real_type theta,
          real_type L,
          int npts):
    xc(xc), yc(yc), theta(theta), L(L), npts(npts)        
  {}

  size_t size() const {
    return 2*npts + 1;
  }

  
  real_type x(int idx) const {
    if (!npts) return xc;

    real_type ds = L/npts;
    real_type cos_theta = cos(theta);
    real_type sin_theta = sin(theta);
    
    // 0 ->  0
    // 1 ->  1, 2 -> -1
    // 3 ->  2, 4 -> -2
    // 5 ->  3, 6 -> -3
    int sign = idx%2==0 ? -1 : +1;
    idx = (idx+1)/2;
    real_type cl = sign*idx*ds;
    return xc+cl*cos_theta;
  } 
  
  real_type y(int idx) const { 
    if (!npts) return yc;

    real_type ds = L/npts;
    real_type cos_theta = cos(theta);
    real_type sin_theta = sin(theta);
    
    // 0 ->  0
    // 1 ->  1, 2 -> -1
    // 3 ->  2, 4 -> -2
    // 5 ->  3, 6 -> -3
    int sign = idx%2==0 ? -1 : +1;
    idx = (idx+1)/2;
    real_type cl = sign*idx*ds;
    return yc+cl*sin_theta;
  } 

};

struct AngularRange {
  real_type thetac;
  real_type L;
  size_t npts;
  std::vector<real_type> special;

  //vector<real_type> _x, _y;

//public:
  AngularRange(real_type thetac, 
               real_type L,
               int npts):
    thetac(thetac), L(L), npts(npts)        
  {}

  size_t size() const {
    return 2*npts + 1 + special.size();
  }

  real_type theta(int idx) const {
    if (!idx) return thetac;
    if (idx>2*npts) {
      return special[idx-2*npts-1];
    }

    real_type ds = L/npts;
       
    // 0 ->  0
    // 1 ->  1, 2 -> -1
    // 3 ->  2, 4 -> -2
    // 5 ->  3, 6 -> -3
    int sign = idx%2==0 ? -1 : +1;
    idx = (idx+1)/2;
    real_type cl = sign*idx*ds;
    return thetac+cl;
  } 

  void addSpecial(real_type theta) {
    special.push_back(theta);
  }

};

// struct Map
// {
//   double resolution;
//   double x0;
//   double y0;
//   cv::Mat img;

//   void map2world(int i, int j, double & x, double & y) const
//   {
// //    x = (j+0.5)*resolution + x0;
// //    y = (i+0.5)*resolution + y0;
//     x = (j)*resolution + x0;
//     y = (i)*resolution + y0;
//   }

//   void world2map(double x, double y, int & i, int & j) const
//   {
// //    i = (y-y0)/resolution+0.5;
// //    j = (x-x0)/resolution+0.5;
//     i = std::round((y-y0)/resolution);
//     j = std::round((x-x0)/resolution);
//   }
// };

inline
int Signum(double x){
  if(x>0){
    return 1.0;
  }else if(x<0){
    return -1.0;
  }else{
    return 0;
  }
}

inline
bool getObstacles(nlohmann::json const & jdata, std::vector<Obstacle>& obstacles)
{
  using nlohmann::json;

  obstacles.clear();

  double x, y;
    
  try 
  {
    json j_arr_boundaries = jdata.at("boundaries");
    json j_arr_obstacles  = jdata.at("obstacles");
  
    Obstacle curr;
    for (json::iterator it = j_arr_boundaries.begin(); it != j_arr_boundaries.end(); ++it) 
    {      
      x = (*it).at("x");
      y = (*it).at("y"); 
      curr.push_back({x, y});
    }
    if (curr.back() != curr.front())
      curr.push_back(curr.front());
    obstacles.push_back(curr);

    for (json::iterator it = j_arr_obstacles.begin(); it != j_arr_obstacles.end(); ++it) 
    {
      Obstacle curr;     

      json tmp_obstacles = (*it).at("data");
      for (json::iterator itt = tmp_obstacles.begin(); itt != tmp_obstacles.end(); ++itt) 
      {
        curr.push_back({(*itt).at("x"), (*itt).at("y")});
      }
      if (curr.back() != curr.front())
        curr.push_back(curr.front());
      obstacles.push_back(curr);
    }
  } 
  catch (std::exception & ex)
  {
    std::cerr << ex.what() << std::endl;
    return false;
  }

  return true;
}

inline
bool getObstacles(std::string const & filename, std::vector<Obstacle> & map) 
{
  map.clear();

  std::ifstream input(filename);
  if (!input.is_open())
  {
    std::cerr << "Error opening file " << filename << std::endl;
    return false;
  }

  try 
  {
    std::string str((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    nlohmann::json jdata = nlohmann::json::parse(str);
    return getObstacles(jdata, map);
  } 
  catch (...)
  {}

  input.clear();                 // clear fail and eof bits
  input.seekg(0, std::ios::beg); // back to the start!
  
  const std::string PARSE_ERROR_MSG = "Error while parsing map file";

  while (!input.eof())
  {
    int size;
    if (!(input >> size))
    {
      if (input.eof())
        break;
      std::cerr << PARSE_ERROR_MSG << std::endl;
      return false;
    }

    Obstacle curr;
    for (int i=0; i<size; ++i)
    {
      double x, y;
      if (!(input >> x >> y))
      {
        std::cerr << PARSE_ERROR_MSG << std::endl;
        return false;
      }

      curr.push_back({x, y});
    }

    if (curr.back()!=curr.front())
    {
      curr.push_back(curr.front());
    }

    map.push_back(curr);
  }
  
  return true;
}

inline
std::vector<Obstacle> getObstacles(std::string const & filename) {
  std::vector<Obstacle> res;
  bool ok = getObstacles(filename, res);
  if (!ok) {
    res.clear();
  }
  return res;
}

