#include "hardwareglobalinterface.h"

#include <json.hpp>
#include <chrono>
#include "utils.hpp"
#include <cstdlib>

std::vector<HardwareGlobalInterface *> HardwareGlobalInterface::instances;

static inline double isLeft(Point2d const & P0, Point2d const & P1, Point2d const & P2)
{
  return ((P1.x - P0.x) * (P2.y - P0.y) -(P2.x - P0.x) * (P1.y - P0.y));
}

static bool contain(Point2d const & point, std::vector<Point2d> const & vertices)
{
  int wn = 0; // the winding number counter

  // loop through all edges of the polygon
  for (int i=1; i<vertices.size(); ++i)
  {
    // edge from V[i] to V[i+1]
    if (vertices.at(i-1).y <= point.y)
    {
      // start y <= P.y
      if (vertices.at(i).y > point.y) // an upward crossing
        if (isLeft(vertices.at(i-1), vertices.at(i), point) > 0) // P left of edge
          ++wn; // have a valid up intersect
    }
    else
    {
      // start y > P.y (no test needed)
      if (vertices.at(i).y <= point.y) // a downward crossing
        if (isLeft(vertices.at(i-1), vertices.at(i), point) < 0) // P right of edge
          --wn; // have a valid down intersect
    }
  }
  return wn != 0;
}

HardwareGlobalInterface::~HardwareGlobalInterface()
{

  subLoc->stop();
  subHW->stop();
  subFrontLidar->stop();
  subRearLidar->stop();
}


HardwareGlobalInterface::HardwareGlobalInterface()
{

}

HardwareGlobalInterface::HardwareGlobalInterface(HardwareParameters* hp)
{
  params = hp;
}

void HardwareGlobalInterface::sub_locSubscriber(const char *topic, const char *buf, size_t size, void *data)
{
  nlohmann::json j;

  std::unique_lock<std::mutex> lock(locDataMTX);

  try{
    j = nlohmann::json::parse(std::string(buf, size));

    locData.x = j.at("state").at("x");
    locData.y = j.at("state").at("y");
    locData.theta = j.at("state").at("theta");
    locData.locTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
  catch(std::exception &e){
    std::cerr << "error parsing loc data: " << e.what() << std::endl;
  }
}

void HardwareGlobalInterface::subHW_callback(const char *topic, const char *buf, size_t size, void *data)
{
  std::unique_lock<std::mutex> lock(hwDataMTX);

  nlohmann::json j;
  try{
    j = nlohmann::json::parse(std::string(buf, size));
    //std::cout << j.dump() << std::endl;
    double omega_r = j.at("3").at("state").at("vel");
    double omega_l = j.at("4").at("state").at("vel");
    double i_r = j.at("3").at("state").at("cur");
    double i_l = j.at("4").at("state").at("cur");
    double tick_r = j.at("3").at("state").at("tck");
    double tick_l = j.at("4").at("state").at("tck");

    
    hwData.rightWheel.omega = -omega_r;
    hwData.rightWheel.current = i_r;
    hwData.rightWheel.ticks = -tick_r/1024*2*M_PI; //RIGHT_INCREMENTS_PER_TOUR*2*M_PI;

    hwData.leftWheel.omega = omega_l;
    hwData.leftWheel.current = i_l;
    hwData.leftWheel.ticks = tick_l/1024*2*M_PI; //LEFT_INCREMENTS_PER_TOUR*2*M_PI;

    if(this->id == 1 || this->id == 3){
      hwData.speed = (LEFT_RADIUS/2.0)*(omega_r-omega_l);
      hwData.omega = (LEFT_RADIUS/0.4)*(-omega_r-omega_l);

      hwData.hardwareTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      updateOdometryV(hwData.hardwareTimer, -omega_l, omega_r);
  } else {
      hwData.speed = (LEFT_RADIUS/2.0)*(omega_l-omega_r);
      hwData.omega = (LEFT_RADIUS/0.4)*(-omega_r-omega_l);

      hwData.hardwareTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      updateOdometryV(hwData.hardwareTimer, omega_l, -omega_r);
  }

  }catch(std::exception &e){

  }
}

void HardwareGlobalInterface::subHW_trolley(const char *topic, const char *buf, size_t size, void *data)
{
  std::unique_lock<std::mutex> lock(trolleyDataMTX);

  nlohmann::json j;
  try{
    j = nlohmann::json::parse(std::string(buf, size));
    //std::cout << j.dump() << std::endl;
    trolleyData.angle = j.at("2").at("state").at("pos");
    trolleyData.force = j.at("2").at("state").at("for");
    trolleyData.powerEnable = j.at("2").at("state").at("powerEnable");

    trolleyData.trolleyTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  }catch(std::exception &e){

  }
}
/*
void HardwareGlobalInterface::subTracker_callback(const char *topic, const char *buf, size_t size, void *data){
    nlohmann::json j;

    try{
        j = nlohmann::json::parse(std::string(buf, size));

        //std::cout << j.dump() << std::endl;
        nlohmann::json j_data = j.at("object_tracked");

        peopleVector.clear();

        nlohmann::json j_point;

        QString type;
        for (nlohmann::json::iterator it = j_data.begin(); it != j_data.end(); ++it) {
           // type = QString::fromStdString((*it).at("type").get<std::string>());

            PersonItem person;
            person.x       = (*it).at("x");
            person.y       = (*it).at("y");
            person.theta   = (*it).at("theta");
            person.id      = (*it).at("id");

            if(j.at("tracking_id")==person.id){
                person.isTracking = true;
            }else{
                person.isTracking = false;
            }

            peopleVector.push_back(person);

        }

        std::unique_lock<std::mutex> lockObstacle(obstacleMutex);
        if(j.at("tracking_id")==-1){
            obstacle1.isPresent = false;
        }else{
            for (std::vector<PersonItem>::iterator it = peopleVector.begin() ; it != peopleVector.end(); ++it){
                if((*it).id == j.at("tracking_id")){

                    obstacle1.pose[0] = (*it).x;
                    obstacle1.pose[1] = (*it).y;
                    obstacle1.theta = (*it).theta;
                    obstacle1.vel[0] = 0;
                    obstacle1.vel[1] = 0;
                    obstacle1.vel[1] = 0;
                    obstacle1.vel[1] = 0;
                    obstacle1.isPresent = true;
                }
            }
        }
        int count_obs = 0;
        obstacle2.vel.resize(2,peopleVector.size());
        obstacle2.pose.resize(2,peopleVector.size());
        obstacle2.theta.resize(peopleVector.size());
        obstacle2.id.resize(peopleVector.size());
        for (std::vector<PersonItem>::iterator it = peopleVector.begin() ; it != peopleVector.end(); ++it){

            obstacle2.pose(0,count_obs) = (*it).x;
            obstacle2.pose(1,count_obs) = (*it).y;
            obstacle2.theta(count_obs)  = (*it).theta;
            obstacle2.vel(0,count_obs)  = 0;
            obstacle2.vel(1,count_obs)  = 0;
            obstacle2.id(count_obs)     = (*it).id;
                count_obs++;
                //obstacle1.isPresent = true;
        }
        for(int n;n=0;n<2){
            RobotStatus::LocalizationData neigPose;
            HardwareGlobalInterface::getInstance(n).getLocalizationDataKarto(neigPose);
            obstacle2.pose(0,count_obs) = neigPose.x;
            obstacle2.pose(1,count_obs) = neigPose.y;
            obstacle2.theta(count_obs)  = 0;
            obstacle2.vel(0,count_obs)  = 0;
            obstacle2.vel(1,count_obs)  = 0;
            obstacle2.id(count_obs)     = 99;
            count_obs++;
        }
        lockObstacle.unlock();
        navigatorPainter->drawPersons(peopleVector);

        try {
            nlohmann::json j_cone = j.at("cones");

            std::vector<Cone> cones;
            cones.clear();
            for (nlohmann::json::iterator it = j_cone.begin(); it != j_cone.end(); ++it) {
                if((*it).at("type")!="person") continue;
                Cone cone;
                cone.min_angle = (*it).at("min_angle");
                cone.max_angle = (*it).at("max_angle");
                cones.push_back(cone);
            }
        } catch (std::exception &e) {
        }


        //Get the lines
        try {
            nlohmann::json j_lines = j.at("lines");


            std::vector<QLineF> linesVector;
            for (nlohmann::json::iterator it = j_lines.begin(); it != j_lines.end(); ++it) {
                double x1 = (*it).at("x1");
                double x2 = (*it).at("x2");
                double y1 = (*it).at("y1");
                double y2 = (*it).at("y2");
                QLineF tmp(x1,y1,x2,y2);
                linesVector.push_back(tmp);
            }

            //navigatorPainter->drawLines(linesVector);

        } catch (std::exception &e) {
        }

        try {
            nlohmann::json j_lines = j.at("lines_starting");


            std::vector<QLineF> linesVector;
            for (nlohmann::json::iterator it = j_lines.begin(); it != j_lines.end(); ++it) {
                double x1 = (*it).at("x1");
                double x2 = (*it).at("x2");
                double y1 = (*it).at("y1");
                double y2 = (*it).at("y2");
                QLineF tmp(x1,y1,x2,y2);
                linesVector.push_back(tmp);
            }

            //navigatorPainter->drawLinesOld(linesVector);

        } catch (std::exception &e) {
        }

        try {
            nlohmann::json j_cat = j.at("category");
            //std::cout << j_cat.dump() << std::endl;
            std::vector<std::vector<QLineF>> linesCategory;
            int cnt = 0;
            for (nlohmann::json::iterator it = j_cat.begin(); it != j_cat.end(); ++it) {
                nlohmann::json j_lines = j_cat.at(std::to_string(cnt));
                std::vector<QLineF> linesVector;
                for (nlohmann::json::iterator it1 = j_lines.begin(); it1 != j_lines.end(); ++it1) {
                    double x1 = (*it1).at("x1");
                    double x2 = (*it1).at("x2");
                    double y1 = (*it1).at("y1");
                    double y2 = (*it1).at("y2");
                    QLineF tmp(x1,y1,x2,y2);
                    linesVector.push_back(tmp);
                }
                linesCategory.push_back(linesVector);
                cnt++;
            }
            //navigatorPainter->drawVectorLines(linesCategory);


        } catch (std::exception &e) {
        }


    }catch(std::exception &e){
        //std::cerr<<e.what()<<std::endl;
    }

}
*/

void HardwareGlobalInterface::sub_tracking(const char *topic, const char *buf, size_t size, void *data){
  nlohmann::json j;
  std::vector<PersonItem> peopleVector;

  try {
    j = nlohmann::json::parse(std::string(buf, size));

    //std::cout << j.dump() << std::endl;
    nlohmann::json j_data = j.at("object_tracked");

    peopleVector.clear();

    nlohmann::json j_point;

    std::string type;
    for (nlohmann::json::iterator it = j_data.begin(); it != j_data.end(); ++it) {
      PersonItem person;
      person.x       = (*it).at("x");
      person.y       = (*it).at("y");
      person.theta   = (*it).at("theta");
      person.id      = (*it).at("id");

      if(j.at("tracking_id")==person.id){
        person.isTracking = true;
      }else{
        person.isTracking = false;
      }

      peopleVector.push_back(person);
    }

    {
      std::unique_lock<std::mutex> lockTracking(trackingMTX);
      trackingData = peopleVector;
      trackingTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }
  }catch(std::exception &e){
    //std::cerr<<e.what()<<std::endl;
  }
}

void HardwareGlobalInterface::updateOdometryV(double current_time, double vl, double vr)
{
  std::unique_lock<std::mutex> lock(odomMTX);

  bool firstTime = !wsLeft.init;
  double seconds_since_last_update = (current_time - wsLeft.lastTime)/1000;

  if(seconds_since_last_update < 0.005) {
    std::cout << "updateOdometryV quit because too early" << std::endl;
    return;
  }

  if (!wsLeft.init) {
    wsLeft.dTick = 0;
    //wsLeft.lastTick = pL;
    wsLeft.lastTime = current_time;
    wsLeft.init = true;
  }
  else {
    //wsLeft.dTick = (vl*36/(2*M_PI)*2048)*(current_time-wsLeft.lastTime)/1000; //pL-wsLeft.lastTick;
    wsLeft.dTick = (vl/(2*M_PI)*this->encoder_ppr)*(current_time-wsLeft.lastTime)/1000; //pL-wsLeft.lastTick;
    // std::cout << this->encoder_ppr << std::endl;
    //wsLeft.lastTick = pL;
    wsLeft.lastTime = current_time;
  }

  if (!wsRight.init) {
    wsRight.dTick = 0;
    //wsRight.lastTick = pR;
    wsRight.lastTime = current_time;
    wsRight.init = true;
  }
  else {
    //wsRight.dTick = (vr*36/(2*M_PI)*2048)*(current_time-wsRight.lastTime)/1000; //pR-wsRight.lastTick;
    wsRight.dTick = (vr/(2*M_PI)*this->encoder_ppr)*(current_time-wsRight.lastTime)/1000; //pR-wsRight.lastTick;
    //wsRight.lastTick = pR;
    wsRight.lastTime = current_time;
  }
  
  if (firstTime) {
    return;
  };
  
  double ticks_l = wsLeft.dTick;
  double ticks_r = wsRight.dTick;
  double df = 0.5 * (ticks_l/LEFT_INCREMENTS_PER_TOUR * 2*M_PI * LEFT_RADIUS + ticks_r/RIGHT_INCREMENTS_PER_TOUR * 2*M_PI * RIGHT_RADIUS);

  odomData.twist_lin_x = df/seconds_since_last_update;
}

void HardwareGlobalInterface::updateOdometryOmega(double current_time, std::vector<double> const & orientation, std::vector<double> const & angular_vel){
  std::unique_lock<std::mutex> lock(odomMTX);

  double seconds_since_last_update = (current_time - odomData.last_odom_update)/1000;
  odomData.last_odom_update = current_time;

  if(seconds_since_last_update < 0.005) {
    return;
  };

  std::vector<double> angles = ToEulerAngles(orientation);  
  angles.at(2) = -angles.at(2); // flip sign
  double dtheta = angles.at(2) - odomData.theta;
  double df = odomData.twist_lin_x * seconds_since_last_update; 
  if (dtheta != 0)
    dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta)); // angdiff?
  odomData.twist_ang_z = dtheta/seconds_since_last_update;

  double dx, dy;
  double theta_k, theta_k1;
  theta_k = odomData.theta;
  theta_k1 = odomData.theta + dtheta;
  if (std::abs(odomData.twist_ang_z)>1e-6) {
    dx = odomData.twist_lin_x/odomData.twist_ang_z*(std::sin(theta_k1)-std::sin(theta_k));
    dy = -odomData.twist_lin_x/odomData.twist_ang_z*(std::cos(theta_k1)-std::cos(theta_k));
  }
  else {
    dx = df*std::cos(theta_k + 0.5*dtheta);
    dy = df*std::sin(theta_k + 0.5*dtheta);
  }

  odomData.pos_x += dx;
  odomData.pos_y += dy;
  odomData.theta += dtheta;

  double cy = cos(odomData.theta * 0.5);
  double sy = sin(odomData.theta * 0.5);
  double cp = cos(0 * 0.5);
  double sp = sin(0 * 0.5);
  double cr = cos(0 * 0.5);
  double sr = sin(0 * 0.5);
  odomData.orient_x = sr * cp * cy - cr * sp * sy;
  odomData.orient_y = cr * sp * cy + sr * cp * sy;
  odomData.orient_z = cr * cp * sy - sr * sp * cy;
  odomData.orient_w = cr * cp * cy + sr * sp * sy;

}

std::vector<double> HardwareGlobalInterface::ToEulerAngles(std::vector<double> const & q) {
  double angles[3];
  //std::cout << "+++ x: " << q.at(0) << "| y: " << q.at(1) << "| z: " << q.at(2) << "| w: " << q.at(3) << std::endl;

  double test = q.at(0)*q.at(1) + q.at(2)*q.at(3);
  if (test > 0.499) { // singularity at north pole
    angles[0] = 2 * std::atan2(q.at(0),q.at(3));
    angles[1] = M_PI/2;
    angles[2] = 0;
    std::vector<double> dest(std::begin(angles), std::end(angles));
    //std::cout << "-1- x: " << q.at(0) << "| y: " << q.at(1) << "| z: " << q.at(2) << std::endl;
    return dest;
  }
  if (test < -0.499) { // singularity at south pole
    angles[0] = -2 * std::atan2(q.at(0),q.at(3));
    angles[1] = - M_PI/2;
    angles[2] = 0;
    std::vector<double> dest(std::begin(angles), std::end(angles));
    //std::cout << "-2- x: " << dest.at(0) << "| y: " << dest.at(1) << "| z: " << dest.at(2) << std::endl;
    return dest;
  }
  double sqx = q.at(0)*q.at(0);
  double sqy = q.at(1)*q.at(1);
  double sqz = q.at(2)*q.at(2);
  angles[0] = std::atan2(2*q.at(1)*q.at(3)-2*q.at(0)*q.at(2) , 1 - 2*sqy - 2*sqz);
  angles[1] = std::asin(2*test);
  angles[2] = std::atan2(2*q.at(0)*q.at(3)-2*q.at(1)*q.at(2) , 1 - 2*sqx - 2*sqz);
  std::vector<double> dest(std::begin(angles), std::end(angles));
  //std::cout << "-3- x: " << dest.at(0) << "| y: " << dest.at(1) << "| z: " << dest.at(2) << std::endl;
  return dest;

}

void HardwareGlobalInterface::sub_realSenseOdom(const char *topic, const char *buf, size_t size, void *data)
{
  std::unique_lock<std::mutex> lock(realSenseOdomMTX);

  nlohmann::json j;

  try{
    j = nlohmann::json::parse(std::string(buf, size));
    //std::cout << j.dump() << std::endl;

    realSenseOdom.pose_cov.clear();
    realSenseOdom.twist_cov.clear();

    realSenseOdom.last_odom_update = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    realSenseOdom.pos_x=j.at("pose").at("pose").at("position").at("x");
    realSenseOdom.pos_y=j.at("pose").at("pose").at("position").at("y");
    realSenseOdom.pos_z=j.at("pose").at("pose").at("position").at("z");
    realSenseOdom.orient_x=j.at("pose").at("pose").at("orientation").at("x");
    realSenseOdom.orient_y=j.at("pose").at("pose").at("orientation").at("y");
    realSenseOdom.orient_z=j.at("pose").at("pose").at("orientation").at("z");
    realSenseOdom.orient_w=j.at("pose").at("pose").at("orientation").at("w");
    realSenseOdom.twist_lin_x=j.at("twist").at("twist").at("linear").at("x");
    realSenseOdom.twist_lin_y=j.at("twist").at("twist").at("linear").at("y");
    realSenseOdom.twist_lin_z=j.at("twist").at("twist").at("linear").at("z");
    realSenseOdom.twist_ang_x=j.at("twist").at("twist").at("angular").at("x");
    realSenseOdom.twist_ang_y=j.at("twist").at("twist").at("angular").at("y");
    realSenseOdom.twist_ang_z=j.at("twist").at("twist").at("angular").at("z");

    std::unique_lock<std::mutex> lock1(odomMTX);
    odomData.pose_cov.clear();
    odomData.twist_cov.clear();
    for(int i=0;i<36;i++){
      realSenseOdom.pose_cov.push_back(j.at("pose").at("covariance")[i]);
      realSenseOdom.twist_cov.push_back(j.at("twist").at("covariance")[i]);

      odomData.pose_cov.push_back(j.at("pose").at("covariance")[i]);
      odomData.twist_cov.push_back(j.at("twist").at("covariance")[i]);
    }
    std::vector<double> cov = std::vector<double>(25, 0.01); // j.at("covariance");
    odomData.pose_cov.at(0) = cov[0]; // cov_x
    odomData.pose_cov.at(7) = cov[6]; // cov_y
    odomData.pose_cov.at(14) = 1000000000000.0;
    odomData.pose_cov.at(21) = 1000000000000.0;
    odomData.pose_cov.at(28) = 1000000000000.0;
    odomData.pose_cov.at(35) = cov[12]; // cov_theta
    
    odomData.twist_cov.at(0) = cov[18]; // cov_v_x
    odomData.twist_cov.at(7) = cov[18]; // cov_v_y
    odomData.twist_cov.at(14) = 1000000000000.0;
    odomData.twist_cov.at(21) = 1000000000000.0;
    odomData.twist_cov.at(28) = 1000000000000.0;
    odomData.twist_cov.at(35) = cov[24]; // cov_omega
    lock1.unlock();

    std::vector<double> orientation, angular_vel;

    orientation.push_back(realSenseOdom.orient_x);
    orientation.push_back(realSenseOdom.orient_y);
    orientation.push_back(realSenseOdom.orient_z);
    orientation.push_back(realSenseOdom.orient_w);
    angular_vel.push_back(realSenseOdom.twist_ang_x);
    angular_vel.push_back(realSenseOdom.twist_ang_y);
    angular_vel.push_back(realSenseOdom.twist_ang_z);

    updateOdometryOmega(realSenseOdom.last_odom_update, orientation, angular_vel);
  }
  catch(std::exception &e){
    //std::cerr << "error parsing: " << e.what() << std::endl;
  }
}


void HardwareGlobalInterface::sub_frontLidar(const char *topic, const char *buf, size_t size, void *data)
{
  std::unique_lock<std::mutex> lock(frontLidarMTX);

  nlohmann::json j;
  static int count = 0;

  try{
    j = nlohmann::json::parse(std::string(buf, size));

    //std::cout << j.dump() << std::endl;
    int size = j.at("size");
    double val;
    double angle = 0;
    double inc = 360.0/((double)(size));
    double angle_offset = 0;

    frontLidar.datum.clear();

    for(int i=0;i<size;i++){
      val = j.at("data")[i];

      angle = count*inc*M_PI/180.0 + angle_offset;
      count++;

      if(val < 0.05){
        //continue;
      }

      //Normalize the angle within 0/2Pi
      angle = atan2(sin(angle),cos(angle));

      RobotStatus::LidarDatum datumLidar;
      datumLidar.angle = angle;
      datumLidar.distance = val;
      datumLidar.x = val*cos(angle) + frontLidar.x_offset;
      datumLidar.y = val*sin(angle) + frontLidar.y_offset;
      datumLidar.isSafe = true;
      frontLidar.datum.push_back(datumLidar);
    }

    frontLidar.lidarTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  }
  catch(std::exception &e){
    //std::cerr << "error parsing: " << e.what() << std::endl;
  }
}

void HardwareGlobalInterface::sub_realSenseLidar(const char *topic, const char *buf, size_t size, void *data)
{
  std::unique_lock<std::mutex> lock(realSenseLidarMTX);

  nlohmann::json j;
  static int count = 0;

  try{
    j = nlohmann::json::parse(std::string(buf, size));

    int size = j.at("size");
    double val;
    double angle = 0;
    double inc = 360.0/((double)(size));
    double angle_offset = 0;

    realSenseLidar.datum.clear();

    //std::cout << j.dump() << std::endl;

    for(int i=0;i<size;i++){
      val = j.at("data")[i];

      angle = count*inc*M_PI/180.0 + angle_offset;
      count++;


      if(val < 0.05 || val > 3){
        continue;
      }

      //Normalize the angle within 0/2Pi
      angle = atan2(sin(angle),cos(angle));

      RobotStatus::LidarDatum datumLidar;
      datumLidar.angle = angle;
      datumLidar.distance = val;
      datumLidar.x = val*cos(angle) + realSenseLidar.x_offset;
      datumLidar.y = val*sin(angle) + realSenseLidar.y_offset;
      datumLidar.isSafe = true;
      realSenseLidar.datum.push_back(datumLidar);
    }

    realSenseLidar.lidarTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  }
  catch(std::exception &e){
    //std::cerr << "error parsing: " << e.what() << std::endl;
  }
}

void HardwareGlobalInterface::sub_rearLidar(const char *topic, const char *buf, size_t size, void *data)
{
  std::unique_lock<std::mutex> lock(rearLidarMTX);

  nlohmann::json j;
  static int count = 0;

  try{
    j = nlohmann::json::parse(std::string(buf, size));
    //std::cout << j.dump() << std::endl;
    int size = j.at("size");
    double val;
    double angle = 0;
    double inc = 360.0/((double)(size));
    double angle_offset = 0;

    rearLidar.datum.clear();

    for(int i=0;i<size;i++){
      val = j.at("data")[i];

      angle = count*inc*M_PI/180.0 + angle_offset;
      count++;

      if(val < 0.05){
        continue;
      }

      //Normalize the angle within 0/2Pi
      angle = atan2(sin(angle),cos(angle));

      double threshold = 15;

      if(angle*180.0/M_PI>=-90+threshold && angle*180.0/M_PI<0){
        continue;
      }
      if(angle*180.0/M_PI>=0 && angle*180.0/M_PI<90-threshold){
        continue;
      }

      RobotStatus::LidarDatum datumLidar;
      datumLidar.angle = angle;
      datumLidar.distance = val;
      datumLidar.x = val*cos(angle) + rearLidar.x_offset;
      datumLidar.y = val*sin(angle) + rearLidar.y_offset;
      datumLidar.isSafe = true;
      rearLidar.datum.push_back(datumLidar);
    }

    rearLidar.lidarTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  }
  catch(std::exception &e){
    //std::cerr << "error parsing: " << e.what() << std::endl;
  }
}

void HardwareGlobalInterface::sub_locSubscriberKarto(const char *topic, const char *buf, size_t size, void *data)
{
  nlohmann::json j;

  std::unique_lock<std::mutex> lock(locDataMTX);

  try{
    j = nlohmann::json::parse(std::string(buf, size));

    locDataKarto.x = j.at("loc_data").at("x");
    locDataKarto.y = j.at("loc_data").at("y");
    locDataKarto.theta = j.at("loc_data").at("theta");
    locDataKarto.locTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
  catch(std::exception &e){
    std::cerr << "SUB_LOC_KARTO" << std::endl;
    std::cerr << "\"" << std::string(buf, size) << "\"" << std::endl;
    std::cerr << "error parsing loc data: " << e.what() << std::endl;
  }
}

void HardwareGlobalInterface::sub_locSubscriberRealSense(const char *topic, const char *buf, size_t size, void *data)
{
  nlohmann::json j;

  std::unique_lock<std::mutex> lock(locDataRealSenseMTX);

  try{
    j = nlohmann::json::parse(std::string(buf, size));

    locDataRealSense.x = j.at("loc_data").at("x");
    locDataRealSense.y = j.at("loc_data").at("y");
    locDataRealSense.theta = j.at("loc_data").at("theta");
    locDataRealSense.locTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
  catch(std::exception &e){
    std::cerr << "SUB_LOC_REALSENSE" << std::endl;
    std::cerr << "\"" << std::string(buf, size) << "\"" << std::endl;
    std::cerr << "error parsing loc data: " << e.what() << std::endl;
  }
}

void HardwareGlobalInterface::getLastInputs(double &v, double &omega)
{
  std::unique_lock<std::mutex> lock(dataMTX);
  v = lastInputVel;
  omega = lastInputOmega;
  lock.unlock();
}

void HardwareGlobalInterface::getBrakingCoefficient(double &bc)
{
  std::unique_lock<std::mutex> lock(dataMTX);
  bc = brakingCoefficient;
  lock.unlock();
}

bool HardwareGlobalInterface::powerEnable(bool val){
  ZMQCommon::RequesterSimple::status_t req_status;
  nlohmann::json j_req;
  bool ret = true;
  if(val){
    j_req["cmd"] = std::string("set_power_enable");
    j_req["enable"] = true;
    std::string response;
    reqHW->request(j_req.dump(),response,req_status);
    // std::cout << j_req.dump()<<std::endl;
    if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
      try{nlohmann::json j_resp = nlohmann::json::parse(response);
        // std::cout << j_resp.dump()<<std::endl;
        if(j_resp.at("ack") == "true"){
        }else{

        }
      }catch(std::exception &e){
        std::cerr << "POWER_ENABLE TRUE" << std::endl;
        std::cerr << "\"" << response << "\"" << std::endl;
        std::cerr<<e.what()<<std::endl;
      }
    }else{
      ret = false;
    }
  }else{
    j_req["cmd"] = std::string("set_power_enable");
    j_req["enable"] = false;
    std::string response;
    reqHW->request(j_req.dump(),response,req_status);
    if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
      try{nlohmann::json j_resp = nlohmann::json::parse(response);
        if(j_resp.at("ack") == "true"){

        }else{

        }
      }catch(std::exception &e){
        std::cerr << "POWER_ENABLE FALSE" << std::endl;
        std::cerr << "\"" << response << "\"" << std::endl;
        std::cerr<<e.what()<<std::endl;
      }
    }else{
      ret = false;
    }
  }
  return ret;
}

void HardwareGlobalInterface::calibrateTrolley()
{
  ZMQCommon::RequesterSimple::status_t req_status;
  nlohmann::json j_req;
  j_req["cmd"] = std::string("calibrate_trolley_angle");
  std::string response;
  reqHW->request(j_req.dump(),response,req_status);
  // std::cout << j_req.dump()<<std::endl;
  if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
    try{nlohmann::json j_resp = nlohmann::json::parse(response);
      // std::cout << j_resp.dump()<<std::endl;
      if(j_resp.at("ack") == "true"){

      }else{
      }
    }catch(std::exception &e){
      std::cerr<<e.what()<<std::endl;
    }
  }else{

  }
}

void HardwareGlobalInterface::robotOnCurrControl()
{
  if (!powerEnable(true)){
    std::cerr << "Robot motors could not be powered up" << std::endl;
    exit(1);
  }
  setDeviceMode(2);
}

void HardwareGlobalInterface::robotOnVelControl()
{
  if (!powerEnable(true)){
    std::cerr << "Robot motors could not be powered up" << std::endl;
    exit(1);
  }
  setDeviceMode(5);
}


void HardwareGlobalInterface::robotOff()
{
  if (!powerEnable(false)){
    std::cerr << "Robot motors could not be turned OFF" << std::endl;
    exit(1);
  }
}


void HardwareGlobalInterface::setDeviceMode(int deviceMode){
  ZMQCommon::RequesterSimple::status_t req_status;
  nlohmann::json j_req;
  j_req["cmd"] = std::string("set_device_mode");
  j_req["device_mode"] = deviceMode;
  std::string response;
  reqHW->request(j_req.dump(),response,req_status);
  // std::cout << j_req.dump()<<std::endl;
  if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
    try{nlohmann::json j_resp = nlohmann::json::parse(response);
      // std::cout << j_resp.dump()<<std::endl;
      if(j_resp.at("ack") == "true"){

      }else{

      }
    }catch(std::exception &e){
      std::cerr << "SET_DEVICE_MODE" << std::endl;
      std::cerr << "\"" << response << "\"" << std::endl;
      std::cerr<<e.what()<<std::endl;
    }
  }else{

  }
}


void HardwareGlobalInterface::vehicleMove(float vel, float omega){

  ZMQCommon::RequesterSimple::status_t req_status;
  nlohmann::json j_req;
  j_req["cmd"] = std::string("move");
  j_req["speed"] = vel;
  j_req["omega"] = omega;

  std::string response;
  reqHW->request(j_req.dump(),response,req_status);
  if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
    try{nlohmann::json j_resp = nlohmann::json::parse(response);

      if(j_resp.at("ack") == "true"){
      }else{
      }
    }catch(std::exception &e){
      std::cout << "VEHICLE_MOVE" << std::endl;
      std::cout << "\"" << response << "\"" << std::endl;
      std::cout << e.what()<<std::endl;
    }
  }else{

  }
}

void HardwareGlobalInterface::vehicleSafeMove(float vel, float omega)
{
  //First of all check possible collisions
  std::vector<RobotStatus::LocalizationData> futurePoses;
  futurePoses.clear();
  RobotStatus::LocalizationData actPose;
  actPose.x = 0;
  actPose.y = 0;
  actPose.theta = 0;
  actPose.locTimer = 0;

  futurePoses.push_back(actPose);
  double dt = 0.5;
  double distanceStar = 0;

  for(double t=0;t<=2.5;t=t+dt){
    double theta_end = futurePoses.back().theta+omega*dt;
    double dx = vel*dt*cos(futurePoses.back().theta+omega*dt/2.0);
    double dy = vel*dt*sin(futurePoses.back().theta+omega*dt/2.0);

    double xk = futurePoses.back().x + dx;
    double yk = futurePoses.back().y  + dy;

    if(t==1.5){
      if(futurePoses.back().x<0){
        distanceStar = sqrt(pow(xk-hb*cos(theta_end),2)+pow(yk-hb*sin(theta_end),2))-hb;
      }else{
        distanceStar = sqrt(pow(xk,2)+pow(yk,2))+ha;
      }
    }

    RobotStatus::LocalizationData nextPose;
    nextPose.x = xk;
    nextPose.y = yk;
    nextPose.theta = theta_end;
    nextPose.locTimer = futurePoses.back().locTimer + dt;

    futurePoses.push_back(nextPose);
  }

  //Now check possible collisions
  double numberZoneGreen = 0;
  double numberZoneYellow = 0;
  double numberZoneRed = 0;
  double numberZoneCritical = 0;
  double closestDistance = 99999;

  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////CHECK WITH THE FRONT LIDAR//////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  RobotStatus::LidarData lidarFrontScans;
  HardwareGlobalInterface::getInstance().getFrontLidarData(lidarFrontScans);
  for (std::vector<RobotStatus::LidarDatum>::iterator itLidar = lidarFrontScans.datum.begin() ; itLidar != lidarFrontScans.datum.end(); ++itLidar){
    //Check  if it is inside the critical zone
    Point2d p1,p2,p3,p4;
    if(futurePoses.at(1).x>0){
      p1 = Point2d(0, -safetyWidth);
      p2 = Point2d(criticalDistance+ha, -safetyWidth);
      p3 = Point2d(criticalDistance+ha, safetyWidth);
      p4 = Point2d(0, safetyWidth);

      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        continue;
      }

    }else if(futurePoses.at(1).x<0){
      p1 = Point2d(-hb, -safetyWidth);
      p2 = Point2d(-criticalDistance-hb, -safetyWidth);
      p3 = Point2d(-criticalDistance-hb, safetyWidth);
      p4 = Point2d(-hb, safetyWidth);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }

    }

    if(futurePoses.at(1).y<-0.05){
      p1 = Point2d(ha, -width/2);
      p2 = Point2d(ha,-criticalDistance- width/2);
      p3 = Point2d(-hb, -criticalDistance- width/2);
      p4 = Point2d(-hb, -width/2);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }
    }else if(futurePoses.at(1).y>0.05){
      p1 = Point2d(ha, width/2);
      p2 = Point2d(ha, criticalDistance + width/2);
      p3 = Point2d(-hb, criticalDistance + width/2);
      p4 = Point2d(-hb, -width/2);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }

    }

    int zoneCount = 1;
    for (std::vector<RobotStatus::LocalizationData>::iterator itPose = futurePoses.begin() ; itPose != futurePoses.end(); ++itPose){

      RobotStatus::LocalizationData p = (*itPose);

      Point2d p1 = Point2d(p.x + ha*cos(p.theta) + safetyWidth*cos(p.theta+M_PI_2), p.y + ha*sin(p.theta) + safetyWidth*sin(p.theta+M_PI_2));
      Point2d p2 = Point2d(p.x + ha*cos(p.theta) - safetyWidth*cos(p.theta+M_PI_2), p.y + ha*sin(p.theta) - safetyWidth*sin(p.theta+M_PI_2));
      Point2d p3 = Point2d(p.x - hb*cos(p.theta) - safetyWidth*cos(p.theta+M_PI_2), p.y - hb*sin(p.theta) - safetyWidth*sin(p.theta+M_PI_2));
      Point2d p4 = Point2d(p.x - hb*cos(p.theta) + safetyWidth*cos(p.theta+M_PI_2), p.y - hb*sin(p.theta) + safetyWidth*sin(p.theta+M_PI_2));

      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){

        double tmp_distance = sqrt(pow((*itLidar).x,2)+pow((*itLidar).y,2));
        if(tmp_distance<closestDistance){
          closestDistance = tmp_distance;
        }

        (*itLidar).isSafe = false;
        if(zoneCount==1 || zoneCount==2){
          numberZoneRed++;
        }else if (zoneCount==3||zoneCount==4){
          numberZoneYellow++;
        }else if(zoneCount==5||zoneCount==6){
          numberZoneGreen++;
        }

        //As soon as the lidar point is associated to a region -> I continue the loop
        break;
      }
      zoneCount++;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////CHECK WITH THE REAL SENSE///////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  RobotStatus::LidarData realSense;
  HardwareGlobalInterface::getInstance().getRealSenseLidarData(realSense);
  for (std::vector<RobotStatus::LidarDatum>::iterator itLidar = realSense.datum.begin() ; itLidar != realSense.datum.end(); ++itLidar){
    //Check  if it is inside the critical zone
    Point2d p1,p2,p3,p4;
    if(futurePoses.at(1).x>0){
      p1 = Point2d(0, -safetyWidth);
      p2 = Point2d(criticalDistance+ha, -safetyWidth);
      p3 = Point2d(criticalDistance+ha, safetyWidth);
      p4 = Point2d(0, safetyWidth);

      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        continue;
      }

    }else if(futurePoses.at(1).x<0){
      p1 = Point2d(-hb, -safetyWidth);
      p2 = Point2d(-criticalDistance-hb, -safetyWidth);
      p3 = Point2d(-criticalDistance-hb, safetyWidth);
      p4 = Point2d(-hb, safetyWidth);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }

    }

    if(futurePoses.at(1).y<-0.05){
      p1 = Point2d(ha, -width/2);
      p2 = Point2d(ha,-criticalDistance- width/2);
      p3 = Point2d(-hb, -criticalDistance- width/2);
      p4 = Point2d(-hb, -width/2);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }
    }else if(futurePoses.at(1).y>0.05){
      p1 = Point2d(ha, width/2);
      p2 = Point2d(ha, criticalDistance + width/2);
      p3 = Point2d(-hb, criticalDistance + width/2);
      p4 = Point2d(-hb, -width/2);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }

    }

    int zoneCount = 1;
    for (std::vector<RobotStatus::LocalizationData>::iterator itPose = futurePoses.begin() ; itPose != futurePoses.end(); ++itPose){

      RobotStatus::LocalizationData p = (*itPose);
      double ha = ha;
      double hb = hb;

      Point2d p1 = Point2d(p.x + ha*cos(p.theta) + safetyWidth*cos(p.theta+M_PI_2), p.y + ha*sin(p.theta) + safetyWidth*sin(p.theta+M_PI_2));
      Point2d p2 = Point2d(p.x + ha*cos(p.theta) - safetyWidth*cos(p.theta+M_PI_2), p.y + ha*sin(p.theta) - safetyWidth*sin(p.theta+M_PI_2));
      Point2d p3 = Point2d(p.x - hb*cos(p.theta) - safetyWidth*cos(p.theta+M_PI_2), p.y - hb*sin(p.theta) - safetyWidth*sin(p.theta+M_PI_2));
      Point2d p4 = Point2d(p.x - hb*cos(p.theta) + safetyWidth*cos(p.theta+M_PI_2), p.y - hb*sin(p.theta) + safetyWidth*sin(p.theta+M_PI_2));

      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){

        double tmp_distance = sqrt(pow((*itLidar).x,2)+pow((*itLidar).y,2));
        if(tmp_distance<closestDistance){
          closestDistance = tmp_distance;
        }

        (*itLidar).isSafe = false;
        if(zoneCount==1 || zoneCount==2){
          numberZoneRed++;
        }else if (zoneCount==3||zoneCount==4){
          numberZoneYellow++;
        }else if(zoneCount==5||zoneCount==6){
          numberZoneGreen++;
        }

        //As soon as the lidar point is associated to a region -> I continue the loop
        break;
      }
      zoneCount++;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////CHECK WITH THE REAR LIDAR///////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  RobotStatus::LidarData rearLidar;
  HardwareGlobalInterface::getInstance().getRearLidarData(rearLidar);
  for (std::vector<RobotStatus::LidarDatum>::iterator itLidar = rearLidar.datum.begin() ; itLidar != rearLidar.datum.end(); ++itLidar){
    //Check  if it is inside the critical zone
    Point2d p1,p2,p3,p4;
    if(futurePoses.at(1).x>0){
      p1 = Point2d(0, -safetyWidth);
      p2 = Point2d(criticalDistance+ha, -safetyWidth);
      p3 = Point2d(criticalDistance+ha, safetyWidth);
      p4 = Point2d(0, safetyWidth);

      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        continue;
      }

    }else if(futurePoses.at(1).x<0){
      p1 = Point2d(-hb, -safetyWidth);
      p2 = Point2d(-criticalDistance-hb, -safetyWidth);
      p3 = Point2d(-criticalDistance-hb, safetyWidth);
      p4 = Point2d(-hb, safetyWidth);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }

    }

    if(futurePoses.at(1).y<-0.05){
      p1 = Point2d(ha, -width/2);
      p2 = Point2d(ha,-criticalDistance- width/2);
      p3 = Point2d(-hb, -criticalDistance- width/2);
      p4 = Point2d(-hb, -width/2);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }
    }else if(futurePoses.at(1).y>0.05){
      p1 = Point2d(ha, width/2);
      p2 = Point2d(ha, criticalDistance + width/2);
      p3 = Point2d(-hb, criticalDistance + width/2);
      p4 = Point2d(-hb, -width/2);
      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){
        (*itLidar).isSafe = false;
        numberZoneCritical++;
        //I associated the lidar Point to the critical zone, go to the next one
        continue;
      }

    }

    int zoneCount = 1;
    for (std::vector<RobotStatus::LocalizationData>::iterator itPose = futurePoses.begin() ; itPose != futurePoses.end(); ++itPose){

      RobotStatus::LocalizationData p = (*itPose);
      double ha = ha;
      double hb = hb;

      Point2d p1 = Point2d(p.x + ha*cos(p.theta) + safetyWidth*cos(p.theta+M_PI_2), p.y + ha*sin(p.theta) + safetyWidth*sin(p.theta+M_PI_2));
      Point2d p2 = Point2d(p.x + ha*cos(p.theta) - safetyWidth*cos(p.theta+M_PI_2), p.y + ha*sin(p.theta) - safetyWidth*sin(p.theta+M_PI_2));
      Point2d p3 = Point2d(p.x - hb*cos(p.theta) - safetyWidth*cos(p.theta+M_PI_2), p.y - hb*sin(p.theta) - safetyWidth*sin(p.theta+M_PI_2));
      Point2d p4 = Point2d(p.x - hb*cos(p.theta) + safetyWidth*cos(p.theta+M_PI_2), p.y - hb*sin(p.theta) + safetyWidth*sin(p.theta+M_PI_2));

      std::vector<Point2d> polygonVertex;
      polygonVertex.push_back(p1);
      polygonVertex.push_back(p2);
      polygonVertex.push_back(p3);
      polygonVertex.push_back(p4);
      polygonVertex.push_back(p1);

      Point2d lidarPoint;
      lidarPoint.x = ((*itLidar).x);
      lidarPoint.y = ((*itLidar).y);
      if(contain(lidarPoint,polygonVertex)){

        double tmp_distance = sqrt(pow((*itLidar).x,2)+pow((*itLidar).y,2)) - sqrt(pow(rearLidar.x_offset,2)+pow(rearLidar.y_offset,2));
        if(tmp_distance<closestDistance){
          closestDistance = tmp_distance;
        }

        (*itLidar).isSafe = false;
        if(zoneCount==1 || zoneCount==2){
          numberZoneRed++;
        }else if (zoneCount==3||zoneCount==4){
          numberZoneYellow++;
        }else if(zoneCount==5||zoneCount==6){
          numberZoneGreen++;
        }

        //As soon as the lidar point is associated to a region -> I continue the loop
        break;
      }
      zoneCount++;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////TAKE A DECISION////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  double brakingCoefficient = 1.0;
  if(numberZoneCritical!=0){
    brakingCoefficient = 0.0;
  }else if(numberZoneRed!=0){
    brakingCoefficient = fabs(closestDistance/distanceStar);
  }else if (numberZoneYellow!=0){
    brakingCoefficient = fabs(closestDistance/distanceStar);
  }else if (numberZoneGreen!=0){
    brakingCoefficient = 1.0;
  }else{
    brakingCoefficient = 1.0;
  }

  std::unique_lock<std::mutex> lock(dataMTX);
  lastInputVel = vel;
  lastInputOmega = omega;

  if(brakingCoefficient>this->brakingCoefficient){
    //Ho strada libera per cui voglio accelerare
    brakingCoefficient = this->brakingCoefficient+0.01;
  }
  this->brakingCoefficient = brakingCoefficient;
  lock.unlock();

  //std::cout << "BC: " << brakingCoefficient << std::endl;

  //Move the vehicle
  vehicleMove(vel*brakingCoefficient,omega*brakingCoefficient);

}

void HardwareGlobalInterface::setLocalization(double x, double y, double theta)
{
  ZMQCommon::RequesterSimple::status_t req_status;
  nlohmann::json j;
  j["cmd"] = std::string("initialize");
  j["x"] = x;
  j["y"] = y;
  j["theta"] = theta;
  /*j["cov_x"] = 10;
    j["cov_y"] = 10;
    j["cov_theta"] = 5;*/
  std::string response;
  //std::cout << j.dump() << std::endl;
  reqLoc->request(j.dump(),response,req_status);
  if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
    try{nlohmann::json j_resp = nlohmann::json::parse(response);
      //std::cout << j_resp.dump() << std::endl;
      if(j_resp.at("ack") == true){
        //Restart the subscriber
      }else{

      }
    }catch(std::exception &e){
      std::cerr<<e.what()<<std::endl;
    }
  }else if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_TIMEOUT){

  }
}

void HardwareGlobalInterface::setCurrent(double i_r, double i_l)
{
  ZMQCommon::RequesterSimple::status_t req_status;
  nlohmann::json j_req;
  j_req["cmd"] = std::string("set_rear_current");
  j_req["right"] = i_r;
  j_req["left"] = i_l;

  std::string response;
  reqHW->request(j_req.dump(),response,req_status);
  //std::cout << j_req.dump()<<std::endl;
  if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
    try{nlohmann::json j_resp = nlohmann::json::parse(response);

      if(j_resp.at("ack") == "true"){

      }else{

      }
    }catch(std::exception &e){
      std::cerr<<e.what()<<std::endl;
    }
  }else{

  }
}
