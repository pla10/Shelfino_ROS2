#ifndef HARDWAREGLOBALINTERFACE_H
#define HARDWAREGLOBALINTERFACE_H

#define LOC_REALSENSE

#include <single_thread.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "hardwareparameters.h"
#include "robotstatus.h"
#include "personitem.h"

#include <Subscriber.hpp>
#include <RequesterSimple.hpp>

#define LOC_TIMEOUT 50
#define HW_TIMEOUT 50
#define LIDAR_TIMEOUT 200
#define TRACKING_TIMEOUT 200

static const double LEFT_INCREMENTS_PER_TOUR = 230000.; //230000.
static const double RIGHT_INCREMENTS_PER_TOUR = 230000.; //294912.
static const double LEFT_RADIUS = 0.99 * 0.125 ; //0.9907*0.125; //0.125186;
static const double RIGHT_RADIUS = 0.99 * 0.125 ; //0.99*0.125; //0.125138;
static const double WHEEL_BASE = 0.383;
static const double LEFT_B_D = 4* WHEEL_BASE / (2.*LEFT_RADIUS); 
static const double RIGHT_B_D = 4* WHEEL_BASE / (2.*RIGHT_RADIUS); 
static const double ENCODER_PPR = 36*4*2048;                           //* Encoder pulses per revolution
static const double RATIO_LEFT_RIGHT = RIGHT_B_D/LEFT_B_D;

/**
 * @brief Interface that handles all the communication with ZMQ for the hardware of the robot.
 * 
 */
class HardwareGlobalInterface
{
public:

  /**
   * @brief Get the Instance object
   * 
   * @return HardwareGlobalInterface& 
   */
  static HardwareGlobalInterface & getInstance() {

    if(instances.size()<1) throw std::runtime_error("Robot not initialized");
    return *instances.at(0);
  }

  /**
   * @brief Get a specific Instance object
   * 
   * @param i the index of the wanted initialized robot
   * @return HardwareGlobalInterface& 
   */
  static HardwareGlobalInterface & getInstance(unsigned long i) {

    if(instances.size()<i) throw std::runtime_error("Robot not initialized");
    return *instances.at(i);
  }

  /**
   * @brief Initialize a robot istance defining its parameters.
   * 
   * @param hp object containing the list of the robot IP addresses and ports used by ZMQ in the robot.
   * @return HardwareGlobalInterface& 
   */
  static HardwareGlobalInterface &initialize(HardwareParameters* hp){


    instances.push_back(new HardwareGlobalInterface(hp));

    instances.back()->params = hp;
    instances.back()->id = hp->id;
    instances.back()->encoder_ppr = ENCODER_PPR; // (Shelfino1) *1,376  | (Shelfino2) *1  | (Shelfino3) *0.6
    if(hp->id == 1) instances.back()->encoder_ppr = ENCODER_PPR*1,376 ;
    if(hp->id == 3) instances.back()->encoder_ppr = ENCODER_PPR*0.6;

    instances.back()->frontLidar.setMountingPosition(0,0);
    instances.back()->rearLidar.setMountingPosition(-0.7,0);
    instances.back()->realSenseLidar.setMountingPosition(hp->cameraOffsetX,hp->cameraOffsetY);

    instances.back()->subLoc.reset(new ZMQCommon::Subscriber());
    instances.back()->subLoc->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->sub_locSubscriber(topic,buf,size,data);});
    instances.back()->subLoc->start(hp->localizationPublisher,"LOC");

    instances.back()->subLocKarto.reset(new ZMQCommon::Subscriber());
    instances.back()->subLocKarto->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->sub_locSubscriberKarto(topic,buf,size,data);});
    instances.back()->subLocKarto->start(hp->kartoPosePublisher,"POS");

    instances.back()->subLocRealSense.reset(new ZMQCommon::Subscriber());
    instances.back()->subLocRealSense->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->sub_locSubscriberRealSense(topic,buf,size,data);});
    instances.back()->subLocRealSense->start(hp->realSensePosePublisher,"POS");

    instances.back()->subHW.reset(new ZMQCommon::Subscriber());
    instances.back()->subHW->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->subHW_callback(topic,buf,size,data);});
    instances.back()->subHW->start(hp->hardwarePublisher,"PUB_HW");

    instances.back()->subHWTrolley.reset(new ZMQCommon::Subscriber());
    instances.back()->subHWTrolley->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->subHW_trolley(topic,buf,size,data);});
    instances.back()->subHWTrolley->start(hp->trolleyPublisher,"PUB_HW");

    instances.back()->subFrontLidar.reset(new ZMQCommon::Subscriber());
    instances.back()->subFrontLidar->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->sub_frontLidar(topic,buf,size,data);});
    instances.back()->subFrontLidar->start(hp->frontLidarPublisher,"LIDAR");

    instances.back()->subRealSense.reset(new ZMQCommon::Subscriber());
    instances.back()->subRealSense->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->sub_realSenseLidar(topic,buf,size,data);});
    instances.back()->subRealSense->start(hp->realSensePublisher,"LIDAR");

    instances.back()->subRearLidar.reset(new ZMQCommon::Subscriber());
    instances.back()->subRearLidar->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->sub_rearLidar(topic,buf,size,data);});
    instances.back()->subRearLidar->start(hp->rearLidarPublisher,"LIDAR");

    instances.back()->subTracking.reset(new ZMQCommon::Subscriber());
    instances.back()->subTracking->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->sub_tracking(topic,buf,size,data);});
    instances.back()->subTracking->start(hp->trackingPublisher,"PUB_TRACKER");

    instances.back()->subRealSenseOdom.reset(new ZMQCommon::Subscriber());
    instances.back()->subRealSenseOdom->register_callback([&](const char *topic, const char *buf, size_t size, void *data){instances.back()->sub_realSenseOdom(topic,buf,size,data);});
    instances.back()->subRealSenseOdom->start(hp->realSenseOdom,"ODOM");

    std::cout << "Setting HW server with " << hp->hardwareServer << std::endl;

    instances.back()->reqHW.reset(new ZMQCommon::RequesterSimple(hp->hardwareServer));
    instances.back()->reqLoc.reset(new ZMQCommon::RequesterSimple(hp->localizationServer));

    return *instances.back();
  }

  /**
   * @brief Get the Localization Data object
   * 
   * @param locData the localization data object.
   * @return true the localization data is available.
   * @return false the localization data has expired.
   */
  bool getLocalizationData(RobotStatus::LocalizationData &locData){
    std::unique_lock<std::mutex> lock(locDataMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    locData = this->locData;
    if(currTime - this->locData.locTimer < LOC_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  /**
   * @brief Get the Params object containing the list of IP addresses and ports used by ZMQ in the robot.
   * 
   * @return HardwareParameters object containing the list of IP addresses and ports used by ZMQ in the robot.
   */
  HardwareParameters getParams() {
    HardwareParameters res;
    if (params)
      res = *params;
    return res;
  }

  bool getLocalizationDataKarto(RobotStatus::LocalizationData &locDataKarto){
    std::unique_lock<std::mutex> lock(locDataKartoMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    locDataKarto = this->locDataKarto;
    if(currTime - this->locDataKarto.locTimer < LOC_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }


  bool getLocalizationDataRealSense(RobotStatus::LocalizationData &locDataRealSense){
    std::unique_lock<std::mutex> lock(locDataRealSenseMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    locDataRealSense = this->locDataRealSense;
    if(currTime - this->locDataRealSense.locTimer < LOC_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  bool getHardwareData(RobotStatus::HardwareData &hwData){
    std::unique_lock<std::mutex> lock(hwDataMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    hwData = this->hwData;
    if(currTime - this->hwData.hardwareTimer < HW_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  bool getTrolleyData(RobotStatus::TrolleyData &trolleyData){
    std::unique_lock<std::mutex> lock(trolleyDataMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    trolleyData = this->trolleyData;
    if(currTime - this->trolleyData.trolleyTimer < HW_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  //    bool subTracker_callback(RobotStatus::TrackerData &trackerData){
  //        std::unique_lock<std::mutex> lock(trackerDataMTX);
  //        double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  //        trackerData = this->trackerData;
  //        if(currTime - this->trackerData.trackerTimer < TRACKER_TIMEOUT){
  //            return true;
  //        }else{
  //            return false;
  //        }
  //    }// subTracker_callback(const char *topic, const char *buf, size_t size, void *data)

  /**
   * @brief Get the Front Lidar Data object.
   * 
   * @param lidarData 
   * @return true the lidar data is available.
   * @return false the lidar data has expired.
   */
  bool getFrontLidarData(RobotStatus::LidarData &lidarData){
    std::unique_lock<std::mutex> lock(frontLidarMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    lidarData = this->frontLidar;
    if(currTime - this->frontLidar.lidarTimer < LIDAR_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  bool getRearLidarData(RobotStatus::LidarData &lidarData){
    std::unique_lock<std::mutex> lock(rearLidarMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    lidarData = this->rearLidar;
    if(currTime - this->rearLidar.lidarTimer < LIDAR_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  bool getRealSenseLidarData(RobotStatus::LidarData &lidarData){
    std::unique_lock<std::mutex> lock(realSenseLidarMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    lidarData = this->realSenseLidar;
    if(currTime - this->realSenseLidar.lidarTimer < LIDAR_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  bool getTrackingData(std::vector<PersonItem> &trackingData){
    std::unique_lock<std::mutex> lock(trackingMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    trackingData = this->trackingData;
    if(currTime - this->trackingTimer < TRACKING_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  /**
   * @brief Get the Real Sense Odometry Data object
   * 
   * @param realSenseOdomData 
   * @return true the real sense odometry data is available.
   * @return false the real sense odometry data has expired.
   */
  bool getRealSenseOdomData(RobotStatus::OdometryData &realSenseOdomData){
    std::unique_lock<std::mutex> lock(realSenseOdomMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    realSenseOdomData = this->realSenseOdom;
    if(currTime - this->realSenseOdom.last_odom_update < LIDAR_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }

  bool getOdomData(RobotStatus::OdometryData &odomData){
    std::unique_lock<std::mutex> lock(odomMTX);
    double currTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    odomData = this->odomData;
    if(currTime - this->odomData.last_odom_update < LIDAR_TIMEOUT){
      return true;
    }else{
      return false;
    }
  }


  ~HardwareGlobalInterface();

  void robotOnVelControl();
  void robotOnCurrControl();
  void robotOff();
  void vehicleMove(float vel, float omega);
  //Safely move the vehicle since it ckecks for possible collisions
  void vehicleSafeMove(float vel, float omega);
  void setLocalization(double x, double y, double theta);
  void setCurrent(double i_r, double i_l);
  void calibrateTrolley();
  void getLastInputs(double &v, double &omega);
  void getBrakingCoefficient(double &bc);

private:

  static std::vector<HardwareGlobalInterface *> instances;

  HardwareGlobalInterface();
  HardwareGlobalInterface(HardwareParameters* hp);

  HardwareGlobalInterface(const HardwareGlobalInterface &rs) {
    instances = rs.instances;
  }

  HardwareGlobalInterface &operator=(const HardwareGlobalInterface &rs) {
    if (this != &rs) {
      instances = rs.instances;
    }

    return *this;
  }

  std::mutex locDataMTX;
  RobotStatus::LocalizationData locData;
  std::unique_ptr<ZMQCommon::Subscriber> subLoc;

  std::mutex locDataKartoMTX;
  RobotStatus::LocalizationData locDataKarto;
  std::unique_ptr<ZMQCommon::Subscriber> subLocKarto;

  std::mutex locDataRealSenseMTX;
  RobotStatus::LocalizationData locDataRealSense;
  std::unique_ptr<ZMQCommon::Subscriber> subLocRealSense;

  std::mutex hwDataMTX;
  RobotStatus::HardwareData hwData;
  std::unique_ptr<ZMQCommon::Subscriber> subHW;

  std::mutex trolleyDataMTX;
  RobotStatus::TrolleyData trolleyData;
  std::unique_ptr<ZMQCommon::Subscriber> subHWTrolley;

  //    std::mutex trackerDataMTX;
  //    RobotStatus::TrackerData trackerData;
  //    std::unique_ptr<ZMQCommon::Subscriber> subTracker;

  std::mutex frontLidarMTX;
  RobotStatus::LidarData frontLidar;
  std::unique_ptr<ZMQCommon::Subscriber> subFrontLidar;

  std::mutex rearLidarMTX;
  RobotStatus::LidarData rearLidar;
  std::unique_ptr<ZMQCommon::Subscriber> subRearLidar;

  std::mutex realSenseLidarMTX;
  RobotStatus::LidarData realSenseLidar;
  std::unique_ptr<ZMQCommon::Subscriber> subRealSense;

  std::mutex trackingMTX;
  std::vector<PersonItem> trackingData;
  double trackingTimer;
  std::unique_ptr<ZMQCommon::Subscriber> subTracking;

  std::mutex realSenseOdomMTX;
  RobotStatus::OdometryData realSenseOdom;
  std::unique_ptr<ZMQCommon::Subscriber> subRealSenseOdom;

  std::mutex odomMTX;
  RobotStatus::OdometryData odomData;
  RobotStatus::WheelSpeed wsLeft, wsRight;

  std::unique_ptr<ZMQCommon::RequesterSimple> reqHW;
  std::unique_ptr<ZMQCommon::RequesterSimple> reqLoc;

  std::mutex dataMTX;

  void sub_locSubscriber(const char *topic, const char *buf, size_t size, void *data);
  void subHW_callback(const char *topic, const char *buf, size_t size, void *data);
  void subHW_trolley(const char *topic, const char *buf, size_t size, void *data);
  void sub_frontLidar(const char *topic, const char *buf, size_t size, void *data);
  void sub_realSenseLidar(const char *topic, const char *buf, size_t size, void *data);
  void sub_rearLidar(const char *topic, const char *buf, size_t size, void *data);
  void sub_locSubscriberKarto(const char *topic, const char *buf, size_t size, void *data);
  void sub_locSubscriberRealSense(const char *topic, const char *buf, size_t size, void *data);
  void sub_tracking(const char *topic, const char *buf, size_t size, void *data);
  void sub_realSenseOdom(const char *topic, const char *buf, size_t size, void *data);
  void updateOdometryV(double current_time, double vl, double vr);
  void updateOdometryOmega(double current_time, std::vector<double> const & orientation, std::vector<double> const & angular_vel);
  std::vector<double> ToEulerAngles(std::vector<double> const & q);


  HardwareParameters* params = nullptr;
  double safetyWidth = 0.3;
  double criticalDistance = 0.2;

  double ha = 0.25; //0.1; //distanza x della punta del veicolo rispetto al suo sistema di riferimento
  double hb = 0.25; //0.7; //distanza x della coda del veicolo rispetto al suo sistema di riferimento
  double width = 0.5; //larghezza veicolo
  double length = 0.5;
  int id = 0;

  double encoder_ppr = ENCODER_PPR;

  double lastInputVel = 0;
  double lastInputOmega = 0;
  double brakingCoefficient = 1.0;

  void setDeviceMode(int deviceMode);
  bool powerEnable(bool val);
};

#endif // HARDWAREGLOBALINTERFACE_H
