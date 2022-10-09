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

class HardwareGlobalInterface
{
public:

  static HardwareGlobalInterface & getInstance() {

    if(instances.size()<1) throw std::runtime_error("Robot not initialized");
    return *instances.at(0);
  }

  static HardwareGlobalInterface & getInstance(unsigned long i) {

    if(instances.size()<i) throw std::runtime_error("Robot not initialized");
    return *instances.at(i);
  }

  static HardwareGlobalInterface & initialize() {

    instances.push_back(new HardwareGlobalInterface(new HardwareParameters()));
    return *instances.back();
  }


  static HardwareGlobalInterface &initialize(HardwareParameters* hp){


    instances.push_back(new HardwareGlobalInterface(hp));

    instances.back()->params = hp;

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


    instances.back()->reqHW.reset(new ZMQCommon::RequesterSimple(hp->hardwareServer));
    instances.back()->reqLoc.reset(new ZMQCommon::RequesterSimple(hp->localizationServer));

    return *instances.back();
  }

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

  HardwareParameters* params = nullptr;
  double safetyWidth = 0.3;
  double criticalDistance = 0.2;

  double ha = 0.25; //0.1; //distanza x della punta del veicolo rispetto al suo sistema di riferimento
  double hb = 0.25; //0.7; //distanza x della coda del veicolo rispetto al suo sistema di riferimento
  double width = 0.5; //larghezza veicolo
  double length = 0.5;

  double lastInputVel = 0;
  double lastInputOmega = 0;
  double brakingCoefficient = 1.0;

  void setDeviceMode(int deviceMode);
  void powerEnable(bool val);
};

#endif // HARDWAREGLOBALINTERFACE_H
