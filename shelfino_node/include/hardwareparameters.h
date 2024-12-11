#ifndef HARDWAREPARAMETERS_H
#define HARDWAREPARAMETERS_H

#include <string>

/**
 * @brief Structure that contains all the IP addresses and ports information used by ZMQ in the robot.
 * 
 */
struct HardwareParameters{
public:

  std::string hardwareServer = "tcp://10.196.80.112:5601";
  std::string hardwarePublisher = "tcp://10.196.80.112:6000";
  std::string trolleyPublisher = "tcp://10.196.80.112:6001";
  std::string localizationServer = "tcp://10.196.80.115:5600";
  std::string localizationPublisher = "tcp://10.196.80.115:5563";
  std::string frontLidarPublisher = "tcp://10.196.80.115:7500";
  std::string rearLidarPublisher = "tcp://10.196.80.115:7501";
  std::string realSensePublisher = "tcp://10.196.80.115:7502";
  std::string kartoPosePublisher = "tcp://192.168.3.99:7001";
  std::string realSenseOdom = "tcp://10.196.80.115:9304";
  std::string realSensePosePublisher = "tcp://10.196.80.115:9112";
  std::string realSenseTagsPublisher = "tcp://10.196.80.115:9114";
  std::string trackingPublisher = "tcp://10.196.80.115:7901";
  std::string autnavServer = "tcp://10.196.80.115:1991";

  int id = 0;

  double vehicleWidth = 0.5;
  double vehicleLength = 0.5;
  double cameraOffsetX = 0.25;
  double cameraOffsetY = 0.0;

  /**
   * @brief Construct a new Hardware Parameters structure using an integer value for the initialization of the IP addresses.
   * 
   * @param i integer value used to set the NUC and BeagleBone addresses.
   * ~~~~~~~~~~{.cpp}
   * BeagleBone IP = 100 + i*10 + 2;
   * NUC IP = 100 + i*10 + 5;
   * ~~~~~~~~~~
   */
  HardwareParameters(int i = 0){
    int bbIP = 100 + i*10 + 2;
    int nucIP = 100 + i*10 + 5;
    id = i;
    hardwareServer = "tcp://10.196.80." + std::to_string(bbIP) + ":5601";
    hardwarePublisher = "tcp://10.196.80." + std::to_string(bbIP) + ":6000";
    trolleyPublisher = "tcp://10.196.80." + std::to_string(bbIP) + ":6001";
    localizationServer = "tcp://10.196.80." + std::to_string(nucIP) + ":5600";
    localizationPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":5563"; // 5563
    frontLidarPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7500";
    rearLidarPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7501";
    realSensePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7502";
    kartoPosePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7001";
    realSenseOdom = "tcp://10.196.80." + std::to_string(nucIP) + ":9304";
    realSensePosePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9112";
    realSenseTagsPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9114";
    trackingPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7901";
    autnavServer = "tcp://10.196.80." + std::to_string(nucIP) + ":1991";
  }
};

#endif // HARDWAREPARAMETERS_H
