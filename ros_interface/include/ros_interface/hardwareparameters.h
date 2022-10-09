#ifndef HARDWAREPARAMETERS_H
#define HARDWAREPARAMETERS_H

#include <string>

struct HardwareParameters{
public:

  std::string hardwareServer = "tcp://192.168.3.2:5601";
  std::string hardwarePublisher = "tcp://192.168.3.2:6000";
  std::string trolleyPublisher = "tcp://192.168.3.2:6001";
  std::string localizationServer = "tcp://192.168.3.5:5600";
  std::string localizationPublisher = "tcp://192.168.3.5:5563";
  std::string frontLidarPublisher = "tcp://192.168.3.5:7500";
  std::string rearLidarPublisher = "tcp://192.168.3.5:7501";
  std::string realSensePublisher = "tcp://192.168.3.5:7502";
  std::string kartoPosePublisher = "tcp://192.168.3.99:7001";
  std::string realSensePosePublisher = "tcp://192.168.3.5:9112";
  std::string realSenseTagsPublisher = "tcp://192.168.3.5:9114";
  std::string trackingPublisher = "tcp://192.168.3.5:7901";
  std::string autnavServer = "tcp://192.168.3.5:1991";

  double vehicleWidth = 0.5;
  double vehicleLength = 0.5;
  double cameraOffsetX = 0.25;
  double cameraOffsetY = 0.0;

  HardwareParameters(){
    hardwareServer = "tcp://192.168.3.2:5601";
    hardwarePublisher = "tcp://192.168.3.2:6000";
    trolleyPublisher = "tcp://192.168.3.2:6001";
    localizationServer = "tcp://192.168.3.5:5600";
    localizationPublisher = "tcp://192.168.3.5:5563";
    frontLidarPublisher = "tcp://192.168.3.5:7500";
    rearLidarPublisher = "tcp://192.168.3.5:7501";
    realSensePublisher = "tcp://192.168.3.5:7502";
    kartoPosePublisher = "tcp://192.168.3.99:7001";
    realSensePosePublisher = "tcp://192.168.3.5:9112";
    realSenseTagsPublisher = "tcp://192.168.3.5:9114";
    trackingPublisher = "tcp://192.168.3.5:7901";
    autnavServer = "tcp://192.168.3.5:1991";
  }

  HardwareParameters(std::string val){
    if(val.compare("localhost")==0){
      hardwareServer = "tcp://localhost:5601";
      hardwarePublisher = "tcp://localhost:6000";
      trolleyPublisher = "tcp://localhost:6001";
      localizationServer = "tcp://localhost:5600";
      localizationPublisher = "tcp://localhost:5563"; // 5563
      frontLidarPublisher = "tcp://localhost:7500";
      rearLidarPublisher = "tcp://localhost:7501";
      realSensePublisher = "tcp://localhost:7502";
      kartoPosePublisher = "tcp://localhost:7001";
      realSensePosePublisher = "tcp://localhost:9112";
      realSenseTagsPublisher = "tcp://localhost:9114";
      trackingPublisher = "tcp://localhost:7901";
      autnavServer = "tcp://localhost:1991";
    }
  }

  HardwareParameters(int i){
    int bbIP = 100 + i*10 + 2;
    int nucIP = 100 + i*10 + 5;
    hardwareServer = "tcp://10.196.80." + std::to_string(bbIP) + ":5601";
    hardwarePublisher = "tcp://10.196.80." + std::to_string(bbIP) + ":6000";
    trolleyPublisher = "tcp://10.196.80." + std::to_string(bbIP) + ":6001";
    localizationServer = "tcp://10.196.80." + std::to_string(nucIP) + ":5600";
    localizationPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":5563"; // 5563
    frontLidarPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7500";
    rearLidarPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7501";
    realSensePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7502";
    kartoPosePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7001";
    realSensePosePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9112";
    realSenseTagsPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9114";
    trackingPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7901";
    autnavServer = "tcp://10.196.80." + std::to_string(nucIP) + ":1991";
  }
};

#endif // HARDWAREPARAMETERS_H
