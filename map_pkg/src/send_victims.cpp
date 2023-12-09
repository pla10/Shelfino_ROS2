#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <random>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"

#include "gazebo_msgs/srv/spawn_entity.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "map_pkg/utilities.hpp"
#include "map_pkg/spawn_model.hpp"

class VictimPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  std::string node_namespace;
  std::string share_dir;

public:
  VictimPublisher() : Node("obstacles_sender")
  {
    this->share_dir = ament_index_cpp::get_package_share_directory("map_pkg");
    this->node_namespace = this->get_namespace();

    // Map parameters
    this->declare_parameter("map", "hexagon");
    this->declare_parameter("dx", 10.0);
    this->declare_parameter("dy", 10.0);
    this->declare_parameter("max_timeout", 3);

    // Obstacle parameters
    this->declare_parameter("n_victims", 3);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // Get parameters
    std::string map = this->get_parameter("map").as_string();
    if (map != "hexagon" && map != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
      exit(1);
    }
    double dx = this->get_parameter("dx").as_double();
    double dy = this->get_parameter("dy").as_double();
    double max_timeout = this->get_parameter("max_timeout").as_int();
    uint n_victims = this->get_parameter("n_victims").as_int();

    // Print parameters values
    RCLCPP_INFO(this->get_logger(), "map: %s", map.c_str());
    RCLCPP_INFO(this->get_logger(), "dx: %f", dx);
    RCLCPP_INFO(this->get_logger(), "dy: %f", dy);
    RCLCPP_INFO(this->get_logger(), "max_timeout: %f", max_timeout);
    RCLCPP_INFO(this->get_logger(), "n_victims: %d", n_victims);

    publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/victims", qos);

    // Define a random number generator for doubles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dis(-dx, dx);
    std::uniform_real_distribution<> y_dis(-dy, dy);

    std::vector<obstacle> victims;
    auto startTime = this->get_clock()->now();
    for (uint i=0; i<n_victims && !overTime(this->get_clock(), startTime, max_timeout); i++) {
      victim vict = victim(0.0, 0.0);
      do {
        vict.x = x_dis(gen);
        vict.y = y_dis(gen);
      } while(overlaps(vict, victims) && !is_inside_map(vict, map, dx, dy)
              && !overTime(this->get_clock(), startTime, max_timeout));
      victims.push_back(vict);
    }

    if (overTime(this->get_clock(), startTime, max_timeout)) {
      RCLCPP_ERROR(this->get_logger(), "Timeout while trying to generate victims.");
      exit(1);
    }

    obstacles_msgs::msg::ObstacleArrayMsg msg;
    std_msgs::msg::Header hh;
    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    for (auto vict : victims) {
      RCLCPP_INFO(this->get_logger(), "Publishing victim at x=%f, y=%f", vict.x, vict.y);

      obstacles_msgs::msg::ObstacleMsg obs;
      geometry_msgs::msg::Polygon pol;
    
      geometry_msgs::msg::Point32 point;
      point.x = vict.x;
      point.y = vict.y;
      point.z = 0.0;
      pol.points.push_back(point);
      obs.polygon = pol;
      obs.radius = vict.radius;

      msg.obstacles.push_back(obs);

      // Read XML file to string
      std::string xml_string;
      std::ifstream xml_file(this->share_dir + "/models/victim/model.sdf");
      xml_string.assign(
        std::istreambuf_iterator<char>(xml_file),
        std::istreambuf_iterator<char>()
      );

      // Spawn model in gazebo
      geometry_msgs::msg::Pose pose;
      pose.position.x = vict.x;
      pose.position.y = vict.y;
      pose.position.z = 0.01;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 0;
    
      spawn_model(xml_string, pose, this->spawner_, this->node_namespace);
      sleep(0.5);
    }

    this->publisher_->publish(msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VictimPublisher>());
  rclcpp::shutdown();
  return 0;
}


// /**
//  * @brief Spawn model of the gate in gazebo
//  * 
//  * @param pose The position in which the gate should be spawned
//  */
// void VictimPublisher::spawn_victims(std::string xml, geometry_msgs::msg::Pose pose){
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service spawn_entity...");
//   while(!this->spawner_->wait_for_service(std::chrono::milliseconds(100))){
//     if (!rclcpp::ok()){
//       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//       rclcpp::shutdown(nullptr, "Interrupted while waiting for the service. Exiting.");
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//   }

//   // Configure request
//   auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
//   request->name = "victim"+std::to_string(pose.position.x)+"_"+std::to_string(pose.position.y);
//   request->initial_pose = pose;
//   request->robot_namespace = this->node_namespace;
//   request->xml = xml;

//   // Send request
//   auto result = this->spawner_->async_send_request(request);
// }




// // Function that checks that two victims do not overlap
// bool overlaps(victim o1, victim o2) {
//   double dist = sqrt(pow(o1.x - o2.x, 2) + pow(o1.y - o2.y, 2));
//   if (dist < o1.radius + o2.radius) {
//     return true;
//   }
//   return false;
// }

// bool overlaps(victim o1, std::vector<victim> victims){
//   for (auto o2 : victims){
//     if (overlaps(o1, o2)){
//       return true;
//     }
//   }
//   return false;
// }

// bool is_inside_map(victim obs, std::string map, uint dx, uint dy){
//   if (map == "square"){
//     // Check if the map is a square and consider it as a rectangle of equal edges
//     if (dy == 0) {
//       dy = dx;
//     }
//     if (abs(obs.x) < dx/2.0 - obs.radius  && abs(obs.y) < dy/2.0 - obs.radius) {
//       return true;
//     }
//   }
//   else if (map == "hexagon"){
//     // Check if circle is inside of hexagon 
//     // Inside the combination of rectangles
//     if (abs(obs.x) < dx/2.0 + sqrt(2.0)/2.0*dx - obs.radius && abs(obs.y) < dy/2.0 + sqrt(2.0)/2.0*dy - obs.radius){
//       if (abs(obs.x) < dx/2.0 - obs.radius && abs(obs.y) < dy/2.0 + sqrt(2.0)/2.0*dy - obs.radius && 
//           abs(obs.x) < dx/2.0 + sqrt(2.0)/2.0*dx - obs.radius && abs(obs.y) < dy/2.0 - obs.radius) {
//         return true;
//       }
//       else {
//         return false;
//       }
//     }
//   }
//   else{
//     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Map %s not recognized.", map.c_str());
//     return false;
//   }
//   return false;
// }
