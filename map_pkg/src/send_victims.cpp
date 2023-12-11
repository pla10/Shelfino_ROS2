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
    this->declare_parameter("min_weight", 10);
    this->declare_parameter("max_weight", 500);

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
    uint min_weight = this->get_parameter("min_weight").as_int();
    uint max_weight = this->get_parameter("max_weight").as_int();

    // Print parameters values
    RCLCPP_INFO(this->get_logger(), "map: %s", map.c_str());
    RCLCPP_INFO(this->get_logger(), "dx: %f", dx);
    RCLCPP_INFO(this->get_logger(), "dy: %f", dy);
    RCLCPP_INFO(this->get_logger(), "max_timeout: %f", max_timeout);
    RCLCPP_INFO(this->get_logger(), "n_victims: %d", n_victims);
    RCLCPP_INFO(this->get_logger(), "min_weight: %d", min_weight);
    RCLCPP_INFO(this->get_logger(), "max_weight: %d", max_weight);

    if (n_victims == 0) {
      RCLCPP_INFO(this->get_logger(), "No victims to generate.");
      exit(0);
    }

    publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/victims", qos);

    // Define a random number generator for doubles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dis(-dx, dx);
    std::uniform_real_distribution<> y_dis(-dy, dy);
    std::uniform_int_distribution<> weight_dis(min_weight, max_weight);

    std::vector<obstacle> victims;
    auto startTime = this->get_clock()->now();
    for (uint i=0; i<n_victims && !overTime(this->get_clock(), startTime, max_timeout); i++) {
      victim vict = victim(0.0, 0.0);
      do {
        vict.x = x_dis(gen);
        vict.y = y_dis(gen);
        if (!overlaps(vict, victims) && is_inside_map(vict, map, dx, dy)) {
          victims.push_back(vict);
          break;
        }
      } while(!overTime(this->get_clock(), startTime, max_timeout));
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
      // While physically the radius of the victim won't be touched, the now 
      // assigned value indicates the weight of the victim
      obs.radius = weight_dis(gen);

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
    
      spawn_model(this->get_node_base_interface(), this->spawner_, xml_string, pose);
      sleep(0.5);
    }

    this->publisher_->publish(msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VictimPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}