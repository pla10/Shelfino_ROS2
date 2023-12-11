#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "map_pkg/spawn_model.hpp"
#include "map_pkg/utilities.hpp"

// TODO It would be nice to wait for the service response.
// TODO The gate should be spawned along the borders of the map

class GatesPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  std::string node_namespace;
  std::string share_dir;

public:
  GatesPublisher() : Node("send_gates")
  {
    // Get share directory with ament 
    this->share_dir = ament_index_cpp::get_package_share_directory("map_pkg");
    this->node_namespace = this->get_namespace();

    // Map parameters
    this->declare_parameter("map", "hexagon");
    this->declare_parameter("dx", 10.0);
    this->declare_parameter("dy", 10.0);

    // Gate parameters
    this->declare_parameter("use_namespace", false);
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    
    this->publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/gate_position", qos);
    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // Get parameters
    std::string map = this->get_parameter("map").as_string();
    if (map != "hexagon" && map != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
      exit(1);
    }

    double x = this->get_parameter("x").as_double();
    double y = this->get_parameter("y").as_double();
    double dx = this->get_parameter("dx").as_double();
    double dy = this->get_parameter("dy").as_double();

    // If the position was passed, then use it
    if (x != 0.0 && y != 0.0) {
      this->spawn_gates(x, y);
    }
    else {
      if (map == "hexagon"){
        this->rand_hexagon_gate(dx, dy);
      }
      else if (map == "rectangle"){
        this->rand_rectangle_gate(dx, dy);
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
        exit(1);
      }
    }
  }

private:
  void spawn_gates(double x, double y);
  void rand_hexagon_gate(double dx, double dy);
  void rand_rectangle_gate(double dx, double dy);
};

void GatesPublisher::rand_hexagon_gate(double dx, double dy){
  // Declare random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(-dx, dx);
  std::uniform_int_distribution<> dis_left_right(0,1);
  obstacle obs {0.0, 0.0, 0.0, 1.0, 1.0, obstacle_type::BOX};

  obs.y = sqrt(3)/2.0*dx - 0.5-0.0001;
  obs.y = obs.y * (dis_left_right(gen) == 0 ? 1 : -1);
  do {
    obs.x = dis_x(gen);
  } while(!is_inside_map(obs, "hexagon", dx, dy));
  
  this->spawn_gates(obs.x, obs.y);
}

void GatesPublisher::rand_rectangle_gate(double dx, double dy){
  // Declare random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(-dx, dx);
  std::uniform_real_distribution<> dis_y(-dy, dy);
  std::uniform_int_distribution<> dis_edge(0, 3);

  obstacle obs {0.0, 0.0, 0., 1.0, 1.0, obstacle_type::BOX};

  do {
    int edge = dis_edge(gen);
    switch (edge){
      //Top horizontal edge 
      case 0:
        obs.x = dis_x(gen);
        obs.y = dy/2.0 - 0.5-0.0001;
        break;
      //Bottom horizontal edge
      case 1:
        obs.x = dis_x(gen);
        obs.y = -(dy/2.0 - 0.5-0.0001);
        break;
      //Right vertical edge
      case 2:
        obs.x = dx/2.0 - 0.5-0.0001;
        obs.y = dis_y(gen);
        break;
      //Left vertical edge
      case 3:
        obs.x = -(dx/2.0 - 0.5-0.0001);
        obs.y = dis_y(gen);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Random number generator failed, extracted %d, but should be in [0,4)", edge);
        exit(1);
    }
  } while(!is_inside_map(obs, "rectangle", dx, dy));
  
  this->spawn_gates(obs.x, obs.y);
}

/**
 * @brief Publish the position of the gate in the hexagon map
 * 
 */
void GatesPublisher::spawn_gates(double x, double y){
  std_msgs::msg::Header hh;
  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Pose> pose_array_temp;
  geometry_msgs::msg::PoseArray msg;

  // Set headers of messages
  hh.stamp = this->get_clock()->now();
  hh.frame_id = "/map";
  msg.header = hh;

  // Set position of the gate
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.05;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  pose_array_temp.push_back(pose);
  
  // Add gate to the message
  msg.poses = pose_array_temp;

  // Publish message
  publisher_->publish(msg);

  // Spawn gate in gazebo
  std::string xml = std::string((
    std::istreambuf_iterator<char>(std::ifstream(this->share_dir + "/models/gate/model.sdf").rdbuf())), std::istreambuf_iterator<char>());
  spawn_model(this->get_node_base_interface(), this->spawner_, xml, pose);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create the node 
  auto node = std::make_shared<GatesPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

