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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "map_pkg/spawn_model.hpp"
#include "map_pkg/utilities.hpp"

//TODO it would be nice if we could send the initial position, but it either this node is launched too early, or it does not work

class InitPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;    
  std::vector<std::string> names;

public:
  InitPublisher() : Node("send_initialposes")
  {
    RCLCPP_INFO(this->get_logger(), "Starting sending initial poses");  

    // Map parameters
    this->declare_parameter("map", "hexagon");
    this->declare_parameter("dx", 10.0);
    this->declare_parameter("dy", 10.0);

    // initial parameters
    this->declare_parameter("vect_x",   std::vector<double>({0.0}));
    this->declare_parameter("vect_y",   std::vector<double>({0.0}));
    this->declare_parameter("vect_yaw", std::vector<double>({0.0}));
    this->declare_parameter("names", std::vector<std::string>({"shelfino1"}));
    this->declare_parameter("random",   true);

    RCLCPP_INFO(this->get_logger(), "Starting sending initial poses");  

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    this->publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/initial_poses", qos);

    // Get parameters
    std::string map = this->get_parameter("map").as_string();
    if (map != "hexagon" && map != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
      exit(1);
    }

    this->names  = this->get_parameter("names").as_string_array();
    int n_robots = names.size();
    double dx    = this->get_parameter("dx").as_double();
    double dy    = this->get_parameter("dy").as_double();
    bool random  = this->get_parameter("random").as_bool();

    RCLCPP_INFO(this->get_logger(), "shelfino%s: map: %s, dx: %f, dy: %f, n_robots: %d, random: %d", this->get_namespace(), map.c_str(), dx, dy, n_robots, random);

    geometry_msgs::msg::PoseArray pose_array_msg;
    if (random) {
      // Otherwise, generate n_robot random positions
      if (map == "hexagon"){
        for (int i = 0; i < n_robots; i++){
          pose_array_msg.poses.push_back(this->rand_hexagon(dx, dy));        
        }
      }
      else if (map == "rectangle"){
        for (int i = 0; i < n_robots; i++){
          pose_array_msg.poses.push_back(this->rand_hexagon(dx, dy));        
        }
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
        exit(1);
      }
    }
    else {
      std::vector<double> vect_x = this->get_parameter("vect_x").as_double_array();
      std::vector<double> vect_y = this->get_parameter("vect_y").as_double_array();
      std::vector<double> vect_yaw = this->get_parameter("vect_yaw").as_double_array();

      if (vect_x.size() != vect_y.size() || vect_x.size() != vect_yaw.size() || vect_y.size() != vect_yaw.size()){
        RCLCPP_ERROR(this->get_logger(), "vect_x, vect_y and vect_yaw must have the same size.");
        exit(1);
      }

      for (uint i=0; i<vect_x.size(); i++){
        geometry_msgs::msg::Pose pose;
        pose.position.x = vect_x[i];
        pose.position.y = vect_y[i];
        pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, vect_yaw[i]);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        pose_array_msg.poses.push_back(pose);
      }
    }

    pose_array_msg.header.stamp = this->get_clock()->now();
    pose_array_msg.header.frame_id = "/map";
    this->publish_poses(pose_array_msg);
  }

private:
  void publish_poses(geometry_msgs::msg::PoseArray msg);
  geometry_msgs::msg::Pose rand_hexagon(double dx, double dy);
  geometry_msgs::msg::Pose rand_rectangle(double dx, double dy);
};


geometry_msgs::msg::Pose InitPublisher::rand_hexagon(double dx, double dy){
  // Declare random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(-dx, dx);
  std::uniform_int_distribution<> dis_left_right(0,1);
  obstacle obs {0.0, 0.0, 0.0, 1.0, 1.0, obstacle_type::BOX};

  do {
    obs.x = dis_x(gen);
    obs.y = dis_x(gen);
  } while(!is_inside_map(obs, "hexagon", dx, dy));
  RCLCPP_INFO(this->get_logger(), "Starting in x: %f, y: %f", obs.x, obs.y);
  
  geometry_msgs::msg::Pose pose;
  pose.position.x = obs.x;
  pose.position.y = obs.y;
  pose.position.z = 0.0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;

  return pose;
}


geometry_msgs::msg::Pose InitPublisher::rand_rectangle(double dx, double dy){
  // Declare random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(-dx, dx);
  std::uniform_real_distribution<> dis_y(-dy, dy);
  std::uniform_int_distribution<> dis_edge(0, 3);

  obstacle obs {0.0, 0.0, 0., 1.0, 1.0, obstacle_type::BOX};

  do {
    obs.x = dis_x(gen);
    obs.y = dis_y(gen);
  } while(!is_inside_map(obs, "rectangle", dx, dy));

  RCLCPP_INFO(this->get_logger(), "Starting in x: %f, y: %f", obs.x, obs.y);

  geometry_msgs::msg::Pose pose;
  pose.position.x = obs.x;
  pose.position.y = obs.y;
  pose.position.z = 0.0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  
  return pose;
}


void InitPublisher::publish_poses(geometry_msgs::msg::PoseArray msg){
  this->publisher_->publish(msg);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create the node 
  auto node = std::make_shared<InitPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

