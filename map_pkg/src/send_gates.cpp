#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "gazebo_msgs/srv/spawn_entity.hpp"

//TODO It would be nice to wait for the service response.

#define MYDEBUG RCLCPP_INFO

class GatesPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  std::string node_namespace;


public:
  GatesPublisher(int argc, char* argv[]) : Node("send_gates")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    this->publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("gates_position", qos);
    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    // MYDEBUG(rclcpp::get_logger("rclcpp"), "Subscribed to service %s", this->spawner_->get_service_name());
    this->node_namespace = this->get_namespace();

    // Initiate timer
    if (std::string(argv[1]) == "lab1"){
      this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), 
        std::bind(&GatesPublisher::lab1_callback, this)
      );
    }
    else if (std::string(argv[1]) == "hexagon"){
      this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), 
        std::bind(&GatesPublisher::hexagon_callback, this)
      );
    }
    else{
      MYDEBUG(rclcpp::get_logger("rclcpp"), "Not a valid map, using random argv[1]=%s", (argc>1 ? argv[1] : ""));
      this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), 
        std::bind(&GatesPublisher::random_callback, this)
      );
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "send_gates node started.");
  }

private:
  void random_callback();
  void lab1_callback();
  void hexagon_callback();
  void spawn_gate(
    const geometry_msgs::msg::Pose& pose
  );
};

void GatesPublisher::random_callback(){
  std_msgs::msg::Header hh;
  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Pose> pose_array_temp;
  geometry_msgs::msg::PoseArray msg;

  hh.stamp = this->get_clock()->now();
  hh.frame_id = "map";

  msg.header = hh;

  pose.position.x = -5;
  pose.position.y = -2.5;
  //pose.position.x = -2.81;
  //pose.position.y = -4.46;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  pose_array_temp.push_back(pose);
  pose.position.x = -4;
  pose.position.y = 5;
  //pose.position.x = -1.36;
  //pose.position.y = -1.11;

  pose_array_temp.push_back(pose);
  msg.poses = pose_array_temp;

  publisher_->publish(msg);
}

void GatesPublisher::lab1_callback(){
  std_msgs::msg::Header hh;
  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Pose> pose_array_temp;
  geometry_msgs::msg::PoseArray msg;

  // Set headers of messages
  hh.stamp = this->get_clock()->now();
  hh.frame_id = "map";
  msg.header = hh;

  // Set position of the gate
  pose.position.x = -1.59;
  pose.position.y = -2.69;
  pose.position.z = 0;
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
  this->spawn_gate(pose);
}

/**
 * @brief Publish the position of the gate in the hexagon map
 * 
 */
void GatesPublisher::hexagon_callback(){
  std_msgs::msg::Header hh;
  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Pose> pose_array_temp;
  geometry_msgs::msg::PoseArray msg;

  // Set headers of messages
  hh.stamp = this->get_clock()->now();
  hh.frame_id = "map";
  msg.header = hh;

  // Set position of the gate
  pose.position.x = 0;
  pose.position.y = 9.5;
  pose.position.z = 0;
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
  this->spawn_gate(pose);
}

/**
 * @brief Spawn model of the gate in gazebo
 * 
 * @param pose The position in which the gate should be spawned
 */
void GatesPublisher::spawn_gate(const geometry_msgs::msg::Pose& pose){
  MYDEBUG(rclcpp::get_logger("rclcpp"), "Waiting for service spawn_entity...");
  while(!this->spawner_->wait_for_service(std::chrono::milliseconds(100))){
    if (!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown(nullptr, "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Configure request
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = "gate";
  request->initial_pose = pose;
  request->robot_namespace = this->node_namespace;
  request->xml = std::string((
    std::istreambuf_iterator<char>(std::ifstream("/home/enrico/shelfino_ros2_ws/src/map_pkg/models/gate/model.sdf").rdbuf())), std::istreambuf_iterator<char>());

  // Send request
  auto result = this->spawner_->async_send_request(request);
}

int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting send_gates node...");
  MYDEBUG(rclcpp::get_logger("rclcpp"), "argc=%d, argv=%s", argc, (argc > 1 ? argv[1] : ""));
  for (int i = 0; i < argc; i++) {
    MYDEBUG(rclcpp::get_logger("rclcpp"), "argv[%d]=%s", i, argv[i]);
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GatesPublisher>(argc, argv));
  rclcpp::shutdown();
  return 0;
}

