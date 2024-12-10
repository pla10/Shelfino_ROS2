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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "map_pkg/utilities.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Class to spawn the gate(s) in Gazebo and also publish the position of the gate 
 * in the map. The gate can be spawned in a random position in a hexagon or rectangle map,
 * or its position can be passed as a parameter.
 * 
 */
class GatesPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;

  struct Data {
    std::string map_name;
    double dx;
    double dy;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> th;
  } data ;

public:
  explicit GatesPublisher(bool intra_process_comms = false) 
  : rclcpp_lifecycle::LifecycleNode("send_gates",
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
    RCLCPP_INFO(this->get_logger(), "Node created.");
  }

  /**
   * @brief This function is called to deactivate the node
   * 
   * @param state State
   * @return CallbackReturn SUCCESS
   */
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node %s.", this->get_name());
    LifecycleNode::deactivate();
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief This function is called to configure the node. It maps the parameters and
   * creates the publisher and client to publish the gate position and spawn the gate in
   * Gazebo, respectively.
   * 
   * @param state State
   * @return CallbackReturn SUCCESS
   */
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring node %s.", this->get_name());
    LifecycleNode::on_configure(state);

    // Map parameters
    this->declare_parameter("map", "hexagon");
    this->declare_parameter("dx", 10.0);
    this->declare_parameter("dy", 10.0);

    // Gate parameters
    this->declare_parameter("x", std::vector<double>());
    this->declare_parameter("y", std::vector<double>());
    this->declare_parameter("yaw", std::vector<double>());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    
    this->publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/gates", qos);

    // Get parameters
    this->data.map_name = this->get_parameter("map").as_string();
    if (this->data.map_name != "hexagon" && this->data.map_name != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
      exit(1);
    }

    this->data.dx = this->get_parameter("dx").as_double();
    this->data.dy = this->get_parameter("dy").as_double();
    this->data.x = this->get_parameter("x").as_double_array();
    this->data.y = this->get_parameter("y").as_double_array();
    this->data.th = this->get_parameter("yaw").as_double_array();

    if (this->data.x.size() != this->data.y.size() || this->data.x.size() != this->data.th.size()){
      RCLCPP_ERROR(this->get_logger(), "The number of x, y, and th must be the same.");
      exit(1);
    }

    for (size_t i=0; i<this->data.x.size(); i++){
      RCLCPP_INFO(this->get_logger(), "Gate %ld: x=%f, y=%f, th=%f", i, this->data.x[i], this->data.y[i], this->data.th[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Node %s configured.", this->get_name());

    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief This function is called to activate the node. It will check if the gate(s) 
   * coordinates are passed, otherwise it will generate random coordinates for the
   * gates based on the map type.
   * 
   * @param state State
   * @return CallbackReturn SUCCESS 
   */
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating node %s.", this->get_name());
    LifecycleNode::on_activate(state);
    
    std::vector<geometry_msgs::msg::Pose> pose_array_temp;

    for (size_t i=0; i<this->data.x.size(); i++){
      this->publish_gate(this->data.x[i], this->data.y[i], this->data.th[i], pose_array_temp);
    }

    geometry_msgs::msg::PoseArray msg;
    // Set headers of messages
    std_msgs::msg::Header hh;
    hh.stamp = this->get_clock()->now();
    hh.frame_id = "/map";
    msg.header = hh;

    // Add gates to the message
    msg.poses = pose_array_temp;

    // Publish message
    publisher_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "Node %s activated.", this->get_name());
    return CallbackReturn::SUCCESS;
  }

private:
  /**
   * @brief Publish the position of the gate in the hexagon map and spawns the model of the 
   * gate in Gazebo.
   * 
   * @param[in] x Abscissa of the gate
   * @param[in] y Ordinate of the gate
   * @param[in] th Orientation of the gate
   */
  void publish_gate(double x, double y, double th, std::vector<geometry_msgs::msg::Pose>& pose_array_temp){
    geometry_msgs::msg::Pose pose;

    // Set position of the gate
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.05;

    tf2::Quaternion q;
    q.setRPY(0, 0, th);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_array_temp.push_back(pose);
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create the node 
  auto node = std::make_shared<GatesPublisher>();
  node->configure();
  node->activate();
  RCLCPP_INFO(node->get_logger(), "Adding node.");
  rclcpp::spin(node->get_node_base_interface());
  // exe->spin();
  rclcpp::shutdown();
  return 0;
}

