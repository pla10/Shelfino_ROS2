#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <random>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "map_pkg/utilities.hpp"

class ObstaclesPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
  
  // Data obtained from parameters
  struct Data {
    std::vector<std::string> vect_type;
    std::vector<double> vect_x;
    std::vector<double> vect_y;
    std::vector<double> vect_yaw;
    std::vector<double> vect_dim_x;
    std::vector<double> vect_dim_y;
  } data;

  std::vector<Obstacle> gates;

public:
  explicit ObstaclesPublisher(bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode("send_obstacles", 
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
    RCLCPP_INFO(this->get_logger(), "Node created.");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state)
  {
    // Obstacle parameters
    try{
      this->declare_parameter("n_obstacles", 3);
      this->declare_parameter("no_cylinders", false);
      this->declare_parameter("no_boxes", false);
      this->declare_parameter("min_size", 0.5);
      this->declare_parameter("max_size", 1.5);
      this->declare_parameter("vect_type", std::vector<std::string>());
      this->declare_parameter("vect_x", std::vector<double>());
      this->declare_parameter("vect_y", std::vector<double>());
      this->declare_parameter("vect_yaw", std::vector<double>());
      this->declare_parameter("vect_dim_x", std::vector<double>());
      this->declare_parameter("vect_dim_y", std::vector<double>());
    } catch (...){
      RCLCPP_ERROR(this->get_logger(), "Error declaring parameters");
    }

    // Get parameters
    try{
      this->data.vect_type = this->get_parameter("vect_type").as_string_array();
    } catch (...){
      RCLCPP_ERROR(this->get_logger(), "Error getting parameter vect_type");
    }
    try{
      this->data.vect_x = this->get_parameter("vect_x").as_double_array();
    } catch (...){
      RCLCPP_ERROR(this->get_logger(), "Error getting parameter vect_x");
    }
    try{
      this->data.vect_y = this->get_parameter("vect_y").as_double_array();
    } catch (...){
      RCLCPP_ERROR(this->get_logger(), "Error getting parameter vect_y");
    }
    try{
      this->data.vect_yaw = this->get_parameter("vect_yaw").as_double_array();
    } catch (...){
      RCLCPP_ERROR(this->get_logger(), "Error getting parameter vect_yaw");
    }
    try{
      this->data.vect_dim_x = this->get_parameter("vect_dim_x").as_double_array();
    } catch (...){
      RCLCPP_ERROR(this->get_logger(), "Error getting parameter vect_dim_x");
    }
    try{
      this->data.vect_dim_y = this->get_parameter("vect_dim_y").as_double_array();
    } catch (...){
      RCLCPP_ERROR(this->get_logger(), "Error getting parameter vect_dim_y");
    }

    // Print parameters values
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "vect_type: %s", to_string(this->data.vect_type).c_str());
    RCLCPP_INFO(this->get_logger(), "vect_x: %s", to_string(this->data.vect_x).c_str());
    RCLCPP_INFO(this->get_logger(), "vect_y: %s", to_string(this->data.vect_y).c_str());
    RCLCPP_INFO(this->get_logger(), "vect_yaw: %s", to_string(this->data.vect_yaw).c_str());
    RCLCPP_INFO(this->get_logger(), "vect_dim_x: %s", to_string(this->data.vect_dim_x).c_str());
    RCLCPP_INFO(this->get_logger(), "vect_dim_y: %s", to_string(this->data.vect_dim_y).c_str());
    

    if (!equal_sizes({this->data.vect_type.size(), this->data.vect_x.size(), this->data.vect_y.size(), 
                     this->data.vect_yaw.size(), this->data.vect_dim_x.size(), this->data.vect_dim_y.size()})) 
    {
      RCLCPP_ERROR(this->get_logger(), "The number of elements in the vectors type, x (%ld), y (%ld), yaw (%ld) and the dimensions must be the same.", 
        this->data.vect_x.size(), this->data.vect_y.size(), this->data.vect_yaw.size()
      );
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Create callback groups for gates and obstacles
    auto cb_group_gates = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options_gates;
    sub_options_gates.callback_group = cb_group_gates;

    this->publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/obstacles", this->qos);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  { 
    RCLCPP_INFO(this->get_logger(), "Activating node %s.", this->get_name());

    std::vector<Obstacle> obstacles;
    for (uint i=0; i<this->data.vect_type.size(); i++){
      Obstacle obs;
      if (this->data.vect_type[i] == "cylinder"){
        obs = Obstacle{this->data.vect_dim_x[0], this->data.vect_x[i], this->data.vect_y[i], 0.0, 0.0, 0.0, OBSTACLE_TYPE::CYLINDER};
      }
      else if (this->data.vect_type[i] == "box"){
        obs = Obstacle{0.0, this->data.vect_x[i], this->data.vect_y[i], this->data.vect_dim_x[i], this->data.vect_dim_y[i], this->data.vect_yaw[i], OBSTACLE_TYPE::BOX};
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Type %s is not valid. It must be either cylinder or box.", this->data.vect_type[i].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
      }
      // if (!valid_position(this->data.map_name, this->data.dx, this->data.dy, obs, {obstacles, this->gates})){
      //   RCLCPP_ERROR(this->get_logger(), "The Obstacle %s is not valid.", this->data.vect_type[i].c_str());
      //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
      // }
      obstacles.push_back(obs);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing %ld obstacles", obstacles.size());

    this->publish_obstacles(obstacles);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; 
  }

  void publish_obstacles(std::vector<Obstacle>& obstacles);
};



void ObstaclesPublisher::publish_obstacles(std::vector<Obstacle>& obstacles)
{
  obstacles_msgs::msg::ObstacleArrayMsg msg;
  std_msgs::msg::Header hh;
  hh.stamp = this->get_clock()->now();
  hh.frame_id = "map";

  for (auto o : obstacles) {
    RCLCPP_INFO(this->get_logger(), "Publishing Obstacle: %s x=%f, y=%f, yaw=%f, radius=%f, dx=%f, dy=%f", 
      (o.type==OBSTACLE_TYPE::BOX ? "box" : "cylinder"), o.x, o.y, o.radius, o.yaw, o.dx, o.dy);

    obstacles_msgs::msg::ObstacleMsg obs;
    if (o.type == OBSTACLE_TYPE::CYLINDER){
      geometry_msgs::msg::Point32 point;
      point.x = o.x;
      point.y = o.y;
      point.z = 0.0;

      geometry_msgs::msg::Polygon pol;
      pol.points.push_back(point);

      obs.polygon = pol;
      obs.radius = o.radius;
    }
    else {
      geometry_msgs::msg::Polygon pol = create_rectangle(o.dx, o.dy, o.x, o.y, o.yaw);
      obs.polygon = pol;
    }
    msg.obstacles.push_back(obs);
  }

  this->publisher_->publish(msg);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstaclesPublisher>();

  node->configure();
  node->activate();

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  
  rclcpp::shutdown();
  return 0;
}

