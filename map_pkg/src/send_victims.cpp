#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <random>
#include <fstream>

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
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "map_pkg/utilities.hpp"

/**
 * @brief This class is responsible for generating and publishing victims in the map.
 * @details It is a lifecycle node that either generates random victims in the map, or reads the
 * positions from the parameters. It also subscribes to the /gates topic to get the gates
 * positions, and to the /obstacles topic to get the obstacles positions. It publishes the victims
 * 
 */
class VictimPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  struct Data {
    double max_weight;
    double min_weight;
    std::vector<double> vect_x;
    std::vector<double> vect_y;
    std::vector<double> vect_weight;
  } data ;

public:
  explicit VictimPublisher(bool intra_process_comms = false) 
  : rclcpp_lifecycle::LifecycleNode("victims_sender",
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
    RCLCPP_INFO(this->get_logger(), "Node created.");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring node.");

    // Victims parameters
    this->declare_parameter("max_weight", 100.0);
    this->declare_parameter("min_weight", 1.0);
    this->declare_parameter("vect_x", std::vector<double>());
    this->declare_parameter("vect_y", std::vector<double>());
    this->declare_parameter("vect_weight", std::vector<double>());

    // Get parameters
    this->data.max_weight = this->get_parameter("max_weight").as_double();
    this->data.min_weight = this->get_parameter("min_weight").as_double();
    this->data.vect_x = this->get_parameter("vect_x").as_double_array();
    this->data.vect_y = this->get_parameter("vect_y").as_double_array();
    this->data.vect_weight = this->get_parameter("vect_weight").as_double_array();
    
    // Print parameters values
    RCLCPP_INFO(this->get_logger(), "max_weight: %f", this->data.max_weight);
    RCLCPP_INFO(this->get_logger(), "min_weight: %f", this->data.min_weight);
    RCLCPP_INFO(this->get_logger(), "vect_x: %s", to_string(this->data.vect_x).c_str());
    RCLCPP_INFO(this->get_logger(), "vect_y: %s", to_string(this->data.vect_y).c_str());
    RCLCPP_INFO(this->get_logger(), "vect_weight: %s", to_string(this->data.vect_weight).c_str());
    
    if (!equal_sizes({this->data.vect_x.size(), this->data.vect_y.size(), this->data.vect_weight.size()})) {
      RCLCPP_ERROR(this->get_logger(), "vect_x and vect_y and vect_weigths must have the same size.");
      exit(1);
    }

    //Create publishers
    this->publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/victims", this->qos);
    this->marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers/victims", this->qos);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    obstacles_msgs::msg::ObstacleArrayMsg msg;
    std_msgs::msg::Header hh;
    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    for (uint i=0; i<this->data.vect_x.size(); i++) {
      Victim vict = {this->data.vect_x[i], this->data.vect_y[i], this->data.vect_weight[i]};
      this->publish_victim(vict, msg);
    }
    
    this->publish_victims(msg);
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node.");
    LifecycleNode::deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void rand_victims(std::vector<Obstacle>& victims);
  void publish_victim(const Victim& vict, obstacles_msgs::msg::ObstacleArrayMsg& msg);
  void publish_victims(const obstacles_msgs::msg::ObstacleArrayMsg& msg);
};


void VictimPublisher::publish_victim(const Victim& vict, obstacles_msgs::msg::ObstacleArrayMsg& msg) 
{

  RCLCPP_INFO(this->get_logger(), "Adding victim at x=%f, y=%f", vict.x, vict.y);

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
  obs.radius = vict.radius;

  msg.obstacles.push_back(obs);
}


void VictimPublisher::publish_victims(const obstacles_msgs::msg::ObstacleArrayMsg& msg){
  RCLCPP_INFO(this->get_logger(), "Publishing victims");
  visualization_msgs::msg::MarkerArray markers;

  for (size_t vict_id = 0; vict_id < msg.obstacles.size(); vict_id++) {
    std_msgs::msg::Header hh = msg.header;
    geometry_msgs::msg::Pose pose;
    const geometry_msgs::msg::Point32 point = msg.obstacles[vict_id].polygon.points[0];
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.y;
    double radius = msg.obstacles[vict_id].radius;

    // markers for rviz
    visualization_msgs::msg::Marker pos_marker;
    pos_marker.header = hh;
    pos_marker.header.frame_id = "map";
    pos_marker.ns = "victims";
    pos_marker.id = vict_id;
    pos_marker.action = visualization_msgs::msg::Marker::ADD;
    pos_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    pos_marker.pose = pose;
    pos_marker.scale.x = radius/this->data.max_weight*1.0;
    pos_marker.scale.y = radius/this->data.max_weight*1.0;
    pos_marker.scale.z = 0.1;
    pos_marker.color.a = 0.5;
    pos_marker.color.r = 0.0;
    pos_marker.color.g = 0.0;
    pos_marker.color.b = 1.0;
    markers.markers.push_back(pos_marker);

    visualization_msgs::msg::Marker weight_marker;
    weight_marker.header = hh;
    weight_marker.header.frame_id = "map";
    weight_marker.ns = "victims_weight";
    weight_marker.id = vict_id;
    weight_marker.action = visualization_msgs::msg::Marker::ADD;
    weight_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    weight_marker.pose = pose;
    weight_marker.scale.z = 0.9;
    weight_marker.color.a = 1.0;
    weight_marker.color.r = 1.0;
    weight_marker.color.g = 0.5;
    weight_marker.color.b = 1.0;
    weight_marker.text = std::to_string((int)radius); 
    markers.markers.push_back(weight_marker);
  }
  RCLCPP_INFO(this->get_logger(), "[1] Publishing victims.");
  this->publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "[2] Victims published, publishing markers.");
  this->marker_publisher_->publish(markers);
  RCLCPP_INFO(this->get_logger(), "[3] Markers published.");
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VictimPublisher>();
  node->configure();
  node->activate();

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}