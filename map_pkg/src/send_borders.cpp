#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"

#include "gazebo_msgs/srv/spawn_entity.hpp"

#include "map_pkg/spawn_model.hpp"
#include "map_pkg/utilities.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

class BordersPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  std::string gz_models;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;

  // Data obtained from parameters
  struct Data {
    std::string map_name;
    double dx;
    double dy;
  } data ;

public:
  explicit BordersPublisher(bool intra_process_comms = false) 
  : rclcpp_lifecycle::LifecycleNode("send_borders", 
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
    this->gz_models = ament_index_cpp::get_package_share_directory("shelfino_gazebo");
    this->configure();
    this->activate();
    this->deactivate();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State& state)
  {
    this->declare_parameter<std::string>("map", "hexagon");
    this->declare_parameter<double>("dx", 5.0);
    this->declare_parameter<double>("dy", 5.0);
    this->data.map_name = this->get_parameter("map").as_string();  // hexagon, rectangle
    this->data.dx = this->get_parameter("dx").as_double();
    this->data.dy = this->get_parameter("dy").as_double();

    RCLCPP_INFO(this->get_logger(), "Map name: %s", this->data.map_name.c_str());
    RCLCPP_INFO(this->get_logger(), "dx: %f", this->data.dx);
    RCLCPP_INFO(this->get_logger(), "dy: %f", this->data.dy);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    this->publisher_  = this->create_publisher<geometry_msgs::msg::Polygon>("/map_borders", qos);
    this->pub_        = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/borders", qos);
    this->spawner_    = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    std_msgs::msg::Header hh;

    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";
    
    geometry_msgs::msg::Polygon pol;
    std::string xml_string;
    
    if(this->data.map_name=="hexagon"){
      this->map_hexagon(xml_string, pol);
    }
    else if(this->data.map_name=="rectangle"){
      this->map_rectangle(xml_string, pol);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Map name %s not recognized", this->data.map_name.c_str());
    }
    
    geometry_msgs::msg::PolygonStamped pol_stamped;
    pol_stamped.header = hh;
    pol_stamped.polygon = pol;
    
    // Publish borders
    this->publisher_->publish(pol);
    this->pub_->publish(pol_stamped);

    // Spawn model in gazebo
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
    spawn_model(this->get_node_base_interface(), this->spawner_, xml_string, pose, "border", true);
  
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  
private:
  void map_hexagon(std::string & xml_string, geometry_msgs::msg::Polygon & pol);
  void map_rectangle(std::string & xml_string, geometry_msgs::msg::Polygon & pol);
};

void 
BordersPublisher::map_hexagon(std::string & xml_string, geometry_msgs::msg::Polygon & pol)
{
  pol = create_hexagon(this->data.dx);
  // Read XML file to string
  std::ifstream xml_file(this->gz_models + "/worlds/hexagon_world/model.sdf");
  if (!xml_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file %s", 
      (this->gz_models + "/worlds/hexagon_world/model.sdf").c_str()
    );
    exit(1);
  }
  xml_string.assign(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>()
  );

  float original_size = 12.00;     // 13.20
  std::string size_string = "<scale>1 1 1</scale>";
  std::string size_replace_string = "<scale>" + 
    std::to_string(this->data.dx/original_size) + " " + 
    std::to_string(this->data.dx/original_size) + " 1</scale>";
  
  size_t pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
}

void 
BordersPublisher::map_rectangle(std::string & xml_string, geometry_msgs::msg::Polygon & pol)
{
  pol = create_rectangle(this->data.dx, this->data.dy);
  // Read XML file to string
  std::ifstream xml_file(this->gz_models + "/worlds/rectangle_world/model.sdf");
  if (!xml_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file %s", 
      (this->gz_models + "/worlds/rectangle_world/model.sdf").c_str()
    );
    exit(1);
  }
  xml_string.assign(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>()
  );

  float wid = 0.15;
  std::string size_string = "dx";
  std::string size_replace_string = std::to_string(this->data.dx);
  size_t pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
  size_string = "dy";
  size_replace_string = std::to_string(this->data.dy);
  pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
  size_string = "width";
  size_replace_string = std::to_string(wid);
  pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
  size_string = "L1";
  size_replace_string = std::to_string((this->data.dy+wid)/2);
  pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
  size_string = "L2";
  size_replace_string = std::to_string((this->data.dx+wid)/2);
  pos = 0;
  while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
    xml_string.replace(pos, size_string.length(), size_replace_string);
    pos += size_replace_string.length();
  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BordersPublisher>();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}