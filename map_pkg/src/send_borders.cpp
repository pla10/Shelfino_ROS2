#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"

#include "gazebo_msgs/srv/spawn_entity.hpp"

#include "map_pkg/spawn_model.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

geometry_msgs::msg::Polygon create_hexagon(double dx){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        double f = 0.866;  // fixed number for apothem of hexagon calculation
        point.x = -dx/2;
        point.y = dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = -dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx/2;
        point.y = -dx*f;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}


geometry_msgs::msg::Polygon create_rectangle(double dx, double dy){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = -dx/2; 
        point.y = -dy/2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -dx/2; 
        point.y = dy/2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = dy/2; 
        point.z = 0;
        points_temp.push_back(point);
        point.x = dx/2;
        point.y = -dy/2; 
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}

static const rmw_qos_profile_t rmw_qos_profile_custom =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

class BordersPublisher : public rclcpp::Node
{
private:
  std::string share_dir;
  std::string gz_models;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;

public:
  BordersPublisher()
  : Node("send_borders")
  {
    this->share_dir = ament_index_cpp::get_package_share_directory("map_pkg");
    this->gz_models = ament_index_cpp::get_package_share_directory("shelfino_description");

    this->declare_parameter<std::string>("map", "hexagon");
    this->declare_parameter<double>("dx", 5.0);
    this->declare_parameter<double>("dy", 5.0);
    std::string map_name = this->get_parameter("map").as_string();  // hexagon, rectangle
    double dx = this->get_parameter("dx").as_double();
    double dy = this->get_parameter("dy").as_double();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("/map_borders", qos);

    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    std_msgs::msg::Header hh;

    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    geometry_msgs::msg::Polygon pol;

    geometry_msgs::msg::PolygonStamped pol_stamped;

    pol_stamped.header = hh;

    std::string xml_string;
    if(map_name=="hexagon"){
      pol = create_hexagon(dx);
      // Read XML file to string
      std::ifstream xml_file(this->gz_models + "/models/hexagon_world/model.sdf");
      xml_string.assign(
        std::istreambuf_iterator<char>(xml_file),
        std::istreambuf_iterator<char>()
      );
      float original_size = 12.00;     // 13.20
      std::string size_string = "<scale>1 1 1</scale>";
      std::string size_replace_string = "<scale>" + std::to_string(dx/original_size) + " " + std::to_string(dx/original_size) + " 1</scale>";
      size_t pos = 0;
      while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
        xml_string.replace(pos, size_string.length(), size_replace_string);
        pos += size_replace_string.length();
      }
      // Spawn model in gazebo
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.0;
      pose.position.y = 0.0;
      pose.position.z = 0.1;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 0;

      spawn_model(this->get_node_base_interface(), this->spawner_, xml_string, pose);
    }else if(map_name=="rectangle"){
      pol = create_rectangle(dx,dy);
    }
    
    pol_stamped.polygon = pol;

    pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/borders", qos);

    publisher_->publish(pol);
    pub_->publish(pol_stamped);
    usleep(1000000);
  }
  
private:
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BordersPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}