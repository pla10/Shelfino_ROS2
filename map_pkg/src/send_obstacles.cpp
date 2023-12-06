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

enum obstacle_type {
  CYLINDER,
  BOX
};

struct obstacle {
  double radius;
  double x, y;
  double dx, dy;
  obstacle_type type;
  std::string xml_file = "";
};

// Function that checks that two obstacles do not overlap
bool check_overlap(obstacle o1, obstacle o2) {
  if (o1.type == obstacle_type::CYLINDER && o2.type == obstacle_type::CYLINDER) {
    double dist = sqrt(pow(o1.x - o2.x, 2) + pow(o1.y - o2.y, 2));
    if (dist < o1.radius + o2.radius) {
      return false;
    }
  } 
  else if (o1.type == obstacle_type::BOX && o2.type == obstacle_type::BOX) {
    if (abs(o1.x - o2.x) < (o1.dx + o2.dx)/2.0 && abs(o1.y - o2.y) < (o1.dy + o2.dy)/2.0) {
      return false;
    }
  } 
  else {
    if (o1.type == obstacle_type::CYLINDER) {
      obstacle temp = o1;
      o1 = o2;
      o2 = temp;
    }
    if (abs(o1.x - o2.x) < o1.radius + o2.dx/2.0 && abs(o1.y - o2.y) < o1.radius + o2.dy/2.0) {
      return false;
    }
  }
  return true;
}

bool check_overlap(obstacle o1, std::vector<obstacle> obstacles){
  for (auto o2 : obstacles){
    if (!check_overlap(o1, o2)){
      return false;
    }
  }
  return true;
}


class ObstaclesPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  std::string node_namespace;

public:
  ObstaclesPublisher() : Node("obstacles_sender")
  {
    // Parameters
    this->declare_parameter("n_obstacles", 3);
    this->declare_parameter("no_cylinders", false);
    this->declare_parameter("no_boxes", false);
    this->declare_parameter("min_radius", 0.5);
    this->declare_parameter("max_radius", 1.5);

    if (this->get_parameter("no_cylinders").as_bool() && this->get_parameter("no_boxes").as_bool()) {
      RCLCPP_ERROR(this->get_logger(), "Both no_cylinders and no_boxes are true. At least one of them must be false.");
      exit(1);
    }

    this->node_namespace = this->get_namespace();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    std::vector<obstacle> obstacles;

    // Print parameters values
    RCLCPP_INFO(this->get_logger(), "n_obstacles: %ld", this->get_parameter("n_obstacles").as_int());
    RCLCPP_INFO(this->get_logger(), "no_cylinders: %d", this->get_parameter("no_cylinders").as_bool());
    RCLCPP_INFO(this->get_logger(), "no_boxes: %d", this->get_parameter("no_boxes").as_bool());
    RCLCPP_INFO(this->get_logger(), "min_radius: %f", this->get_parameter("min_radius").as_double());
    RCLCPP_INFO(this->get_logger(), "max_radius: %f", this->get_parameter("max_radius").as_double());

    // Define a random number generator for doubles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rad_dis(
      this->get_parameter("min_radius").as_double(), 
      this->get_parameter("max_radius").as_double()
    ); 
    std::uniform_real_distribution<> x_dis(-10, 10);
    std::uniform_real_distribution<> y_dis(-10, 10);

    int n_obstacles = this->get_parameter("n_obstacles").as_int();
    for (int i=0; i<n_obstacles; i++) {
      double radius = 0.0, dx = 0.0, dy = 0.0, x = 0.0, y = 0.0;

      if (this->get_parameter("no_cylinders").as_bool()) {
        do {
          dx = rad_dis(gen);
          dy = rad_dis(gen);
          x = x_dis(gen);
          y = y_dis(gen);
        }
        while(!check_overlap(obstacle{0.0, x, y, dx, dy, obstacle_type::BOX}, obstacles));
        obstacles.push_back(obstacle{0.0, x, y, dx, dy, obstacle_type::BOX});
      } 
      else if (this->get_parameter("no_boxes").as_bool()) {
        do {
          radius = rad_dis(gen);
          x = x_dis(gen);
          y = y_dis(gen);
        } while(!check_overlap(obstacle{radius, x, y, 0.0, 0.0, obstacle_type::CYLINDER}, obstacles));
        obstacles.push_back(obstacle{radius, x, y, 0.0, 0.0, obstacle_type::CYLINDER});
      } 
      else {
        obstacle_type type = (obstacle_type)(rand() % 2);
        if (type == obstacle_type::CYLINDER) {
          do {
            radius = rad_dis(gen);
            x = x_dis(gen);
            y = y_dis(gen);
          } while(!check_overlap(obstacle{radius, x, y, 0.0, 0.0, obstacle_type::CYLINDER}, obstacles));
          obstacles.push_back(obstacle{radius, x, y, 0.0, 0.0, obstacle_type::CYLINDER});
        }
        else {
          do {
            dx = rad_dis(gen);
            dy = rad_dis(gen);
            x = x_dis(gen);
            y = y_dis(gen);
          } while(!check_overlap(obstacle{radius, x, y, 0.0, 0.0, obstacle_type::CYLINDER}, obstacles));
          obstacles.push_back(obstacle{0.0, x, y, dx, dy, obstacle_type::BOX});
        }
      }
    }

    publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos);

    obstacles_msgs::msg::ObstacleArrayMsg msg;
    std_msgs::msg::Header hh;
    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    for (auto o : obstacles) {
      RCLCPP_INFO(this->get_logger(), "Publishing obstacle: %s x=%f, y=%f, radius=%f, dx=%f, dy=%f", 
        (o.type==obstacle_type::BOX ? "box" : "cylinder"), o.x, o.y, o.radius, o.dx, o.dy);

      obstacles_msgs::msg::ObstacleMsg obs;
      geometry_msgs::msg::Polygon pol;
      if (o.type == obstacle_type::CYLINDER){
        geometry_msgs::msg::Point32 point;
        point.x = o.x;
        point.y = o.y;
        point.z = 0.0;
        pol.points.push_back(point);
        obs.polygon = pol;
        obs.radius = o.radius;
      }
      else {
        geometry_msgs::msg::Point32 point;
        point.x = o.x - o.dx/2.0;
        point.y = o.y - o.dy/2.0;
        point.z = 0.0;
        pol.points.push_back(point);
        point.x = o.x - o.dx/2.0;
        point.y = o.y + o.dy/2.0;
        point.z = 0.0;
        pol.points.push_back(point);
        point.x = o.x + o.dx/2.0;
        point.y = o.y + o.dy/2.0;
        point.z = 0.0;
        pol.points.push_back(point);
        point.x = o.x + o.dx/2.0;
        point.y = o.y - o.dy/2.0;
        point.z = 0.0;
        pol.points.push_back(point);
        obs.polygon = pol;
      }
      msg.obstacles.push_back(obs);

      // Read XML file to string
      std::ifstream xml_file("/home/enrico/shelfino_ros2_ws/src/map_pkg/models/box/model.sdf");
      std::string xml_string;
      xml_string.assign(
        std::istreambuf_iterator<char>(xml_file),
        std::istreambuf_iterator<char>()
      );
      // In string, replace all occurences of <size>1 1 1</size> with <size>dx dy dz</size>
      // std::string size_string = "<size>1 1 1</size>";
      // std::string size_replace_string = "<size>" + std::to_string(obstacles[0].dx) + " " + std::to_string(obstacles[0].dy) + " 1</size>";
      // size_t pos = 0;
      // while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
      //   xml_string.replace(pos, size_string.length(), size_replace_string);
      //   pos += size_replace_string.length();
      // }

      // Spawn model in gazebo
      geometry_msgs::msg::Pose pose;
      pose.position.x = o.x;
      pose.position.y = o.y;
      pose.position.z = 0.01;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 0;

      this->spawn_obstacle(xml_string, pose);
      sleep(0.5);
    }



    // publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos);

    // std_msgs::msg::Header hh;
    // hh.stamp = this->get_clock()->now();
    // hh.frame_id = "map";

    // obstacles_msgs::msg::ObstacleArrayMsg msg;
    // obstacles_msgs::msg::ObstacleMsg obs;
    // std::vector<obstacles_msgs::msg::ObstacleMsg> obs_temp;
    // geometry_msgs::msg::Polygon pol;
    // geometry_msgs::msg::Point32 point;

    // geometry_msgs::msg::PolygonStamped pol1;
    // geometry_msgs::msg::PolygonStamped pol2;

    // pol1.header = hh;
    // pol2.header = hh;

    // // First square obstacle
    // {
    //   std::vector<geometry_msgs::msg::Point32> points_temp;
    //   point.x = 0;
    //   point.y = 0;
    //   // point.x = -1.59;
    //   // point.y = -2.69;
    //   point.z = 0;
    //   points_temp.push_back(point);
    //   point.x = 0;
    //   point.y = 1;
    //   // point.x = -1.25;
    //   // point.y = -2.45;
    //   point.z = 0;
    //   points_temp.push_back(point);
    //   point.x = 1;
    //   point.y = 1;
    //   // point.x = -1.06;
    //   // point.y = -2.79;
    //   point.z = 0;
    //   points_temp.push_back(point);
    //   point.x = 1;
    //   point.y = 0;
    //   // point.x = -1.38;
    //   // point.y = -2.99;

    //   points_temp.push_back(point);
    //   pol.points = points_temp;
    //   obs.polygon = pol;
    //   obs_temp.push_back(obs);
    //   pol1.polygon = pol;
    // }

    // // First square obstacle
    // {
    //   std::vector<geometry_msgs::msg::Point32> points_temp;
    //   point.x = -1;
    //   point.y = -1;
    //   //point.x = 0.953;
    //   //point.y = -1.9;
    //   point.z = 0;
    //   points_temp.push_back(point);
    //   point.x = -1;
    //   point.y = -2;
    //   //point.x = 0.762;
    //   //point.y = -1.49;
    //   point.z = 0;
    //   points_temp.push_back(point);
    //   point.x = -2;
    //   point.y = -2;
    //   //point.x = 1.08;
    //   //point.y = -1.31;
    //   point.z = 0;
    //   points_temp.push_back(point);
    //   point.x = -2;
    //   point.y = -1;
    //   //point.x = 1.28;
    //   //point.y = -1.69;
    //   point.z = 0;

    //   points_temp.push_back(point);
    //   pol.points = points_temp;
    //   obs.polygon = pol;
    //   obs_temp.push_back(obs);
    //   pol2.polygon = pol;
    // }

    // msg.obstacles = obs_temp;

    // rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub1 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("obs1", 10);
    // rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub2 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("obs2", 10);

    // while(1){
    //   publisher_->publish(msg);
    //   pub1->publish(pol1);
    //   pub2->publish(pol2);
    //   usleep(1000000);
    // }
  }

private:
  void spawn_obstacle(std::string xml, geometry_msgs::msg::Pose pose);

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclesPublisher>());
  rclcpp::shutdown();
  return 0;
}


/**
 * @brief Spawn model of the gate in gazebo
 * 
 * @param pose The position in which the gate should be spawned
 */
void ObstaclesPublisher::spawn_obstacle(std::string xml, geometry_msgs::msg::Pose pose){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service spawn_entity...");
  while(!this->spawner_->wait_for_service(std::chrono::milliseconds(100))){
    if (!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown(nullptr, "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Configure request
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = "obstacle"+std::to_string(pose.position.x)+"_"+std::to_string(pose.position.y);
  request->initial_pose = pose;
  request->robot_namespace = this->node_namespace;
  request->xml = xml;

  // Send request
  auto result = this->spawner_->async_send_request(request);
}