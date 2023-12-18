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

class ObstaclesPublisher : public rclcpp::Node
{
private:
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  std::string node_namespace;
  std::string share_dir;

public:
  ObstaclesPublisher() : Node("send_obstacles")
  {
    this->share_dir = ament_index_cpp::get_package_share_directory("map_pkg");
    this->node_namespace = this->get_namespace();

    // Map parameters
    this->declare_parameter("map", "hexagon");
    this->declare_parameter("dx", 10.0);
    this->declare_parameter("dy", 10.0);
    this->declare_parameter("max_timeout", 3);

    // Obstacle parameters
    this->declare_parameter("n_obstacles", 3);
    this->declare_parameter("no_cylinders", false);
    this->declare_parameter("no_boxes", false);
    this->declare_parameter("min_size", 0.5);
    this->declare_parameter("max_size", 1.5);

    // Print parameters values
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    
    RCLCPP_INFO(this->get_logger(), "map: %s", this->get_parameter("map").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "dx: %f", this->get_parameter("dx").as_double());
    RCLCPP_INFO(this->get_logger(), "dy: %f", this->get_parameter("dy").as_double());
    RCLCPP_INFO(this->get_logger(), "max_timeout: %ld", this->get_parameter("max_timeout").as_int());
    RCLCPP_INFO(this->get_logger(), "n_obstacles: %ld", this->get_parameter("n_obstacles").as_int());
    RCLCPP_INFO(this->get_logger(), "no_cylinders: %d", this->get_parameter("no_cylinders").as_bool());
    RCLCPP_INFO(this->get_logger(), "no_boxes: %d", this->get_parameter("no_boxes").as_bool());
    RCLCPP_INFO(this->get_logger(), "min_size: %f", this->get_parameter("min_size").as_double());
    RCLCPP_INFO(this->get_logger(), "max_size: %f", this->get_parameter("max_size").as_double());

    if (this->get_parameter("n_obstacles").as_int() == 0) {
      RCLCPP_INFO(this->get_logger(), "Number of requested obstacles is 0, exiting");
      exit(0);
    }

    if (this->get_parameter("no_cylinders").as_bool() && this->get_parameter("no_boxes").as_bool()) {
      RCLCPP_ERROR(this->get_logger(), "Both no_cylinders and no_boxes are true. At least one of them must be false.");
      exit(1);
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    this->publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/obstacles", qos);

    std::vector<obstacle> obstacles;

    std::string map = this->get_parameter("map").as_string();
    if (map != "hexagon" && map != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
      exit(1);
    }
    double dx = this->get_parameter("dx").as_double();
    double dy = this->get_parameter("dy").as_double();
    int max_timeout = this->get_parameter("max_timeout").as_int();
    int n_obstacles = this->get_parameter("n_obstacles").as_int();
    
    // Define a random number generator for doubles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> size_dis(
      this->get_parameter("min_size").as_double(), 
      this->get_parameter("max_size").as_double()
    ); 
    std::uniform_real_distribution<> x_dis(-dx, dx);
    std::uniform_real_distribution<> y_dis(-dy, dy);
    std::uniform_int_distribution<> shape(0, 2);

    auto startTime = this->get_clock()->now();
    for (int i=0; i<n_obstacles && !overTime(this->get_clock(), startTime, max_timeout); i++) {
      obstacle obs {0.0, 0.0, 0.0, 0.0, 0.0, obstacle_type::CYLINDER};

      if (this->get_parameter("no_cylinders").as_bool()) {
        rand_box(obs, obstacles, map, dx, dy, startTime, max_timeout, gen);
      } 
      else if (this->get_parameter("no_boxes").as_bool()) {
        rand_cylinder(obs, obstacles, map, dx, dy, startTime, max_timeout, gen);
      } 
      else {
        obstacle_type type = shape(gen) == 0 ? obstacle_type::CYLINDER : obstacle_type::BOX;
        RCLCPP_INFO(this->get_logger(), "Obstacle type: %s", (type==obstacle_type::BOX ? "box" : "cylinder"));
        if (type == obstacle_type::BOX) {
          rand_box(obs, obstacles, map, dx, dy, startTime, max_timeout, gen);
        }
        else {
          rand_box(obs, obstacles, map, dx, dy, startTime, max_timeout, gen);
        }
      }
      RCLCPP_INFO(this->get_logger(), "Obstacle %d: x=%f, y=%f, radius=%f, dx=%f, dy=%f", 
        i, obs.x, obs.y, obs.radius, obs.dx, obs.dy);
    }

    if (overTime(this->get_clock(), startTime, max_timeout)) {
      RCLCPP_INFO(this->get_logger(), "Could not find a valid position for some obstacles [%ld/%d]", obstacles.size(), n_obstacles); 
    }

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

      std::string xml_string;
      if (o.type == obstacle_type::BOX) {
        // Read XML file to string
        std::ifstream xml_file(this->share_dir + "/models/box/model.sdf");
        xml_string.assign(
          std::istreambuf_iterator<char>(xml_file),
          std::istreambuf_iterator<char>()
        );
        std::string size_string = "<size>1 1 1</size>";
        std::string size_replace_string = "<size>" + std::to_string(o.dx) + " " + std::to_string(o.dy) + " 1</size>";
        size_t pos = 0;
        while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
          xml_string.replace(pos, size_string.length(), size_replace_string);
          pos += size_replace_string.length();
        }
      }
      else { 
        // Read XML file to string
        std::ifstream xml_file(this->share_dir + "/models/cylinder/model.sdf");
        xml_string.assign(
          std::istreambuf_iterator<char>(xml_file),
          std::istreambuf_iterator<char>()
        );
        std::string size_string = "<radius>0.5</radius>";
        std::string size_replace_string = "<radius>" + std::to_string(o.radius) + "</radius>";
        size_t pos = 0;
        while ((pos = xml_string.find(size_string, pos)) != std::string::npos) {
          xml_string.replace(pos, size_string.length(), size_replace_string);
          pos += size_replace_string.length();
        }
      }

      // Spawn model in gazebo
      geometry_msgs::msg::Pose pose;
      pose.position.x = o.x;
      pose.position.y = o.y;
      pose.position.z = 0.1;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 0;

      spawn_model(this->get_node_base_interface(), this->spawner_, xml_string, pose);

      sleep(0.5);
    }

    this->publisher_->publish(msg);
  }
private:
  void rand_cylinder(obstacle& obs, std::vector<obstacle>& obstacles, std::string map, double dx, double dy, rclcpp::Time& startTime, int max_timeout, std::mt19937& gen);
  void rand_box(obstacle& obs, std::vector<obstacle>& obstacles, std::string map, double dx, double dy, rclcpp::Time& startTime, int max_timeout, std::mt19937& gen);
};


void ObstaclesPublisher::rand_cylinder(obstacle& obs, std::vector<obstacle>& obstacles, std::string map, double dx, double dy, rclcpp::Time& startTime, int max_timeout, std::mt19937& gen){
  std::uniform_real_distribution<> size_dis(
    this->get_parameter("min_size").as_double(), 
    this->get_parameter("max_size").as_double()
  );
  std::uniform_real_distribution<> x_dis(-dx, dx);
  std::uniform_real_distribution<> y_dis(-dy, dy);

  obs.type = obstacle_type::CYLINDER;
  do {
    obs.radius = size_dis(gen);
    obs.x = x_dis(gen);
    obs.y = y_dis(gen);
    if (!overlaps(obs, obstacles) && is_inside_map(obs, map, dx, dy)){
      obstacles.push_back(obs);
      break;
    }
  } while(!overTime(this->get_clock(), startTime, max_timeout));
}

void ObstaclesPublisher::rand_box(obstacle& obs, std::vector<obstacle>& obstacles, std::string map, double dx, double dy, rclcpp::Time& startTime, int max_timeout, std::mt19937& gen){
  std::uniform_real_distribution<> size_dis(
    this->get_parameter("min_size").as_double(), 
    this->get_parameter("max_size").as_double()
  );
  std::uniform_real_distribution<> x_dis(-dx, dx);
  std::uniform_real_distribution<> y_dis(-dy, dy);

  obs.type = obstacle_type::BOX;
  do {    
    obs.dx = size_dis(gen);
    obs.dy = size_dis(gen);
    obs.x = x_dis(gen);
    obs.y = y_dis(gen);
    
    if (!overlaps(obs, obstacles) && is_inside_map(obs, map, dx, dy)){
      obstacles.push_back(obs);
      break;
    }
  } while(!overTime(this->get_clock(), startTime, max_timeout));
  RCLCPP_INFO(this->get_logger(), "Obstacle: x=%f, y=%f, dx=%f, dy=%f, intersections: %d, time: %f, max_time: %d, inside_map: %s", 
    obs.x, obs.y, obs.dx, obs.dy, (bool)overlaps(obs, obstacles), (this->get_clock()->now().seconds()-startTime.seconds()), max_timeout, (is_inside_map(obs, map, dx, dy) ? std::string("true").c_str() : std::string("false").c_str()));
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstaclesPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

