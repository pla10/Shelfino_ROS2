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

#include "gazebo_msgs/srv/spawn_entity.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "map_pkg/utilities.hpp"
#include "map_pkg/spawn_model.hpp"


class ObstaclesPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr borders_tran_sub_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr gates_tran_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_sub_;
  std::string share_dir;
  bool borders_ready = false, gates_ready = false;

  // Data obtained from parameters
  struct Data {
    std::string map_name;
    double dx;
    double dy;
    uint max_timeout;
    uint n_obstacles;
    bool no_cylinders;
    bool no_boxes;
    double min_size;
    double max_size;
  } data;

  std::vector<obstacle> gates;

public:
  explicit ObstaclesPublisher(bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode("send_obstacles", 
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
    this->share_dir = ament_index_cpp::get_package_share_directory("map_pkg");
    this->configure();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state)
  {
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

    this->data.map_name = this->get_parameter("map").as_string();  // hexagon, rectangle
    this->data.dx = this->get_parameter("dx").as_double();
    this->data.dy = this->get_parameter("dy").as_double();
    this->data.max_timeout = this->get_parameter("max_timeout").as_int();
    this->data.n_obstacles = this->get_parameter("n_obstacles").as_int();
    this->data.no_cylinders = this->get_parameter("no_cylinders").as_bool();
    this->data.no_boxes = this->get_parameter("no_boxes").as_bool();
    this->data.min_size = this->get_parameter("min_size").as_double();
    this->data.max_size = this->get_parameter("max_size").as_double();

    if (this->data.no_cylinders && this->data.no_boxes) {
      RCLCPP_ERROR(this->get_logger(), "Both no_cylinders and no_boxes are true. At least one of them must be false.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    if (this->data.map_name != "hexagon" && this->data.map_name != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    this->publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/obstacles", qos);

    this->borders_tran_sub_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/send_borders/transition_event", 10,
      std::bind(&ObstaclesPublisher::on_borders_transition_event, this, std::placeholders::_1)
    );
    
    this->gates_tran_sub_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/send_gates/transition_event", 10,
      std::bind(&ObstaclesPublisher::on_gates_transition_event, this, std::placeholders::_1)
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  { 
    RCLCPP_INFO(this->get_logger(), "Activating node");

    LifecycleNode::on_activate(state);
    std::vector<obstacle> obstacles = this->rand_obstacles();
    this->spawn_and_publish_obstacles(obstacles);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; 
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node");
    LifecycleNode::on_deactivate(state);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  /**
   * @brief This function listens and saves the gates positions
   * 
   * @param msg The message containing the gates positions
   */
  void gate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    for (auto pose : msg->poses) {
      obstacle obs {0.0, pose.position.x, pose.position.y, 1.0, 1.0, obstacle_type::BOX};
      this->gates.push_back(obs);
    }
    this->borders_ready = true;
    this->activate_wrapper();
  }

  void on_borders_transition_event(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received borders transition event: %s from %s", 
      msg->goal_state.label.c_str(), msg->start_state.label.c_str());
    if (msg->goal_state.label == "deactivating" && msg->start_state.label == "active") {
      RCLCPP_INFO(this->get_logger(), "Activating node");
      this->borders_tran_sub_.reset();

      this->gate_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gate_position", this->qos,
        std::bind(&ObstaclesPublisher::gate_callback, this, std::placeholders::_1)
      );

      this->activate_wrapper();
    }
  }

  void on_gates_transition_event(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received gates transition event: %s from %s", 
      msg->goal_state.label.c_str(), msg->start_state.label.c_str());
    if (msg->goal_state.label == "deactivating" && msg->start_state.label == "active") {
      RCLCPP_INFO(this->get_logger(), "Activating node");
      this->gates_tran_sub_.reset();

      this->gates_ready = true;

      this->activate_wrapper();
    }
  }

  void activate_wrapper()
  {
    if (this->borders_ready && this->gates_ready) {
      this->activate();
    }
    else {
      if (!this->borders_ready)
        RCLCPP_INFO(this->get_logger(), "Waiting for borders to be ready");
      if (!this->gates_ready)
        RCLCPP_INFO(this->get_logger(), "Waiting for gates to be ready");
    }

    this->deactivate();
  }

  std::vector<obstacle> rand_obstacles();
  void rand_cylinder(obstacle& obs, std::vector<obstacle>& obstacles, rclcpp::Time& startTime, std::mt19937& gen);
  void rand_box(obstacle& obs, std::vector<obstacle>& obstacles, rclcpp::Time& startTime, std::mt19937& gen);
  void spawn_and_publish_obstacles(std::vector<obstacle>& obstacles);
};


std::vector<obstacle> ObstaclesPublisher::rand_obstacles()
{
  std::vector<obstacle> obstacles;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> size_dis(
    this->data.min_size, 
    this->data.max_size
  ); 
  std::uniform_real_distribution<> x_dis(-this->data.dx, this->data.dx);
  std::uniform_real_distribution<> y_dis(-this->data.dy, this->data.dy);
  std::uniform_int_distribution<> shape(0, 1);

  auto startTime = this->get_clock()->now();
  for (
    uint i=0; 
    i<this->data.n_obstacles && !overTime(this->get_clock(), startTime, this->data.max_timeout); 
    i++) 
  {
    obstacle obs {0.0, 0.0, 0.0, 0.0, 0.0, obstacle_type::CYLINDER};

    if (this->data.no_cylinders) {
      rand_box(obs, obstacles, startTime, gen);
    } 
    else if (this->data.no_boxes) {
      rand_cylinder(obs, obstacles, startTime, gen);
    } 
    else {
      obstacle_type type = shape(gen) == 0 ? obstacle_type::CYLINDER : obstacle_type::BOX;
      RCLCPP_INFO(this->get_logger(), "Obstacle type: %s", (type==obstacle_type::BOX ? "box" : "cylinder"));
      if (type == obstacle_type::BOX) {
        rand_box(obs, obstacles, startTime, gen);
      }
      else {
        RCLCPP_INFO(this->get_logger(), "spawning cylinder");
        rand_cylinder(obs, obstacles, startTime, gen);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Obstacle %d: x=%f, y=%f, radius=%f, dx=%f, dy=%f", 
      i, obs.x, obs.y, obs.radius, obs.dx, obs.dy);
  }

  if (overTime(this->get_clock(), startTime, this->data.max_timeout)) {
    // This is not an unrecoverable error, so we don't return ERROR
    RCLCPP_ERROR(this->get_logger(), "Could not find a valid position for some obstacles [%ld/%d]", obstacles.size(), this->data.n_obstacles); 
  }

  return obstacles;
}

void ObstaclesPublisher::spawn_and_publish_obstacles(std::vector<obstacle>& obstacles)
{
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

    // sleep(0.5);
  }

  this->publisher_->publish(msg);
}

void ObstaclesPublisher::rand_cylinder(obstacle& obs, std::vector<obstacle>& obstacles, rclcpp::Time& startTime, std::mt19937& gen){
  std::uniform_real_distribution<> size_dis(
    this->data.min_size, 
    this->data.max_size
  );
  std::uniform_real_distribution<> x_dis(-this->data.dx, this->data.dx);
  std::uniform_real_distribution<> y_dis(-this->data.dy, this->data.dy);

  obs.type = obstacle_type::CYLINDER;
  do {
    obs.radius = size_dis(gen);
    obs.x = x_dis(gen);
    obs.y = y_dis(gen);
    if (valid_position(this->data.map_name, this->data.dx, this->data.dy, obs, obstacles, this->gates)){
      obstacles.push_back(obs);
      break;
    }
  } while(!overTime(this->get_clock(), startTime, this->data.max_timeout));
}

void ObstaclesPublisher::rand_box(obstacle& obs, std::vector<obstacle>& obstacles, rclcpp::Time& startTime, std::mt19937& gen){
  std::uniform_real_distribution<> size_dis(
    this->data.min_size, 
    this->data.max_size
  );
  std::uniform_real_distribution<> x_dis(-this->data.dx, this->data.dx);
  std::uniform_real_distribution<> y_dis(-this->data.dy, this->data.dy);

  obs.type = obstacle_type::BOX;
  do {    
    obs.dx = size_dis(gen);
    obs.dy = size_dis(gen);
    obs.x = x_dis(gen);
    obs.y = y_dis(gen);
    
    if (valid_position(this->data.map_name, this->data.dx, this->data.dy, obs, obstacles, this->gates)){
      obstacles.push_back(obs);
      break;
    }
  } while(!overTime(this->get_clock(), startTime, this->data.max_timeout));
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstaclesPublisher>();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  
  rclcpp::shutdown();
  return 0;
}

