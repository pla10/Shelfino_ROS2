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

class VictimPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr obstacles_tran_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_sub_;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_sub_;
  std::string share_dir;

  struct Data {
    std::string map_name;
    double dx;
    double dy;
    int max_timeout;
    int n_victims;
    int min_weight;
    int max_weight;
  } data ;

  std::vector<obstacle> gates;
  std::vector<obstacle> obstacles;

  bool gates_ready = false;
  bool obstacles_ready = false;

public:
  explicit VictimPublisher(bool intra_process_comms = false) 
  : rclcpp_lifecycle::LifecycleNode("obstacles_sender",
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

    // Victims parameters
    this->declare_parameter("n_victims", 3);
    this->declare_parameter("min_weight", 10);
    this->declare_parameter("max_weight", 500);

    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // Get parameters
    this->data.map_name = this->get_parameter("map").as_string();
    if (this->data.map_name != "hexagon" && this->data.map_name != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
      exit(1);
    }
    this->data.dx = this->get_parameter("dx").as_double();
    this->data.dy = this->get_parameter("dy").as_double();
    this->data.max_timeout = this->get_parameter("max_timeout").as_int();
    this->data.n_victims = this->get_parameter("n_victims").as_int();
    this->data.min_weight = this->get_parameter("min_weight").as_int();
    this->data.max_weight = this->get_parameter("max_weight").as_int();

    // Print parameters values
    RCLCPP_INFO(this->get_logger(), "map: %s", this->data.map_name.c_str());
    RCLCPP_INFO(this->get_logger(), "dx: %f", this->data.dx);
    RCLCPP_INFO(this->get_logger(), "dy: %f", this->data.dy);
    RCLCPP_INFO(this->get_logger(), "max_timeout: %d", this->data.max_timeout);
    RCLCPP_INFO(this->get_logger(), "n_victims: %d", this->data.n_victims);
    RCLCPP_INFO(this->get_logger(), "min_weight: %d", this->data.min_weight);
    RCLCPP_INFO(this->get_logger(), "max_weight: %d", this->data.max_weight);

    if (this->data.n_victims == 0) {
      RCLCPP_INFO(this->get_logger(), "No victims to generate.");
      exit(0);
    }

    this->publisher_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/victims", this->qos);
    this->obstacles_tran_sub_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/send_obstacles/transition_event", 10, 
      std::bind(&VictimPublisher::on_obstacles_transition_event, this, std::placeholders::_1)
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void on_obstacles_transition_event(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received transition event: %s from %s", msg->goal_state.label.c_str(), msg->start_state.label.c_str());
    if (msg->start_state.label == "active" && msg->goal_state.label == "deactivating") {
      this->obstacles_tran_sub_.reset();
      
      this->gate_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gate_position", this->qos, 
        std::bind(&VictimPublisher::gates_callback, this, std::placeholders::_1)
      );
      this->obstacles_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", this->qos, 
        std::bind(&VictimPublisher::obstacles_callback, this, std::placeholders::_1)
      );
      this->activate_wrapper();
    }
  }

  void gates_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    this->gates.clear();
    for (auto pose : msg->poses) {
      obstacle gate = {0.0, pose.position.x, pose.position.y, 1.0, 1.0, obstacle_type::BOX};
      this->gates.push_back(gate);
    }
    this->gates_ready = true;
    this->activate_wrapper();
  }

  void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    this->obstacles.clear();
    for (auto obs : msg->obstacles) {
      if (obs.polygon.points.size() == 1) {
        obstacle obs_tmp = {obs.radius, obs.polygon.points[0].x, obs.polygon.points[0].y, 0.0, 0.0, obstacle_type::CYLINDER};
        this->obstacles.push_back(obs_tmp);
      }
      else if (obs.polygon.points.size() == 4) {
        double max_x = 0.0, max_y = 0.0, min_x = 1000000.0, min_y = 1000000.0;
        for (auto point : obs.polygon.points) {
          if (point.x > max_x) max_x = point.x;
          if (point.x < min_x) min_x = point.x;
          if (point.y > max_y) max_y = point.y;
          if (point.y < min_y) min_y = point.y;
        }
        double xc = (max_x+min_x)/2.0;
        double yc = (max_y+min_y)/2.0;
        double dx = max_x-min_x;
        double dy = max_y-min_y;

        obstacle obs_tmp = {0.0, xc, yc, dx, dy, obstacle_type::BOX};
        this->obstacles.push_back(obs_tmp);
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Obstacle with %ld points not supported.", obs.polygon.points.size());
      }
    }
    this->obstacles_ready = true;
    this->activate_wrapper();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    std::vector<obstacle> victims = this->rand_victims();
    if (victims.size() < this->data.n_victims) {
      RCLCPP_ERROR(this->get_logger(), "Could not generate %d victims.", this->data.n_victims);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    this->spawn_and_publish_victims(victims);
    
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
  std::vector<obstacle> rand_victims();
  void spawn_and_publish_victims(const std::vector<obstacle>& victims);

  void activate_wrapper()
  {
    if (this->gates_ready && this->obstacles_ready) {
      this->activate();
    }
    else {
      if (!this->gates_ready)
        RCLCPP_INFO(this->get_logger(), "Waiting for gates to be ready");
      if (!this->obstacles_ready)
        RCLCPP_INFO(this->get_logger(), "Waiting for obstacles to be ready");
    }

    this->deactivate();
  }

};


std::vector<obstacle> VictimPublisher::rand_victims()
{
  // Define a random number generator for doubles
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> x_dis(-this->data.dx, this->data.dx);
  std::uniform_real_distribution<> y_dis(-this->data.dy, this->data.dy);

  std::vector<obstacle> victims;
  auto startTime = this->get_clock()->now();
  for (uint i=0; i<this->data.n_victims && !overTime(this->get_clock(), startTime, this->data.max_timeout); i++) {
    victim vict = victim(0.0, 0.0);
    do {
      vict.x = x_dis(gen);
      vict.y = y_dis(gen);
      if (valid_position(this->data.map_name, this->data.dx, this->data.dy, vict, {victims, this->obstacles, this->gates})) {
        victims.push_back(vict);
        break;
      }
    } while(!overTime(this->get_clock(), startTime, this->data.max_timeout));
  }

  if (overTime(this->get_clock(), startTime, this->data.max_timeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while trying to generate victims.");
  }

  return victims; 
}

void VictimPublisher::spawn_and_publish_victims(const std::vector<obstacle>& victims) 
{
  obstacles_msgs::msg::ObstacleArrayMsg msg;
  std_msgs::msg::Header hh;
  hh.stamp = this->get_clock()->now();
  hh.frame_id = "map";
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> weight_dis(
    this->data.min_weight, 
    this->data.max_weight
  );
  
  for (auto vict : victims) {
    RCLCPP_INFO(this->get_logger(), "Publishing victim at x=%f, y=%f", vict.x, vict.y);

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
    obs.radius = weight_dis(gen);

    msg.obstacles.push_back(obs);

    // Read XML file to string
    std::string xml_string;
    std::ifstream xml_file(this->share_dir + "/models/victim/model.sdf");
    xml_string.assign(
      std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>()
    );

    // Spawn model in gazebo
    geometry_msgs::msg::Pose pose;
    pose.position.x = vict.x;
    pose.position.y = vict.y;
    pose.position.z = 0.01;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
  
    spawn_model(this->get_node_base_interface(), this->spawner_, xml_string, pose, "victim");
  }

  this->publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VictimPublisher>();
  
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}