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
#include "lifecycle_msgs/srv/get_state.hpp"

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "map_pkg/spawn_model.hpp"
#include "map_pkg/utilities.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using CallbackGroup = rclcpp::CallbackGroup;

using namespace geometry_msgs::msg;

// TODO it would be nice if we could send the initial position, but it either this node is launched too early, or it does not work

class InitPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  bool spawned = false;

  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

  /// Robot state publishers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rsp_sub_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr amcl_tran_sub_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawner_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr publisher_;

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr amcl_get_state_;
  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  PoseWithCovariance pose_c_msg;
  Pose actual_pose;

  std::string rsp_xml = {};
  bool amcl_ready = false;

  struct Data {
    std::string map; 
    double dx;
    double dy;
    double x;
    double y;
    double yaw;
    bool random;
  } data;

public:
  explicit InitPublisher(bool intra_process_comms = false) 
      : rclcpp_lifecycle::LifecycleNode(
          "send_initialpose", 
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
  ){}

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state){
    RCLCPP_INFO(this->get_logger(), "Configuring send_initial pose");

    RCLCPP_INFO(this->get_logger(), "Declaring parameters");
    // Map parameters
    this->declare_parameter("map", "hexagon");
    this->declare_parameter("dx",  10.0);
    this->declare_parameter("dy",  10.0);

    // Init parameters
    this->declare_parameter("init_x",    0.0);
    this->declare_parameter("init_y",    0.0);
    this->declare_parameter("init_yaw",  0.0);
    this->declare_parameter("init_rand", false);

    RCLCPP_INFO(this->get_logger(), "Getting parameters");
    // Get parameters
    this->data.map    = this->get_parameter("map").as_string();
    this->data.dx     = this->get_parameter("dx").as_double();
    this->data.dy     = this->get_parameter("dy").as_double();
    this->data.x      = this->get_parameter("init_x").as_double();
    this->data.y      = this->get_parameter("init_y").as_double();
    this->data.yaw    = this->get_parameter("init_yaw").as_double();
    this->data.random = this->get_parameter("init_rand").as_bool();

    // Check map has the correct value
    RCLCPP_INFO(this->get_logger(), "Checking parameters");
    if (this->data.map != "hexagon" && this->data.map != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle and not %s.", this->data.map.c_str());
      exit(1);
    }


    RCLCPP_INFO(this->get_logger(), "Printing parameters");
    // Print parameters
    RCLCPP_INFO(this->get_logger(), "map: %s, dx: %f, dy: %f", this->data.map.c_str(), this->data.dx, this->data.dy);
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, yaw: %f, random: %d", this->data.x, this->data.y, this->data.yaw, this->data.random);

    // Setup subscriptions
    // Robot state publisher subscriptions
    this->rsp_sub_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description", qos, 
      std::bind(&InitPublisher::rsp_callback, this, std::placeholders::_1)
    );

    this->amcl_tran_sub_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "amcl/transition_event", rclcpp::QoS(rclcpp::KeepLast(1)), 
      std::bind(&InitPublisher::amcl_tran_callback, this, std::placeholders::_1)
    );

    this->publisher_ = this->create_publisher<PoseWithCovarianceStamped>("initialpose", this->qos);

    this->amcl_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>("amcl/get_state");
    
    // // Setup tf2
    // this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    // Setup spawner
    this->spawner_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating send_initial pose");
    return CallbackReturn::SUCCESS;
  }

private:
  void rsp_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    this->rsp_xml = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received robot description %s", this->rsp_xml.c_str());
    this->spawn_and_publish();

    RCLCPP_INFO(this->get_logger(), "Checking if amcl is active %d", this->amcl_ready);

    if (!this->amcl_ready){
      // Test if amcl is active by sending a request to /shelfino1/amcl/get_state
      auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto result = this->amcl_get_state_->async_send_request(request);

      using ServiceResponseFuture =
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture;
      auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        if (result->current_state.label == "active"){
          RCLCPP_INFO(this->get_logger(), "Request answered AMCL active. Publishing initial pose");
          this->spawn_and_publish();
        }
        else{
          RCLCPP_INFO(this->get_logger(), "Request answered AMCL not active. Waiting.");
        }
      };
      RCLCPP_INFO(this->get_logger(), "Sending request to /shelfino1/amcl/get_state");
      auto future_result = this->amcl_get_state_->async_send_request(request, response_received_callback);  
    }
    else{
      RCLCPP_INFO(this->get_logger(), "AMCL active. Publishing initial pose");
      this->publish();
    }
  }
  
  void amcl_tran_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
    if (this->spawned && msg->goal_state.label == "active"){
      RCLCPP_INFO(this->get_logger(), "AMCL active");
      this->amcl_ready = true;
      if (!this->rsp_xml.empty()){
        RCLCPP_INFO(this->get_logger(), "AMCL active. Publishing initial pose");
        this->publish();
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Robot description not received. Waiting.");
      }
    }
  }
  
  void spawn_and_publish();
  Pose rand_hexagon();
  Pose rand_rectangle();
  void publish();  
};

void InitPublisher::publish(){
  // TransformStamped transformStamped;
  // bool received = false, spam = false;
  
  // std::string fromFrame = (std::string(this->get_namespace())+"/base_link").erase(0, 1);;

  // try {
  //   transformStamped = this->tf_buffer_->lookupTransform("map", fromFrame, rclcpp::Time(0));
  //   received = true;
  // } catch (tf2::TransformException &ex) {
  //   RCLCPP_ERROR(this->get_logger(), "Could not find transform between map and %s [%s]", fromFrame.c_str(), ex.what());
    
  //   if (std::string(ex.what()).find("passed to lookupTransform argument target_frame") != std::string::npos){
  //     spam = true;
  //   }
  // }

  // if (received && 
  //     std::abs(transformStamped.transform.translation.x - this->pose_c_msg.pose.position.x) < 0.0001 && 
  //     std::abs(transformStamped.transform.translation.y - this->pose_c_msg.pose.position.y) < 0.0001 &&
  //     std::abs(transformStamped.transform.rotation.x - this->pose_c_msg.pose.orientation.x) < 0.0001 &&
  //     std::abs(transformStamped.transform.rotation.y - this->pose_c_msg.pose.orientation.y) < 0.0001 &&
  //     std::abs(transformStamped.transform.rotation.z - this->pose_c_msg.pose.orientation.z) < 0.0001 &&
  //     std::abs(transformStamped.transform.rotation.w - this->pose_c_msg.pose.orientation.w) < 0.0001)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Done msg");
  //   this->timer_->cancel();
  // } 
  // else if (received || spam)
  // {
    RCLCPP_INFO(this->get_logger(), "Publishing initial pose"); //% %d spam %d", this->count_, spam);
    PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();
    msg.pose = this->pose_c_msg;
    this->publisher_->publish(msg);
    // this->count_++;
  // }
}

void InitPublisher::spawn_and_publish(){
  // this->timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(10), std::bind(&InitPublisher::publish, this)
  // );
  if (!this->spawned){
    // RCLCPP_INFO(this->get_logger(), "Sending intialpose");
    // this->publish();

    RCLCPP_INFO(this->get_logger(), "Spawning entity");
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = std::string(this->get_namespace()).erase(0, 1)+"_from_initialpose";
    request->initial_pose = this->pose_c_msg.pose;
    request->xml = this->rsp_xml;

    while(!this->spawner_->wait_for_service(std::chrono::seconds(1))){
      if (!rclcpp::ok()){
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        exit(1);
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    using ServiceResponseFuture =
        rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      if (result->success){
        RCLCPP_INFO(this->get_logger(), "Model spawned");
        this->spawned = true;
      }
      else{
        RCLCPP_INFO(this->get_logger(), "Model not spawned. Waiting.");
      }
    };

    auto result = this->spawner_->async_send_request(request, response_received_callback);

  }
  else {
    RCLCPP_INFO(this->get_logger(), "Model already spawned");
  }
}








// ////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create the node 
  
  auto node = std::make_shared<InitPublisher>();
  node->configure();
  RCLCPP_INFO(node->get_logger(), "Configured");
  node->activate();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  
  return 0;
}



// Pose InitPublisher::rand_hexagon(){
//   // Declare random generator
//   std::random_device rd;
//   std::mt19937 gen(rd());
//   std::uniform_real_distribution<> dis_x(-this->data.dx, this->data.dx);
//   std::uniform_real_distribution<> dis_yaw(-M_PI, M_PI);


//   double yaw = dis_yaw(gen);
//   obstacle obs {0.0, 0.0, 0.0, 1.0, 1.0, obstacle_type::BOX};

//   do {
//     obs.x = dis_x(gen);
//     obs.y = dis_x(gen);
    
//   } while(!is_inside_map(obs, "hexagon", this->data.dx-0.5, this->data.dy-0.5));
//   RCLCPP_INFO(this->get_logger(), "Starting in x: %f, y: %f", obs.x, obs.y);
  
//   Pose pose;
//   pose.position.x = obs.x;
//   pose.position.y = obs.y;
//   pose.position.z = 0.0;

//   tf2::Quaternion q;
//   q.setRPY(0, 0, yaw);
//   pose.orientation.x = q.x();
//   pose.orientation.y = q.y();
//   pose.orientation.z = q.z();
//   pose.orientation.w = q.w();

//   return pose;
// }


// Pose InitPublisher::rand_rectangle(){
//   // Declare random generator
//   std::random_device rd;
//   std::mt19937 gen(rd());
//   std::uniform_real_distribution<> dis_x(-this->data.dx, this->data.dx);
//   std::uniform_real_distribution<> dis_y(-this->data.dy, this->data.dy);
//   std::uniform_real_distribution<> dis_yaw(-M_PI, M_PI);

//   double yaw = dis_yaw(gen);
//   obstacle obs {0.0, 0.0, 0., 1.0, 1.0, obstacle_type::BOX};

//   do {
//     obs.x = dis_x(gen);
//     obs.y = dis_y(gen);
//   } while(!is_inside_map(obs, "rectangle", this->data.dx-0.5, this->data.dy-0.5));

//   RCLCPP_INFO(this->get_logger(), "Starting in x: %f, y: %f", obs.x, obs.y);

//   Pose pose;
//   pose.position.x = obs.x;
//   pose.position.y = obs.y;
//   pose.position.z = 0.0;

//   tf2::Quaternion q;
//   q.setRPY(0, 0, yaw);
//   pose.orientation.x = q.x();
//   pose.orientation.y = q.y();
//   pose.orientation.z = q.z();
//   pose.orientation.w = q.w();
  
//   return pose;
// }

