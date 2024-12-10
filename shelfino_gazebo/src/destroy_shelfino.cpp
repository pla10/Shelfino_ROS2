#include "rclcpp/rclcpp.hpp"

#include "gazebo_msgs/srv/delete_entity.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


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


class ShelfinoDestroyer : public rclcpp::Node
{
private:
  geometry_msgs::msg::PoseArray gates_pose;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gates_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_pose_;

public:
  bool deleted = false;

  explicit ShelfinoDestroyer() : Node("destroy_shelfino")
  {  
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node initialized with namespace %s", this->get_namespace());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    // Create subscription to /gates 
    this->sub_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/gates", qos, std::bind(&ShelfinoDestroyer::handle_gates_pose, this, std::placeholders::_1));
  }

  void handle_gates_pose(const geometry_msgs::msg::PoseArray::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received gates in %f %f", msg->poses[0].position.x, msg->poses[0].position.y);
    this->gates_pose.poses = msg->poses;
    this->create_amcl_sub();
  }

  void delete_entity(){
    auto client = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    // Check if service is available
    while(!client->wait_for_service(std::chrono::milliseconds(100))){
      if (!rclcpp::ok()){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown(nullptr, "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not yet available, waiting again...");
    }
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = this->get_namespace();
    request->name.erase(0, 1);
    RCLCPP_INFO(this->get_logger(), "Sending request to delete %s", request->name.c_str());
    auto result = client->async_send_request(request);

    deleted = true;
  }

  void listen_to_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received amcl pose: x: %f, y: %f", 
      msg->pose.pose.position.x, msg->pose.pose.position.y
    );
    for (auto gate_pose : this->gates_pose.poses){
      if (std::abs(msg->pose.pose.position.x - gate_pose.position.x) < 0.5 && 
          std::abs(msg->pose.pose.position.y - gate_pose.position.y) < 0.5)
      {
        delete_entity();
      }
    }
  }


  void create_amcl_sub(){
    // Create subscription to amcl_pose
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for pose...");
    this->sub_amcl_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", 10, std::bind(&ShelfinoDestroyer::listen_to_pose, this, std::placeholders::_1)
    );
  }
};




int main (int argc, char** argv){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ShelfinoDestroyer>();

  while (!node->deleted){
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return 0;
}