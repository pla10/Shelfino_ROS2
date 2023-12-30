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
  geometry_msgs::msg::Pose gate_pose;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gates_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_pose_;

public:
  bool deleted = false;

  explicit ShelfinoDestroyer() : Node("destroy_shelfino")
  {  
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node initialized with namespace %s", this->get_namespace());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    // Create subscription to /gate_position 
    this->sub_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/gate_position", qos, 
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received gates in %f %f", msg->poses[0].position.x, msg->poses[0].position.y);
        this->gate_pose = msg->poses[0];
        this->listen_to_pose();
      }
    );
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

  void listen_to_pose(){
    // Create subscription to amcl_pose
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for pose...");
    this->sub_amcl_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", 10, 
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received pose: x: %f, y: %f", 
          msg->pose.pose.position.x, msg->pose.pose.position.y
        );
        if (std::abs(msg->pose.pose.position.x - this->gate_pose.position.x) < 0.5 && 
            std::abs(msg->pose.pose.position.y - this->gate_pose.position.y) < 0.5)
        {
          delete_entity();
        }
        else {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for pose...");
        }
      }
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