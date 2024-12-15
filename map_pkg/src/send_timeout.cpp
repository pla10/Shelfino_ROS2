#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/timer.hpp"

#include "map_pkg/utilities.hpp"

using MsgType = std_msgs::msg::Int32;

/**
 * @brief Class to publish the timeout of the victims in seconds.
 */
class SendTimeout : public rclcpp_lifecycle::LifecycleNode
{
private:
  int32_t timeout_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
  rclcpp::Publisher<MsgType>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  explicit SendTimeout (bool intra_process_comms = false) 
  : rclcpp_lifecycle::LifecycleNode("send_timeout", 
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state)
  {
    this->declare_parameter("victims_timeout", 0);
    this->timeout_ = get_parameter("victims_timeout").as_int();

    // if (this->timeout_ == 0){
    //   this->find_timeout();
    // }

    this->publisher_ = this->create_publisher<MsgType>("/victims_timeout", qos_);
    RCLCPP_INFO(this->get_logger(), "send_timeout configured with timeout %d", this->timeout_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating node %s.", this->get_name());
    this->publish_timeout();
    RCLCPP_INFO(this->get_logger(), "Timeout published, node activated.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node %s.", this->get_name());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up node %s.", this->get_name());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down node %s.", this->get_name());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void publish_timeout()
  {
    MsgType msg;
    msg.data = this->timeout_;
    this->publisher_->publish(msg);
  }

  void find_timeout()
  {
    this->timeout_ = 10;
  }

};


int main (int argc, char * argv[]){
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<SendTimeout>();
  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}

