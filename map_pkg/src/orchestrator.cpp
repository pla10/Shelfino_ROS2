#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "map_pkg/utilities.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using lifecycle_msgs::msg::TransitionEvent;
using lifecycle_msgs::srv::ChangeState;
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief This node is responsible for orchestrating the state transitions of the nodes
 * @details The orchestrator is a managed node.
 * - At the beginning (constructor), it will
 *   - read the parameter 'victims_activated' to determine if the send_victims node is
 *     enabled;
 *   - create the subscriptions to the state_transition events of the send_borders,
 *     send_gates, send_obstacles, and send_victims nodes;
 *   - create the clients to send the state_transition events to the send_borders,
 *     send_gates, send_obstacles, and send_victims nodes.
 * - When it is configuring, it will send the configure message to the send_borders,
 *   send_gates, send_obstacles, and send_victims nodes (if activated), and it will listen
 *   for changes in states of these nodes. Each time one of the nodes to which it is
 *   subscribed changes state, it will check if the new state of that node and make
 *   decisions.
 * - Once configured, it will move to activate and just listen for changes in state.
 * - Once all nodes are active, it will terminate, i.e., move to shutdown.
 * The order in which it runs the nodes is as follows:
 * 1. send_borders
 * 2. send_gates
 * 3. send_obstacles
 * 4. send_victims (if enabled)
 * Each time a change in state is received, the orchestrator it will check the state of
 * node sending the change to decide what transition it should do, but it will also check
 * the previous node and the following node to make sure that no transition is lost. It
 * may happen that the previous node reached the active phase before the current one is
 * configured, hence it would lose a transition.
 *
 * The state of the send_* nodes is stored in the variables borders_state_, gates_state_,
 * obstacles_state_, and victims_state_.
 */
class Orchestrator : public rclcpp_lifecycle::LifecycleNode {
 private:
  rclcpp::Subscription<TransitionEvent>::SharedPtr borders_sub_;
  rclcpp::Subscription<TransitionEvent>::SharedPtr gates_sub_;
  rclcpp::Subscription<TransitionEvent>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<TransitionEvent>::SharedPtr victims_sub_;
  rclcpp::Subscription<TransitionEvent>::SharedPtr init_pos_sub_;

  rclcpp::Client<ChangeState>::SharedPtr border_pub_;
  rclcpp::Client<ChangeState>::SharedPtr gates_pub_;
  rclcpp::Client<ChangeState>::SharedPtr obstacles_pub_;
  rclcpp::Client<ChangeState>::SharedPtr victims_pub_;
  rclcpp::Client<ChangeState>::SharedPtr init_pos_pub_;

  uint8_t borders_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  uint8_t gates_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  uint8_t obstacles_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  uint8_t victims_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  uint8_t init_pos_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

  // Parameters
  bool victims_activated = false;
  std::vector<std::string> robot_names;
  std::vector<double> init_x;
  std::vector<double> init_y;
  std::vector<double> init_yaw;

 public:
  Orchestrator()
      : rclcpp_lifecycle::LifecycleNode(
            "orchestrator", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    // Create two callback groups
    auto cb_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto cb_group2 = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group1;

    // Declare parameters
    this->declare_parameter("victims_activated", false);
    this->declare_parameter("init_names", std::vector<std::string>());
    this->declare_parameter("init_x", std::vector<double>());
    this->declare_parameter("init_y", std::vector<double>());
    this->declare_parameter("init_yaw", std::vector<double>());

    // Read
    this->get_parameter("victims_activated", victims_activated);
    robot_names = this->get_parameter("init_names").as_string_array();
    init_x   = this->get_parameter("init_x").as_double_array();
    init_y   = this->get_parameter("init_y").as_double_array();
    init_yaw = this->get_parameter("init_yaw").as_double_array();

    // Print parameters
    RCLCPP_INFO(this->get_logger(), "Parameter 'victims_activated' = %d",
                victims_activated);
    for (auto name : this->robot_names){
      RCLCPP_INFO(this->get_logger(), "Robot name: %s", name.c_str());
    }
    for (auto x : this->init_x){
      RCLCPP_INFO(this->get_logger(), "Robot x: %f", x);
    }
    for (auto y : this->init_y){
      RCLCPP_INFO(this->get_logger(), "Robot y: %f", y);
    }
    for (auto yaw : this->init_yaw){
      RCLCPP_INFO(this->get_logger(), "Robot yaw: %f", yaw);
    }

    if (
        // this->robot_names.size() != this->init_x.size() ||
        // this->robot_names.size() != this->init_y.size() ||
        // this->robot_names.size() != this->init_yaw.size() || 
        this->init_x.size()     != this->init_y.size() ||
        this->init_x.size()     != this->init_yaw.size() ||
        this->init_y.size()     != this->init_yaw.size() ||
        // this->robot_names.size() == 0 || this->init_x.size() == 0 ||
        this->init_y.size() == 0 || this->init_yaw.size() == 0) {
      throw std::runtime_error("The number of robots is not the same for all parameters");
    }

    // Print parameters
    RCLCPP_INFO(this->get_logger(), "Parameter 'victims_activated' = %d",
                victims_activated);
    for (size_t i=0; i<init_x.size(); i++){
      RCLCPP_INFO(this->get_logger(), "at (%f, %f) th: %f", 
                  // robot_names[i].c_str(), 
                  init_x[i], init_y[i], init_yaw[i]);
    }

    // Create a subscription to the state_transition events of the send_borders node
    borders_sub_ = this->create_subscription<TransitionEvent>(
        "/send_borders/transition_event", 10,
        std::bind(&Orchestrator::borders_callback, this, std::placeholders::_1),
        sub_options);

    // Create publisher to send events to the send_borders node
    this->border_pub_ = this->create_client<ChangeState>(
        "/send_borders/change_state", rmw_qos_profile_services_default, cb_group2);

    // Create a subscription to the state_transition events of the send_gates node
    gates_sub_ = this->create_subscription<TransitionEvent>(
        "/send_gates/transition_event", 10,
        std::bind(&Orchestrator::gates_callback, this, std::placeholders::_1),
        sub_options);

    // Create publisher to send events to the send_gates node
    this->gates_pub_ = this->create_client<ChangeState>(
        "/send_gates/change_state", rmw_qos_profile_services_default, cb_group2);

    // Create a subscription to the state_transition events of the send_obstacles node
    obstacles_sub_ = this->create_subscription<TransitionEvent>(
        "/send_obstacles/transition_event", 10,
        std::bind(&Orchestrator::obstacles_callback, this, std::placeholders::_1),
        sub_options);

    // Create publisher to send events to the send_obstacles node
    this->obstacles_pub_ = this->create_client<ChangeState>(
        "/send_obstacles/change_state", rmw_qos_profile_services_default, cb_group2);

    // Create a subscription to the state_transition events of the send_init_pos node
    init_pos_sub_ = this->create_subscription<TransitionEvent>(
        "/send_initial_pose/transition_event", 10,
        std::bind(&Orchestrator::init_pos_callback, this, std::placeholders::_1),
        sub_options);

    // Create publisher to send events to the send_init_pos node
    this->init_pos_pub_ = this->create_client<ChangeState>(
        "/send_init_pos/change_state", rmw_qos_profile_services_default, cb_group2);

    // Create a subscription to the state_transition events of the send_victims node
    if (this->victims_activated) {
      victims_sub_ = this->create_subscription<TransitionEvent>(
          "/send_victims/transition_event", 10,
          std::bind(&Orchestrator::victims_callback, this, std::placeholders::_1),
          sub_options);

      // Create publisher to send events to the send_victims node
      this->victims_pub_ = this->create_client<ChangeState>(
          "/send_victims/change_state", rmw_qos_profile_services_default, cb_group2);
    } else {
      RCLCPP_INFO(this->get_logger(), "Victims node is not enabled");
    }
  }

  /**
   * @brief Change the state of the node to Configure. It send the configure message
   * to the send_* nodes.
   *
   * @param state State
   * @return CallbackReturn SUCCESS if no error happens
   */
  CallbackReturn on_configure(rclcpp_lifecycle::State const& state) {
    // Send configure message to the nodes
    RCLCPP_INFO(this->get_logger(), "Configuring borders node");
    if (!change_state<Orchestrator*>(this, this->border_pub_,
                                     Transition::TRANSITION_CONFIGURE)) {
      throw std::runtime_error("Failed to configure send_borders node");
    }
    RCLCPP_INFO(this->get_logger(), "Configuring gates node");
    if (!change_state<Orchestrator*>(this, this->gates_pub_,
                                     Transition::TRANSITION_CONFIGURE)) {
      throw std::runtime_error("Failed to configure send_gates node");
    }
    RCLCPP_INFO(this->get_logger(), "Configuring obstacles node");
    if (!change_state<Orchestrator*>(this, this->obstacles_pub_,
                                     Transition::TRANSITION_CONFIGURE)) {
      throw std::runtime_error("Failed to configure send_obstacles node");
    }
    if (this->victims_activated) {
      RCLCPP_INFO(this->get_logger(), "Configuring victims node");
      if (!change_state<Orchestrator*>(this, this->victims_pub_,
                                       Transition::TRANSITION_CONFIGURE)) {
        throw std::runtime_error("Failed to configure send_victims node");
      }
    }
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Change the state of the node to Activate.
   *
   * @param state State
   * @return CallbackReturn SUCCESS
   */
  CallbackReturn on_activate(rclcpp_lifecycle::State const& state) {
    RCLCPP_INFO(this->get_logger(), "Waiting for all nodes to be active");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Change the state of the node to Shutdown.
   *
   * @param state State
   * @return CallbackReturn SUCCESS
   */
  CallbackReturn on_shutdown(rclcpp_lifecycle::State const& state) {
    RCLCPP_INFO(this->get_logger(), "All nodes are active, terminating orchestrator");
    return CallbackReturn::SUCCESS;
  }

 private:
  /**
   * @brief Callback function for the send_borders node changes in states.
   * @details During the orchestrator configuration, it will send a configure message to
   * the send_borders node. When the send_borders node is configured, it will then send an
   * activate message to it. When the send_borders node is active and the send_gates node
   * is configured, it will send an activate message to the send_gates node.
   *
   * @param msg The transition message received from the send_borders node.
   */
  void borders_callback(TransitionEvent::SharedPtr const msg) {
    RCLCPP_INFO(
        this->get_logger(),
        "Received state_transition event from send_borders: '%s' from '%d' to '%d",
        msg->goal_state.label.c_str(), this->borders_state_, msg->goal_state.id);

    // When borders node is inactive from configuring
    if (this->borders_state_ == 10 && msg->goal_state.id == 2) {
      RCLCPP_INFO(this->get_logger(),
                  "[1/1-borders] Borders node is configured and inactive, activating "
                  "borders node");
      if (!change_state(this, this->border_pub_, Transition::TRANSITION_ACTIVATE)) {
        throw std::runtime_error("Failed to activate send_borders node");
      }

      // Sleep for 1 seconds and test if the node is active by calling the service /send_borders/get_state
      RCLCPP_INFO(this->get_logger(), "Checking if send_borders is active1");
      sleep(0.1);
      RCLCPP_INFO(this->get_logger(), "Checking if send_borders is active2");
      auto client = this->create_client<lifecycle_msgs::srv::GetState>("/send_borders/get_state");
      while(!client->wait_for_service(std::chrono::seconds(1))){
        if (!rclcpp::ok()){
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }

      auto service_msg = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

      using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture;
      auto response_received_callback = [](ServiceResponseFuture future) {
        auto response = future.get();
        RCLCPP_INFO(rclcpp::get_logger("orchestrator"), "send_borders current state: %s", response->current_state.label.c_str());
      };

      auto result = client->async_send_request(service_msg, response_received_callback);
    }

    // When borders node is active from activating
    if (this->borders_state_ == 13 && msg->goal_state.id == 3) {
      RCLCPP_INFO(this->get_logger(),
                  "[1/2-borders] Borders node is configured and active");
      // If the gate node has been configured and is now inactive
      if (this->gates_state_ == 2) {
        RCLCPP_INFO(this->get_logger(),
                    "[2/2-borders] and gates node is configure and inactive, so "
                    "activating gates node");
        if (!change_state(this, this->gates_pub_, Transition::TRANSITION_ACTIVATE)) {
          throw std::runtime_error("Failed to activate send_gates node");
        }
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "[2/2-borders] but gates node is not configure and inactive, so not "
                    "activating gates node");
      }
    }

    this->borders_state_ = msg->goal_state.id;
  }

  /**
   * @brief Callback function for the send_gates node changes in states.
   * @details During the orchestrator configuration, it will send a configure message to
   * the send_gates node. When the send_gates node is configured, it will then send an
   * activate message to it. When the send_gates node is active and the send_obstacles
   * node is configured, it will send an activate message to the send_obstacles node.
   * It's possible that the orchestrator may have received before the configured message
   * gates than from the borders node, hence it will check if the borders node is active
   * before activating the gates node.
   *
   * @param msg The transition message received from the send_gates node.
   */
  void gates_callback(TransitionEvent::SharedPtr const msg) {
    RCLCPP_INFO(this->get_logger(),
                "Received state_transition event from send_gates: '%s' from '%d' to '%d",
                msg->goal_state.label.c_str(), this->gates_state_, msg->goal_state.id);

    // If the gate node is inactive from configuring
    if (this->gates_state_ == 10 && msg->goal_state.id == 2) {
      RCLCPP_INFO(this->get_logger(),
                  "[1/2-gates] Gates node is configured and inactive");
      // If the border node is active, then ask the gates node to activate
      if (this->borders_state_ == 3) {
        RCLCPP_INFO(this->get_logger(),
                    "[2/2-gates] and borders node is active, so activating gates node");
        if (!change_state(this, this->gates_pub_, 3)) {
          throw std::runtime_error("Failed to activate send_gates node");
        }
      } else {
        RCLCPP_INFO(
            this->get_logger(),
            "[2/2-gates] but borders node is not active, so not activating gates node");
      }
    }

    // If the gate node is active from activating
    if (this->gates_state_ == 13 && msg->goal_state.id == 3) {
      RCLCPP_INFO(this->get_logger(), "[1/2-gates] Gates node is configured and active");
      // If the obstacles node has been configured and is now inactive
      if (this->obstacles_state_ == 2) {
        RCLCPP_INFO(this->get_logger(),
                    "[2/2-gates] and obstacles node is configured and inactive, so "
                    "activating obstacles node");
        // Send activate message to the send_obstacles node
        if (!change_state(this, this->obstacles_pub_, 3)) {
          throw std::runtime_error("Failed to activate send_obstacles node");
        }
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "[2/2-gates] but obstacles node is not configured and inactive, so "
                    "not activating obstacles node");
      }
    }
    this->gates_state_ = msg->goal_state.id;
  }

  /**
   * @brief Callback function for the send_obstacles node changes in states.
   * @details During the orchestrator configuration, it will send a configure message to
   * the send_obstacles node. When the send_obstacles node is configured, it will then
   * send an activate message to it. When the send_obstacles node is active and the
   * send_victims node is configured, it will send an activate message to the send_victims
   * node. It's possible that the orchestrator may have received before the configured
   * message obstacles than from the gates node, hence it will check if the gates node is
   * active before activating the obstacles node.
   *
   * @param msg The transition message received from the send_obstacles node.
   */
  void obstacles_callback(TransitionEvent::SharedPtr const msg) {
    RCLCPP_INFO(
        this->get_logger(),
        "Received state_transition event from send_obstacles: '%s' from '%d' to '%d",
        msg->goal_state.label.c_str(), this->obstacles_state_, msg->goal_state.id);

    // If the obstacles node is inactive from configuring
    if (this->obstacles_state_ == 10 && msg->goal_state.id == 2) {
      RCLCPP_INFO(this->get_logger(),
                  "[1/2-obstacles] Obstacles node is configured and inactive");
      // If the border node is active, then ask the obstacles node to activate
      if (this->gates_state_ == 3) {
        RCLCPP_INFO(this->get_logger(),
                    "[2/2-obstacles] and gates is active, so activating obstacles node");
        if (!change_state(this, this->obstacles_pub_, 3)) {
          throw std::runtime_error("Failed to activate send_obstacles node");
        }
      } else {
        RCLCPP_INFO(
            this->get_logger(),
            "[2/2-obstacles] but gates is not active, so not activating obstacles node");
      }
    }
    // If the obstacles node is active from activating
    if (this->victims_activated && this->obstacles_state_ == 13 &&
        msg->goal_state.id == 3) {
      RCLCPP_INFO(this->get_logger(),
                  "[1/1-obstacles] Obstacles node is configured and active");
      if (this->victims_state_ == 2) {
        RCLCPP_INFO(this->get_logger(),
                    "[2/2-obstacles] and victims is configured and inactive, so "
                    "activating victims node");
        if (!change_state(this, this->victims_pub_, 3)) {
          throw std::runtime_error("Failed to activate send_victims node");
        }
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "[2/2-obstacles] but victims is not configured and inactive, so not "
                    "activating victims node");
      }
    }
    this->obstacles_state_ = msg->goal_state.id;
    if (!this->victims_activated && this->borders_state_ == 3 &&
        this->gates_state_ == 3 && this->obstacles_state_ == 3) {
      this->shutdown();
    }
  }

  /**
   * @brief Callback function for the send_victims node changes in states.
   * @details During the orchestrator configuration, it will send a configure message to
   * the send_victims node. When the send_victims node is configured, it will then send an
   * activate message to it. It's possible that the orchestrator may have received before
   * the configured message victims than from the obstacles node, hence it will check if
   * the obstacles node is active before activating the victims node.
   * This callback is only called if the victims node is enabled.
   *
   * @param msg The transition message received from the send_victims node.
   */
  void victims_callback(TransitionEvent::SharedPtr const msg) {
    RCLCPP_INFO(
        this->get_logger(),
        "Received state_transition event from send_victims: '%s' from '%d' to '%d",
        msg->goal_state.label.c_str(), this->victims_state_, msg->goal_state.id);

    if (this->victims_activated) {
      if (this->victims_state_ == 10 && msg->goal_state.id == 2) {
        RCLCPP_INFO(this->get_logger(),
                    "[1/2-victims] Victims node is configured and inactive");
        // If the obstacle node is active, then ask the victims node to activate
        if (this->obstacles_state_ == 3) {
          RCLCPP_INFO(
              this->get_logger(),
              "[2/2-victims] and obstacles is active, so activating victims node");
          if (!change_state(this, this->victims_pub_, 3)) {
            throw std::runtime_error("Failed to activate send_victims node");
          }
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "[2/2-victims] but obstacles is not active, so not activating "
                      "victims node");
        }
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Victims node is not enabled");
    }
    this->victims_state_ = msg->goal_state.id;

    if (this->borders_state_ == 3 && this->gates_state_ == 3 &&
        this->obstacles_state_ == 3 && this->victims_state_ == 3) {
      this->shutdown();
    }
  }

  void init_pos_callback(TransitionEvent::SharedPtr const msg) {
    RCLCPP_INFO(
        this->get_logger(),
        "Received state_transition event from send_initial_pose: '%s' from '%d' to '%d",
        msg->goal_state.label.c_str(), this->init_pos_state_, msg->goal_state.id);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  Orchestrator::SharedPtr node = std::make_shared<Orchestrator>();
  node->configure();
  node->activate();

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
