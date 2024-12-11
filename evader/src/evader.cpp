#include "evader/evader.hpp"


Evader::Evader() : Node("evader")
{
  // Define the QoS
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

  // Define the parameters
  this->declare_parameter("difficulty", 0);
  
  // Get parameters 
  this->difficulty = this->get_parameter("difficulty").as_int();
  if (this->difficulty == 0){
    RCLCPP_ERROR(this->get_logger(), "Difficulty not set.");
    exit(1);
  }
  if (this->difficulty < 1 || this->difficulty > 4){
    RCLCPP_ERROR(this->get_logger(), "Difficulty must be between 1 and 4.");
    exit(1);
  }

  if (this->difficulty == 4){
    this->n_pursuers = 2;
  }
  
  // Print the parameters
  RCLCPP_INFO(this->get_logger(), "Difficulty: %d", difficulty);
  RCLCPP_INFO(this->get_logger(), "Number of pursuers: %d", n_pursuers);

  auto amcl_pose_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions amcl_pose_options;
  amcl_pose_options.callback_group = amcl_pose_cb_group;

  // Listen to the /evader/amcl_pose topic
  this->evader_pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
    "/evader/amcl_pose", qos, std::bind(&Evader::handle_evader_pose, this, std::placeholders::_1),
    amcl_pose_options
  );

  // Listen to the /pursuer/amcl_pose topic
  this->pursuer_pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
    "/pursuer/amcl_pose", qos, std::bind(&Evader::handle_pursuer_pose, this, std::placeholders::_1),
    amcl_pose_options
  );

  // Listen to the /gate_pose topic
  this->gates_pose_sub_ = this->create_subscription<PoseArray>(
    "/gates", qos, std::bind(&Evader::handle_gate_pose, this, std::placeholders::_1)
  );

  // Create the publisher to tell the pursuers that the evader is ready
  this->evader_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("ready", qos);
  auto msg = std_msgs::msg::Bool();
  msg.data = false;
  this->evader_ready_pub_->publish(msg);

  // Create the start service
  this->start_service_ = this->create_service<Empty>(
    "start", std::bind(&Evader::start_callback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create the action client
  this->compute_path_to_pose_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
  this->follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

  RCLCPP_INFO(this->get_logger(), "Evader node created.");
}

void Evader::handle_evader_pose(const PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received evader pose: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  this->evader_pose = msg->pose.pose;
}

void Evader::handle_pursuer_pose(const PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received pursuer pose: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  this->pursuer_pose = msg->pose.pose;
}

void Evader::handle_gate_pose(const PoseArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "%ld gates pose received.", msg->poses.size());
  for (size_t i=0; i<msg->poses.size(); i++){
    double yaw = get_yaw_from_q(msg->poses[i].orientation);
    RCLCPP_INFO(this->get_logger(), "%ld pose: x=%f, y=%f, th=%f", i, msg->poses[i].position.x, msg->poses[i].position.y, yaw);
  }

  // Check # of gates is correct
  if (this->difficulty == 1 && msg->poses.size() != 1){
    RCLCPP_ERROR(this->get_logger(), "Difficulty 1 requires only one gate.");
    exit(1);
  }
  else if (this->difficulty > 1 && msg->poses.size() != 2){
    RCLCPP_ERROR(this->get_logger(), "Difficulty 2 or above requires exactly two gates.");
    exit(1);
  }

  if (this->difficulty == 1){ 
    this->gate_pose = msg->poses[0];
  }
  else if (this->difficulty == 2){
    this->gate_pose = msg->poses[rand() % 2];
  }
  else if (this->difficulty == 3){
    this->choose_gates_wrapper_(msg->poses);
  }

  this->gate_pose_ready = true;
  RCLCPP_INFO(this->get_logger(), "Gate pose set to x=%f, y=%f, yaw=%f", this->gate_pose.position.x, this->gate_pose.position.y, get_yaw_from_q(this->gate_pose.orientation));

  // Publish that the gate pose is ready
  auto msg_ready = std_msgs::msg::Bool();
  msg_ready.data = true;
  this->evader_ready_pub_->publish(msg_ready);
}

void Evader::choose_gates_wrapper_(const std::vector<Pose>& gates)
{
  // Compute distance to each gate
  double pursuer_dist1 = euclidean_distance(this->pursuer_pose.position, gates[0].position);
  double pursuer_dist2 = euclidean_distance(this->pursuer_pose.position, gates[1].position);

  // Compute distance from evader to each gate
  double evader_dist1 = euclidean_distance(this->evader_pose.position, gates[0].position);
  double evader_dist2 = euclidean_distance(this->evader_pose.position, gates[1].position);

  bool gate_set = false;

  // Choose the gate that is further from the pursuer and closer to the evader
  for(double dist_weight = 1; dist_weight > 0.7 && !gate_set; dist_weight -= 0.2){
    if (evader_dist1 < pursuer_dist1 * dist_weight){
      if (evader_dist2 < pursuer_dist2 * dist_weight){
        RCLCPP_INFO(this->get_logger(), "Both gates are closer to the evader than the pursuer, choosing randomly.");
        this->gate_pose = gates[rand() % 2];
        gate_set = true;
      }
      else{
        RCLCPP_INFO(this->get_logger(), "Gate 1 (%f, %f) is closer to the evader than the pursuer, choosing it.", gates[0].position.x, gates[0].position.y);
        this->gate_pose = gates[0];
        gate_set = true;
      }
    }
    else if (evader_dist2 < pursuer_dist2 * dist_weight){
      RCLCPP_INFO(this->get_logger(), "Gate 2 (%f, %f) is closer to the evader than the pursuer, choosing it.", gates[1].position.x, gates[1].position.y);
      this->gate_pose = gates[1];
      gate_set = true;
    }
  }

  if (!gate_set){
    RCLCPP_INFO(this->get_logger(), "No gate is closer to the evader than the pursuer, choosing randomly. pursuer_dist1 = %f, pursuer_dist2 = %f, evader_dist1 = %f, evader_dist2 = %f", pursuer_dist1, pursuer_dist2, evader_dist1, evader_dist2);
    this->gate_pose = gates[rand() % 2];
  }
}

void Evader::start_callback(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "`start` service called.");
  if (!this->gate_pose_ready){
    RCLCPP_ERROR(this->get_logger(), "Gate pose not set.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Navigating to x=%f, y=%f, yaw=%f", this->gate_pose.position.x, this->gate_pose.position.y, get_yaw_from_q(this->gate_pose.orientation));

  if(!this->compute_path_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(this->get_logger(), "Action server for ComputePathToPose not available after waiting.");
    exit(1);
  }

  auto action_msg = ComputePathToPose::Goal();
  action_msg.goal.pose = this->gate_pose;
  action_msg.use_start = false;
  action_msg.goal.header.stamp = this->now();
  action_msg.goal.header.frame_id = "map";

  RCLCPP_INFO(this->get_logger(), "Sending goal to ComputePathToPose action server.");

  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&Evader::goal_response_path_planning_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&Evader::feedback_path_planning_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&Evader::result_path_planning_callback, this, std::placeholders::_1);

  this->compute_path_to_pose_client_->async_send_goal(action_msg, send_goal_options);
}

void Evader::goal_response_path_planning_callback(
  const ClientComputePathToPoseGoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle){
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the ComputePathToPose server.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the ComputePathToPose server.");
}

void Evader::feedback_path_planning_callback(
  ClientComputePathToPoseGoalHandle::SharedPtr,
  const std::shared_ptr<const ComputePathToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback from ComputePathToPose");
}

void Evader::result_path_planning_callback(
  const ClientComputePathToPoseGoalHandle::WrappedResult & result)
{
  switch (result.code){
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Path planned, starting to move.");
      this->move(result.result->path);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted from ComputePathToPose");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled by ComputePathToPose");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[ComputePathToPose] Unknown result code");
      exit(1);
      break;
  }
}

void Evader::move(const Path& path)
{
  RCLCPP_INFO(this->get_logger(), "Moving to gate");
  if(!this->follow_path_client_->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(this->get_logger(), "Action server for FollowPath not available after waiting.");
    exit(1);
  }

  auto action_msg = FollowPath::Goal();
  action_msg.path = path;

  RCLCPP_INFO(this->get_logger(), "Sending goal to FollowPath action server.");

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&Evader::goal_response_path_following_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&Evader::feedback_path_following_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&Evader::result_path_following_callback, this, std::placeholders::_1);

  this->follow_path_client_->async_send_goal(action_msg, send_goal_options);
}

void Evader::goal_response_path_following_callback(
  const ClientFollowPathGoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle){
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the FollowPath server.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the FollowPath server.");
}

void Evader::feedback_path_following_callback(
  ClientFollowPathGoalHandle::SharedPtr,
  const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback from FollowPath");
}

void Evader::result_path_following_callback(
  const ClientFollowPathGoalHandle::WrappedResult & result)
{
  switch (result.code){
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Path followed, reached gate.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted from FollowPath");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled by FollowPath");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[FollowPath] Unknown result code");
      exit(1);
      break;
  }
}


int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("evader"), "Starting evader node.");
  srand(time(NULL));
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Evader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

