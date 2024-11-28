#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

#include "map_pkg/utilities.hpp"


using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief This node publishes the borders of the map and spawns them in gazebo.
 * @details The borders can be a hexagon or a rectangle. The size of the map is defined by
 * the edges of the hexagon or the rectangle. If it's an hexagon, then all the edges will
 * have the same length and only dx should be set, whereas, if it's a rectangle, both dx
 * and dy can be set. The borders are published as a Polygon message and a PolygonStamped
 * message. The Polygon message is used to spawn the borders in gazebo.
 * During configuration, it declares and reads the parameters map, dx, and dy. The map
 * parameter can be "hexagon" or "rectangle". The dx and dy parameters are the size of the
 * map. During activation, it creates the Polygon message and the PolygonStamped message,
 * publishes them, and spawns the borders in gazebo. The models are created by reading the
 * SDF files in the worlds directory of the package, "/worlds/hexagon_world/model.sdf" and
 * "/worlds/rectangle_world/model.sdf".
 */
class BordersPublisher : public rclcpp_lifecycle::LifecycleNode {
 private:
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;

  // Data obtained from parameters
  struct Data {
    std::string map_name;
    double dx;
    double dy;
  } data;

 public:
  explicit BordersPublisher(bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(
            "send_borders",
            rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {
    RCLCPP_INFO(this->get_logger(), "Node created.");
  }

  /**
   * @brief This function is called when the node is in the configuring state.
   * @details It creates and reads teh parameters. It also sets up the publishers and the
   * client to spawn the model in gazebo. There are two publishers:
   * - /map_borders: Publishes the borders of the map as a Polygon message. The script
   *   create+map_pgm.py reads from this topic.
   * - /borders: Publishes the borders of the map as a PolygonStamped message. The other
   *   nodes should be reading from this topic.
   *
   * @param state State
   * @return CallbackReturn Success if the node is configured successfully.
   */
  CallbackReturn on_configure(rclcpp_lifecycle::State const& state) {
    RCLCPP_INFO(this->get_logger(), "Configuring node.");
    LifecycleNode::on_configure(state);
    this->declare_parameter<std::string>("map", "hexagon");
    this->declare_parameter<double>("dx", 5.0);
    this->declare_parameter<double>("dy", 5.0);
    this->data.map_name = this->get_parameter("map").as_string();  // hexagon, rectangle
    this->data.dx = this->get_parameter("dx").as_double();
    this->data.dy = this->get_parameter("dy").as_double();

    if (this->data.map_name != "hexagon" && this->data.map_name != "rectangle") {
      RCLCPP_ERROR(this->get_logger(), "Map name %s not recognized",
                   this->data.map_name.c_str());
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "Map name: %s", this->data.map_name.c_str());
    RCLCPP_INFO(this->get_logger(), "dx: %f", this->data.dx);
    RCLCPP_INFO(this->get_logger(), "dy: %f", this->data.dy);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

    auto callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant, true);

    this->publisher_ =
        this->create_publisher<geometry_msgs::msg::Polygon>("/map_borders", qos);
    this->pub_ =
        this->create_publisher<geometry_msgs::msg::PolygonStamped>("/borders", qos);

    RCLCPP_INFO(this->get_logger(), "Node configured.");

    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief This function is called when the node is in the activating state.
   * @details It creates the Polygon message and the PolygonStamped message and publishes
   * them. It also spawns the borders in gazebo. The models are created by reading the SDF
   * files in the worlds directory of the package, "/worlds/hexagon_world/model.sdf" and
   * "/worlds/rectangle_world/model.sdf". It calls either `map_hexagon()` or
   * `map_rectangle()` to create the models correctly.
   *
   * @param state
   * @return CallbackReturn
   */
  CallbackReturn on_activate(rclcpp_lifecycle::State const& state) {
    RCLCPP_INFO(this->get_logger(), "Activating node.");
    LifecycleNode::on_activate(state);
    std_msgs::msg::Header hh;

    hh.stamp = this->get_clock()->now();
    hh.frame_id = "map";

    geometry_msgs::msg::Polygon pol;
    std::string xml_string;

    if (this->data.map_name == "hexagon") {
      pol = create_hexagon(this->data.dx);
    } else if (this->data.map_name == "rectangle") {
      pol = create_rectangle(this->data.dx, this->data.dy);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Map name %s not recognized",
                   this->data.map_name.c_str());
    }

    geometry_msgs::msg::PolygonStamped pol_stamped;
    pol_stamped.header = hh;
    pol_stamped.polygon = pol;

    // Publish borders
    this->publisher_->publish(pol);
    this->pub_->publish(pol_stamped);

    RCLCPP_INFO(this->get_logger(), "Node active.");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(rclcpp_lifecycle::State const& state) {
    RCLCPP_INFO(this->get_logger(), "Deactivating node.");
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
  }
};


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BordersPublisher>();

  node->configure();
  node->activate();

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}


