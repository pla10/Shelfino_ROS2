#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

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
        false};

class MapBordersMarker : public rclcpp::Node
{
public:
    MapBordersMarker()
        : Node("map_borders_marker_node")
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("map_markers", rclcpp::QoS(10));

        map_borders_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            "map_borders",
            qos,
            std::bind(&MapBordersMarker::mapBordersCallback, this, std::placeholders::_1));
    }

private:
    void mapBordersCallback(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received %lu points for map borders.", msg->points.size());

        if (msg->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty map borders.");
            return;
        }

        sampled_points_.clear();
        for (const auto &point : msg->points)
        {
            sampled_points_.push_back({point.x, point.y});
        }

        publishMapBordersMarker();
    }

    void publishMapBordersMarker()
    {
        if (sampled_points_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No sampled points to publish.");
            return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Cambia se necessario
        marker.header.stamp = this->now();
        marker.ns = "sampled_points";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; // Individual points
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Define marker properties
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 1.0; // Verde per maggiore visibilità
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (const auto &point : sampled_points_)
        {
            geometry_msgs::msg::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published map borders with %zu points.", marker.points.size());
    }

    std::vector<std::vector<double>> sampled_points_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr map_borders_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MapBordersMarker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
