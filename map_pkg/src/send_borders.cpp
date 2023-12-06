#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"


#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/header.hpp"

geometry_msgs::msg::Polygon create_exagon(){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = -6;
        point.y = 10;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 6;
        point.y = 10;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 12;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 6;
        point.y = -10;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -6;
        point.y = -10;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -12;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}


geometry_msgs::msg::Polygon create_10square(){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = -5; // -2.5;
        point.y = -5; // -5;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -5; //-3.86;
        point.y = 5; //-2.6;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 5; //1.84;
        point.y = 5; //0.794;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 5; //3.29;
        point.y = -5; //-1.55;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}

geometry_msgs::msg::Polygon create_8square(){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = -4;
        point.y = -4;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -4;
        point.y = 4; 
        point.z = 0;
        points_temp.push_back(point);
        point.x = 4; 
        point.y = 4;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 4; 
        point.y = -4;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}

geometry_msgs::msg::Polygon create_50square(){
        geometry_msgs::msg::Polygon pol;
        geometry_msgs::msg::Point32 point;
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = -25;
        point.y = -25;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -25;
        point.y = 25;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 25;
        point.y = 25;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 25;
        point.y = -25;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        return pol;
}

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

class BordersPublisher : public rclcpp::Node
{
  public:
    BordersPublisher()
    : Node("send_borders")
    {
        std::string map_name = this->get_parameter("map").as_string();  // exagon, 10square, 8square, 50square

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
        publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("map_borders", qos);

        std_msgs::msg::Header hh;

        hh.stamp = this->get_clock()->now();
        hh.frame_id = "map";

        geometry_msgs::msg::Polygon pol;

        geometry_msgs::msg::PolygonStamped pol_stamped;

        pol_stamped.header = hh;

        if(map_name=="exagon")        pol = create_exagon();
        else if(map_name=="10square") pol = create_10square();
        else if(map_name=="50square")  pol = create_50square();
        else                          pol = create_8square();
        
        pol_stamped.polygon = pol;

        pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("borders", qos);

        publisher_->publish(pol);
        pub_->publish(pol_stamped);
        usleep(1000000);
    }

  
  private:
    
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BordersPublisher>());
  rclcpp::shutdown();
  return 0;
}