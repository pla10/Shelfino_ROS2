#include "ros/ros.h"

#include <visualization_msgs/Marker.h>

#include "utils.hpp"

#include <vector>

void runNode(int argc, char* argv[]);


int main(int argc, char* argv[])
{
  runNode(argc, argv);
  return 0;
}



void map_publisher()
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");


  std::vector<std::string> keys;
  nh_priv->getParamNames(keys);
  for (std::string s: keys) {
    std::cout << s << std::endl;
  }

  // configuring parameters
  std::string map_topic, map_file;
  if (!nh_priv->getParam("map_topic", map_topic))
  {
    map_topic = "/cad_map";
  }
  if (!nh_priv->getParam("map_file", map_file))
  {
     ROS_ERROR_STREAM("Failed to retrieve parameter \"" << "map_file" << "\"");
     return;
  }

  // nh->param<std::string>("/map_file", map_file); //, "./data/Povo2_floor1_offset.txt");

  ros::Publisher publisher = nh->advertise<visualization_msgs::Marker>(map_topic, 1, true);

  visualization_msgs::Marker msg;
  msg.header.frame_id = "cad_map";
  msg.header.stamp = ros::Time::now();

  msg.ns = "cad_map";
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1.0;

  msg.id = 1;
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.scale.x = 0.1; // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

  msg.color.r = 1.0;
  msg.color.a = 1.0;

  std::vector<Obstacle> obstacles;
  if (!getObstacles(map_file, obstacles)){
    ROS_ERROR_STREAM("Failed to load map " << map_file);
    return;
  }
  else {
    ROS_INFO_STREAM("READ " << obstacles.size() << " OBSTACLES!");
  }

  for (const Obstacle & o: obstacles) {
    for (int i=1; i<o.size(); ++i) {
      geometry_msgs::Point p0;
      p0.x = o[i-1].x;
      p0.y = o[i-1].y;
      p0.z = 0.;
      
      geometry_msgs::Point p1;
      p1.x = o[i].x;
      p1.y = o[i].y;
      p1.z = 0.;
      
      msg.points.push_back(p0);
      msg.points.push_back(p1);
    }
  }

  publisher.publish(msg);


  ros::Rate rate(10);
  
  while (ros::ok())
  {
    rate.sleep();
  }

}


void runNode(int argc, char* argv[])
{
  ros::init(argc, argv, "map_drawer_node");
  map_publisher();  
}
