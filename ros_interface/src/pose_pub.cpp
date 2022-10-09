#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>

#include "Publisher.hpp"
#include "Subscriber.hpp"
#include "Replier.hpp"
#include "json.hpp"

#include <boost/thread/thread.hpp>


using namespace nlohmann;

void runNode(int argc, char* argv[]);


int main(int argc, char* argv[])
{
  runNode(argc, argv);
  return 0;
}


void pose_publisher()
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");

  // configuring parameters
  std::string map_frame, base_frame;
  double publish_frequency;

  nh_priv->param<std::string>("map_frame", map_frame, "/map");
  nh_priv->param<std::string>("base_frame", base_frame, "/base_link");
  nh_priv->param<double>("pose_publish_frequency", publish_frequency, 50);

  Common::Publisher p_pub("tcp://*:9207");
  
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));
  
  ros::Rate rate(publish_frequency);
  while (nh->ok())
  {
    tf::StampedTransform transform;
    try
    {
      double x, y, theta;
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
      auto const & rot = transform.getRotation();
      tf::Quaternion q(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      x = transform.getOrigin().getX();
      y = transform.getOrigin().getY();
      theta = yaw;
      
      json jobj;
      jobj["loc_data"]["x"] = x;
      jobj["loc_data"]["y"] = y;
      jobj["loc_data"]["theta"] = theta;
    
      std::string jmsg = jobj.dump();
      p_pub.send("POS", jmsg.c_str(), jmsg.size());
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
      std::cerr << ex.what() << std::endl;
    }

    rate.sleep();
  }
  
}




void runNode(int argc, char* argv[])
{
  ros::init(argc, argv, "pose_pub_node");
  
  // spawn threads
  boost::thread thread_pose(pose_publisher);
  
  // wait until ros termination
  ros::spin();

  thread_pose.join();
}
