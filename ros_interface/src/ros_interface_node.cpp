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

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>



//#define REALPOSE

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

  Common::Publisher p_pub("tcp://*:9112");
  
  #ifndef REALPOSE
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));
  #else
  ros::ServiceClient get_models = nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  #endif

  ros::Rate rate(publish_frequency);
  while (nh->ok())
  {
    tf::StampedTransform transform;
    try
    {
      double x, y, theta;
      #ifndef REALPOSE
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
      auto const & rot = transform.getRotation();
      tf::Quaternion q(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      x = transform.getOrigin().getX();
      y = transform.getOrigin().getY();
      theta = yaw;
      #else
      gazebo_msgs::GetModelState getModelState;
      getModelState.request.model_name = "my_robot";
      getModelState.request.relative_entity_name = "world" ;
      get_models.call(getModelState);
      if(getModelState.response.success) {
        auto pos = getModelState.response.pose.position;
        auto rot = getModelState.response.pose.orientation;
        tf::Quaternion q(rot.x, rot.y, rot.z, rot.w);  
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        x = pos.x;
        y = pos.y;
        theta = yaw;
      }
      #endif

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

void scan_publisher()
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");

  // configuring parameters
  std::string scan_topic;
  nh_priv->param<std::string>("scan_topic", scan_topic, "/scan");

  Common::Publisher s_pub("tcp://*:7500");
  boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)> cb = [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
    json jobj;
    jobj["size"] = msg->ranges.size();
    std::vector<float> ranges;
    ranges.reserve(msg->ranges.size());
    for (float r: msg->ranges) {
      if (std::isinf(r) || std::isnan(r)) {
        r = 0.;
      }
      ranges.push_back(r);
    }
    jobj["data"] = ranges;

    std::string jmsg = jobj.dump();
    s_pub.send("LIDAR", jmsg.c_str(), jmsg.size());
  };

  // define user callback queue
  ros::CallbackQueue scan_queue;
  
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
    ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
      scan_topic, // topic name
      1, // queue length
      cb, // callback
      nullptr,
      &scan_queue // pointer to callback queue object
    );
  // subscribe
  ros::Subscriber scan_sub = nh->subscribe(ops);
  
  while (ros::ok())
  {
    scan_queue.callAvailable(ros::WallDuration(0.1));
  }

}

void hw_publisher()
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");

  // configuring parameters
  std::string odom_topic;
  nh_priv->param<std::string>("odom_topic", odom_topic, "/odom");

  Common::Publisher hw_pub("tcp://*:6000");
  boost::function<void(const nav_msgs::Odometry::ConstPtr&)> cb = [&](const nav_msgs::Odometry::ConstPtr& msg) {
    json jobj;
    
    double v = msg->twist.twist.linear.x;
    double omega = msg->twist.twist.angular.z;

    double L = 0.4;
    double R = 0.125;
    double omega_r = -(2*v + omega*L)/(2*R);
    double omega_l = (2*v - omega*L)/(2*R);

    jobj["3"]["state"]["vel"] = omega_r;
    jobj["3"]["state"]["cur"] = 0.;
    jobj["3"]["state"]["tck"] = 0.;
    jobj["4"]["state"]["vel"] = omega_l;
    jobj["4"]["state"]["cur"] = 0.;
    jobj["4"]["state"]["tck"] = 0.;
    
    std::string jmsg = jobj.dump();
    hw_pub.send("PUB_HW", jmsg.c_str(), jmsg.size());
  };

  // define user callback queue
  ros::CallbackQueue odom_queue;
  
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
    ros::SubscribeOptions::create<nav_msgs::Odometry>(
      odom_topic, // topic name
      1, // queue length
      cb, // callback
      nullptr,
      &odom_queue // pointer to callback queue object
    );
  // subscribe
  ros::Subscriber odom_sub = nh->subscribe(ops);
  
  while (ros::ok())
  {
    odom_queue.callAvailable(ros::WallDuration(0.1));
  }

}

void set_velocity(ros::Publisher & cmd_vel_pub, double speed, double omega) 
{
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = speed;
  cmd_vel_msg.angular.z = omega;

  cmd_vel_pub.publish(cmd_vel_msg);
}

void hw_replier() 
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");

  // configuring parameters
  std::string cmd_vel_topic;
  nh_priv->param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");

  ros::Publisher cmd_vel_pub = nh->advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

  bool power_enabled = false;
  set_velocity(cmd_vel_pub, 0., 0.);

  Common::Replier hw_rep;
  hw_rep.register_callback([&](const std::string& received, __attribute__((unused)) void *data, std::string& tosend) 
  { 	
    json j_in, j_out;
    try
    {
			j_in = json::parse(received);

      j_out["ack"] = std::string("false");

      std::string cmd = j_in.at("cmd");
      if (cmd == "server_alive") 
      {
        j_out["ack"] = std::string("true"); 
      }
      else if (cmd == "move")
      {
        double speed = 0.0;
        double omega = 0.0;
        j_out["ack"] = std::string("true");

        if (power_enabled) 
        {
          try 
          {
            speed = j_in.at("speed");
          } 
          catch(std::exception& e)
          {
            j_out["ack"] = std::string("false");
            j_out["err"] = e.what();
          }
          try 
          {
            omega = j_in.at("omega");   
          } 
          catch(std::exception& e)
          {
            j_out["ack"] = std::string("false");
            j_out["err"] = e.what();
          }
        }

        set_velocity(cmd_vel_pub, speed, omega);
      }
      else if (cmd == "set_rear_current") 
      {
        j_out["ack"] = std::string("false");         
        j_out["err"] = "Unsupported operation";
      }
      else if (cmd == "set_device_mode")
      {
        j_out["ack"] = std::string("true"); 
      }
      else if (cmd == "set_power_enable")
      {
        bool enable = false;
        try
        {
          enable = j_in.at("enable");
          j_out["ack"] = std::string("true"); 
        }
        catch(std::exception& e)
        {
          j_out["ack"] = std::string("false");
          j_out["err"] = e.what();
        }

        power_enabled = enable;
        if (!enable) 
        {
          set_velocity(cmd_vel_pub, 0., 0.);
        }
      }
    } 
    catch (std::exception& e)
    {
      std::cerr << "Error while processing request: " << e.what() << std::endl;
      j_out["ack"] = std::string("false");
    }

		tosend = j_out.dump(); 
	}, nullptr);
  
  
  hw_rep.start("tcp://*:5601");

  ros::Rate rate(10);
  while (ros::ok())
  {
    rate.sleep();
  }

  hw_rep.stop();
}




void runNode(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_interface");
  
  // spawn threads
  boost::thread thread_pose(pose_publisher);
  boost::thread thread_scan(scan_publisher);
  boost::thread thread_hw_pub(hw_publisher);
  boost::thread thread_hw_rep(hw_replier);

  // wait until ros termination
  ros::spin();

  thread_pose.join();
  thread_scan.join();
  thread_hw_pub.join();
  thread_hw_rep.join();
}
