#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "ros_interface/Encoders.h"
#include <tf/transform_broadcaster.h>
#include <ros/callback_queue.h>

#include "Publisher.hpp"
#include "Subscriber.hpp"
#include "Replier.hpp"
#include "json.hpp"

#include <boost/thread/thread.hpp>

#include "hardwareparameters.h"
#include "hardwareglobalinterface.h"


using namespace nlohmann;

//const auto R_ID_DEFAULT = "localhost";
const auto R_ID_DEFAULT = 1;
std::unique_ptr<HardwareParameters> hp;


void runNode(int argc, char* argv[]);


int main(int argc, char* argv[])
{
  runNode(argc, argv);
  return 0;
}



void scan_publisher()
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");


  // configuring parameters
  std::string scan_topic, frontLidarPublisher;
  nh_priv->param<std::string>("scan_topic", scan_topic, "/scan");
  nh_priv->param<std::string>("front_lidar_publisher", frontLidarPublisher, hp->frontLidarPublisher);

  ros::Publisher publisher = nh->advertise<sensor_msgs::LaserScan>(scan_topic, 0);
  ZMQCommon::Subscriber subFrontLidar;

  int id = 1;
  subFrontLidar.register_callback([&](const char *topic, const char *buf, size_t size, void *data) {
    nlohmann::json j;

    try {
      j = nlohmann::json::parse(std::string(buf, size));
      int size = j.at("size");
      std::vector<float> data = j.at("data");

      sensor_msgs::LaserScan msg;
      msg.header.seq = id++;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "base_laser";
      msg.angle_increment = 0.00872664625;
      msg.angle_min = msg.angle_increment;
      msg.angle_max = 6.27445866092 + msg.angle_min;
      msg.time_increment = (1./10.) / size;
      msg.scan_time = 1./10.;
      msg.range_min = 0.05;
      msg.range_max = 10;
      msg.ranges = data;
      //msg.intensities = std::vector<float>(size, 0.);

      publisher.publish(msg);
    }
    catch(std::exception &e){
      //std::cerr << "error parsing: " << e.what() << std::endl;
    }
  });
  subFrontLidar.start(frontLidarPublisher, "LIDAR");

  ros::Rate rate(10);
  
  while (ros::ok())
  {
    rate.sleep();
  }

  subFrontLidar.stop();

}

void hw_publisher()
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");

  // configuring parameters
//  std::string odom_topic, odomAddress;
  std::string encoders_topic, encodersAddress;
//  nh_priv->param<std::string>("odom_topic", odom_topic, "/odom");
//  nh_priv->param<std::string>("odomAddress", odomAddress, hp->localizationPublisher);
  nh_priv->param<std::string>("encoders_topic", encoders_topic, "/encoders");
  nh_priv->param<std::string>("encodersAddress", encodersAddress, hp->hardwarePublisher);

//  tf::TransformBroadcaster transform_broadcaster;
//  ros::Publisher odomPublisher = nh->advertise<nav_msgs::Odometry>(odom_topic, 0);
  ros::Publisher encodersPublisher = nh->advertise<ros_interface::Encoders>(encoders_topic, 0);
//  ZMQCommon::Subscriber subOdom;
  ZMQCommon::Subscriber subEncoders;

//  std::string odom_frame = "odom";
//  std::string base_link_frame = "base_link";

//  int id = 1;
//  subOdom.register_callback([&](const char *topic, const char *buf, size_t size, void *data) {
//    ros::Time current_time = ros::Time::now();

//    nlohmann::json j;

//    try {
//      j = nlohmann::json::parse(std::string(buf, size));

//      double x = j.at("state").at("x");
//      double y = j.at("state").at("y");
//      double theta = j.at("state").at("theta");
//      double v = j.at("state").at("v");
//      double omega = j.at("state").at("omega");
//      //double time = j.at("time");
      
//      //ros::Time current_time == ros::Time::now();
//      //current_time.fromSec(time);
//      std::vector<double> cov = std::vector<double>(25, 0.01); // j.at("covariance");

//      tf::Quaternion qt;
//      tf::Vector3 vt;
//      qt.setRPY(0., 0., theta);
//      vt = tf::Vector3(x, y, 0.);

//      nav_msgs::Odometry msg;
//      msg.header.seq = id++;
//      msg.header.stamp = current_time;
//      msg.header.frame_id = odom_frame;
      
//      msg.child_frame_id = base_link_frame;

//      msg.pose.pose.position.x = vt.x();
//      msg.pose.pose.position.y = vt.y();
//      msg.pose.pose.position.z = vt.z();

//      msg.pose.pose.orientation.x = qt.x();
//      msg.pose.pose.orientation.y = qt.y();
//      msg.pose.pose.orientation.z = qt.z();
//      msg.pose.pose.orientation.w = qt.w();

//      msg.pose.covariance[0] = cov[0]; // cov_x
//      msg.pose.covariance[7] = cov[6]; // cov_y
//      msg.pose.covariance[14] = 1000000000000.0;
//      msg.pose.covariance[21] = 1000000000000.0;
//      msg.pose.covariance[28] = 1000000000000.0;
//      msg.pose.covariance[35] = cov[12]; // cov_theta
      
//      msg.twist.twist.linear.x = v;
//      msg.twist.twist.angular.z = omega;
//      msg.twist.covariance[0] = cov[18]; // cov_v_x
//      msg.twist.covariance[7] = cov[18]; // cov_v_y
//      msg.twist.covariance[14] = 1000000000000.0;
//      msg.twist.covariance[21] = 1000000000000.0;
//      msg.twist.covariance[28] = 1000000000000.0;
//      msg.twist.covariance[35] = cov[24]; // cov_omega

//      tf::Transform base_link_to_odom(qt, vt);
//      transform_broadcaster.sendTransform(tf::StampedTransform(base_link_to_odom, current_time,
//        odom_frame, base_link_frame));

//      odomPublisher.publish(msg);
//    }
//    catch(std::exception &e){
//      std::cerr << "error parsing: " << e.what() << std::endl;
//    }
//  });
//  subOdom.start(odomAddress, "LOC");

  subEncoders.register_callback([&](const char *topic, const char *buf, size_t size, void *data) {
    ros::Time current_time = ros::Time::now();

    nlohmann::json j;

    try {
      j = nlohmann::json::parse(std::string(buf, size));

      double vl = j.at("4").at("state").at("vel");
      double vr = j.at("3").at("state").at("vel");
      
      double tl = j.at("4").at("state").at("tck");
      double tr = j.at("3").at("state").at("tck");
      
      double time = j.at("time"); 
      time /= 1000.; // convert from ms to s

      //ros::Time current_time;
      //current_time.fromSec(time);
      std::vector<double> cov = std::vector<double>(25, 0.01); // j.at("covariance");

      ros_interface::Encoders msg;
      msg.stamp = current_time;
    
      msg.time = time;
      msg.leftTicks = tl;
      msg.rightTicks = tr;
      msg.leftVel = vl;
      msg.rightVel = vr;
      
      encodersPublisher.publish(msg);
    }
    catch(std::exception &e){
      std::cerr << "error parsing: " << e.what() << std::endl;
    }
  });
  subEncoders.start(encodersAddress, "PUB_HW");

  ros::Rate rate(10);
  
  while (ros::ok())
  {
    rate.sleep();
  }

//  subOdom.stop();
  subEncoders.stop();
}

void t265_publisher()
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");

  // configuring parameters
  std::string t265_topic, t265Address;
  nh_priv->param<std::string>("t265_topic", t265_topic, "/t265");
  nh_priv->param<std::string>("t265Address", t265Address, "tcp://127.0.0.1:9304");

  ros::Publisher t265Publisher = nh->advertise<nav_msgs::Odometry>(t265_topic, 0);
//  ZMQCommon::Subscriber subOdom;
  ZMQCommon::Subscriber subT265;

  int id = 1;
  subT265.register_callback([&](const char *topic, const char *buf, size_t size, void *data) {
    ros::Time current_time = ros::Time::now();

    nlohmann::json j;

    try {
      j = nlohmann::json::parse(std::string(buf, size));
 
      nav_msgs::Odometry msg;
      msg.header.seq = id++;
      msg.header.stamp = ros::Time(j["time"].get<double>());
      msg.header.frame_id = "odom";
      
      msg.child_frame_id = "base_link";

      msg.pose.pose.position.x = j["pose"]["pose"]["position"]["x"]; 
      msg.pose.pose.position.y = j["pose"]["pose"]["position"]["y"];
      msg.pose.pose.position.z = j["pose"]["pose"]["position"]["z"];

      msg.pose.pose.orientation.x = j["pose"]["pose"]["orientation"]["x"];
      msg.pose.pose.orientation.y = j["pose"]["pose"]["orientation"]["y"];
      msg.pose.pose.orientation.z = j["pose"]["pose"]["orientation"]["z"];
      msg.pose.pose.orientation.w = j["pose"]["pose"]["orientation"]["w"];

      for (int i=0; i<j["pose"]["covariance"].size(); ++i) {
        msg.pose.covariance[i] = j["pose"]["covariance"][i];
      }
      
      
      msg.twist.twist.linear.x = j["twist"]["twist"]["linear"]["x"];
      msg.twist.twist.linear.y = j["twist"]["twist"]["linear"]["y"];
      msg.twist.twist.linear.z = j["twist"]["twist"]["linear"]["z"];
      msg.twist.twist.angular.x = j["twist"]["twist"]["angular"]["x"];
      msg.twist.twist.angular.y = j["twist"]["twist"]["angular"]["y"];
      msg.twist.twist.angular.z = j["twist"]["twist"]["angular"]["z"];
     
      for (int i=0; i<j["twist"]["covariance"].size(); ++i) {
        msg.twist.covariance[i] = j["twist"]["covariance"][i];
      }
     
      t265Publisher.publish(msg);
    }
    catch(std::exception &e){
      std::cerr << "error parsing: " << e.what() << std::endl;
    }
  });
  subT265.start(t265Address, "ODOM");

  
  ros::Rate rate(10);
  
  while (ros::ok())
  {
    rate.sleep();
  }

  subT265.stop();
}



/*void hw_cmd() 
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");


  // configuring parameters
  std::string cmd_vel_topic;
  nh_priv->param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");

  // start robot 
  HardwareGlobalInterface::getInstance().robotOnVelControl();

  boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb = [&](const geometry_msgs::Twist::ConstPtr& msg) {
    HardwareGlobalInterface::getInstance().vehicleMove(msg->linear.x, msg->angular.z);
  };


  // define user callback queue
  ros::CallbackQueue cmd_vel_queue;
  
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
      cmd_vel_topic, // topic name
      1, // queue length
      cb, // callback
      nullptr,
      &cmd_vel_queue // pointer to callback queue object
    );

  // subscribe
  ros::Subscriber cmd_vel_sub = nh->subscribe(ops);
  
  while (ros::ok())
  {
    cmd_vel_queue.callAvailable(ros::WallDuration(0.1));
  }

  HardwareGlobalInterface::getInstance().robotOff();

}*/




void runNode(int argc, char* argv[])
{
  ros::init(argc, argv, "hw_interface");

  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");

  std::string rId_string;
  int rId_int;
  if (nh_priv->getParam("robot_id", rId_string)) {
    hp = std::make_unique<HardwareParameters>(rId_string);
  }
  else if (nh_priv->getParam("robot_id", rId_int)) {
    hp = std::make_unique<HardwareParameters>(rId_int);
  }
  else {
    hp = std::make_unique<HardwareParameters>(R_ID_DEFAULT);
  }
  HardwareGlobalInterface::initialize(hp.get());


  // spawn threads
  boost::thread thread_scan(scan_publisher);
  boost::thread thread_hw_pub(hw_publisher);
  boost::thread thread_t265_pub(t265_publisher);
  //boost::thread thread_hw_cmd(hw_cmd);

  // wait until ros termination
  ros::spin();

  thread_scan.join();
  thread_hw_pub.join();
  thread_t265_pub.join();
  //thread_hw_cmd.join();
}
