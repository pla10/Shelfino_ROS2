#include <iostream>
#include <iomanip>
#include <atomic>
#include <future>
#include <thread>
#include <signal.h>
#include <cmath>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/Quaternion.h"

#include "ros_interface/Encoders.h"

using namespace std;

static const double LEFT_INCREMENTS_PER_TOUR = 294912.;
static const double RIGHT_INCREMENTS_PER_TOUR = 294912.;

// static const double LEFT_RADIUS = 0.114831;
// static const double RIGHT_RADIUS = 0.114892;
// static const double WHEEL_BASE = 0.392347;

static const double LEFT_RADIUS = 0.99 * 0.125 ; //0.9907*0.125; //0.125186;
static const double RIGHT_RADIUS = 0.99 * 0.125 ; //0.99*0.125; //0.125138;
static const double WHEEL_BASE = 0.383;

static const double LEFT_B_D = 4* WHEEL_BASE / (2.*LEFT_RADIUS); 
static const double RIGHT_B_D = 4* WHEEL_BASE / (2.*RIGHT_RADIUS); 

// static const double LEFT_B_D = 1.5764889187282987 * 1.0625; // * 1.0697 ; // * 1.08203;
// static const double RIGHT_B_D = 1.5784089237419516 * 1.0625; // * 1.0697 ; // * 1.08203;
static const double ENCODER_PPR = 36*4*2048;

static const double RATIO_LEFT_RIGHT = RIGHT_B_D/LEFT_B_D;
//static const double RADIUS_SCALING = 0.476; //0.45971498589878657;
//static const double LEFT_RADIUS = RADIUS_SCALING;
//static const double RIGHT_RADIUS = LEFT_RADIUS/RATIO_LEFT_RIGHT;

//static const double WHEEL_RADIUS = 0.125; //0.162; // 0.1291; //0.1288659793814433; //0.125;
//static const double WHEEL_BASE = 0.387; //0.96; //0.387;
//static const double WHEEL_RADIUS = 0.031;
//static const double WHEEL_BASE = 0.2975;

struct OdomData {
  double x, y, theta;
  double v, omega;
  double last_odom_update;
  std::mutex mtx;

  OdomData():
    //x(0.0), y(0.0), theta(0), v(0), omega(0), last_odom_update(0)
    x(35), y(15.4), theta(0.), v(0), omega(0)
    //x(340), y(-16.5), theta(M_PI), v(0), omega(0), last_odom_update(0)
  {}
} odomData;

struct WheelSpeed {
  double dTick;
  //double lastTick;
  double lastTime;
  bool init = false;
};

WheelSpeed wsLeft, wsRight;


struct EulerAngles {
  double roll, pitch, yaw;
};
EulerAngles ToEulerAngles(geometry_msgs::Quaternion const & q);

void runNode(int argc, char* argv[]);
void newEncodersData(const ros_interface::Encoders::ConstPtr& msg);
void newT265Data(const nav_msgs::Odometry::ConstPtr& msg);
void updateWheelsSpeed(double current_time, double vl, double vr);
void updateOdometryV(double current_time, double vl, double vr);
void updateOdometryOmega(double current_time, geometry_msgs::Quaternion const & orientation, geometry_msgs::Vector3 const & angular_vel);
void publishOdometry(ros::Publisher & publisher, tf::TransformBroadcaster & transform_broadcaster, const ros::Time & current_time, OdomData & odomData);


int main(int argc, char* argv[]) {
  runNode(argc, argv);
  return 0;
}




void runNode(int argc, char* argv[]) {
  ros::init(argc, argv, "odom_interface");

  ros::NodeHandlePtr nh_priv = boost::make_shared<ros::NodeHandle>("~");
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

  // configuring parameters
  std::string encoders_topic, odom_topic, odomPublisher, t265_topic;
  nh_priv->param<std::string>("encoders_topic", encoders_topic, "/encoders");
  nh_priv->param<std::string>("t265_topic", t265_topic, "/t265");
  nh_priv->param<std::string>("odom_topic", odom_topic, "/odom");

  tf::TransformBroadcaster transform_broadcaster;
  ros::Publisher publisher = nh->advertise<nav_msgs::Odometry>(odom_topic, 100);

  boost::function<void(const ros_interface::Encoders::ConstPtr&)> cbEnc = [&](const ros_interface::Encoders::ConstPtr& msg) {
    newEncodersData(msg);
  };

  boost::function<void(const nav_msgs::Odometry::ConstPtr&)> cbT265 = [&](const nav_msgs::Odometry::ConstPtr& msg) {
    newT265Data(msg);
    //publishOdometry(publisher, transform_broadcaster, msg->header.stamp, odomData);
    publishOdometry(publisher, transform_broadcaster, ros::Time::now(), odomData);
  };

  // define user callback queue
  ros::CallbackQueue c_queue;
  
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions enc_ops =
    ros::SubscribeOptions::create<ros_interface::Encoders>(
      encoders_topic, // topic name
      1, // queue length
      cbEnc, // callback
      nullptr,
      &c_queue // pointer to callback queue object
    );

  ros::SubscribeOptions t265_ops =
    ros::SubscribeOptions::create<nav_msgs::Odometry>(
      t265_topic, // topic name
      1, // queue length
      cbT265, // callback
      nullptr,
      &c_queue // pointer to callback queue object
    );
  // subscribe
  ros::Subscriber encoders_sub = nh->subscribe(enc_ops);
  ros::Subscriber t265_sub = nh->subscribe(t265_ops);
  
  while (ros::ok())
  {
    c_queue.callAvailable(ros::WallDuration(0.1));
  }
}

void newEncodersData(const ros_interface::Encoders::ConstPtr& msg) {
  double current_time = msg->time; 
  double vl = msg->leftVel;
  double vr = msg->rightVel;
  vr = -vr;

  updateOdometryV(current_time, vl, vr);
}

void newT265Data(const nav_msgs::Odometry::ConstPtr& msg) {
  double current_time = msg->header.stamp.toSec(); 
  geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
  geometry_msgs::Vector3 angular_vel = msg->twist.twist.angular;
  
  updateOdometryOmega(current_time, orientation, angular_vel);
}

void updateWheelsSpeed(double current_time, double vl, double vr) {
  if (!wsLeft.init) {
    wsLeft.dTick = 0;
    //wsLeft.lastTick = pL;
    wsLeft.lastTime = current_time;
    wsLeft.init = true;
  }
  else {
    //wsLeft.dTick = (vl*36/(2*M_PI)*2048)*(current_time-wsLeft.lastTime); //pL-wsLeft.lastTick;
    wsLeft.dTick = (vl/(2*M_PI)*ENCODER_PPR)*(current_time-wsLeft.lastTime); //pL-wsLeft.lastTick;
    //wsLeft.lastTick = pL;
    wsLeft.lastTime = current_time;
  }

  if (!wsRight.init) {
    wsRight.dTick = 0;
    //wsRight.lastTick = pR;
    wsRight.lastTime = current_time;
    wsRight.init = true;
  }
  else {
    //wsRight.dTick = (vr*36/(2*M_PI)*2048)*(current_time-wsRight.lastTime); //pR-wsRight.lastTick;
    wsRight.dTick = (vr/(2*M_PI)*ENCODER_PPR)*(current_time-wsRight.lastTime); //pR-wsRight.lastTick;
    //wsRight.lastTick = pR;
    wsRight.lastTime = current_time;
  }
}


void updateOdometryV(double current_time, double vl, double vr) {
  bool firstTime = !wsLeft.init;
  double seconds_since_last_update = current_time - wsLeft.lastTime;
  updateWheelsSpeed(current_time, vl, vr);
  
  if (firstTime) return;
  
  double ticks_l = wsLeft.dTick;
  double ticks_r = wsRight.dTick;
  double df = 0.5 * (ticks_l/LEFT_INCREMENTS_PER_TOUR * 2*M_PI * LEFT_RADIUS + ticks_r/RIGHT_INCREMENTS_PER_TOUR * 2*M_PI * RIGHT_RADIUS);

  
  //double seconds_since_last_update = current_time - odomData.last_odom_update;
  //odomData.last_odom_update = current_time;

  std::unique_lock<std::mutex> lock(odomData.mtx);
  odomData.v = df/seconds_since_last_update;
  //double dtheta = odomData.omega*seconds_since_last_update;

  // double dx, dy;
  // double theta_k, theta_k1;
  // theta_k = odomData.theta;
  // theta_k1 = odomData.theta + dtheta;
  // if (std::abs(odomData.omega)>1e-6) {
  //   dx = odomData.v/odomData.omega*(std::sin(theta_k1)-std::sin(theta_k));
  //   dy = -odomData.v/odomData.omega*(std::cos(theta_k1)-std::cos(theta_k));
  // }
  // else {
  //   dx = df*std::cos(theta_k + 0.5*dtheta);
  //   dy = df*std::sin(theta_k + 0.5*dtheta);
  // }

  // odomData.x += dx;
  // odomData.y += dy;
  // odomData.theta += dtheta;
}

void updateOdometryOmega(double current_time, geometry_msgs::Quaternion const & orientation, geometry_msgs::Vector3 const & angular_vel) {
  std::unique_lock<std::mutex> lock(odomData.mtx);
  double seconds_since_last_update = current_time - odomData.last_odom_update;
  odomData.last_odom_update = current_time;

  EulerAngles angles = ToEulerAngles(orientation);  
  angles.yaw = -angles.yaw; // flip sign
  double dtheta = angles.yaw - odomData.theta;
  double df = odomData.v * seconds_since_last_update; 
  if (dtheta != 0)
    dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta)); // angdiff?
  odomData.omega = dtheta/seconds_since_last_update;

  double dx, dy;
  double theta_k, theta_k1;
  theta_k = odomData.theta;
  theta_k1 = odomData.theta + dtheta;
  if (std::abs(odomData.omega)>1e-6) {
    dx = odomData.v/odomData.omega*(std::sin(theta_k1)-std::sin(theta_k));
    dy = -odomData.v/odomData.omega*(std::cos(theta_k1)-std::cos(theta_k));
  }
  else {
    dx = df*std::cos(theta_k + 0.5*dtheta);
    dy = df*std::sin(theta_k + 0.5*dtheta);
  }

  odomData.x += dx;
  odomData.y += dy;
  odomData.theta += dtheta;
}


void publishOdometry(ros::Publisher & publisher,  tf::TransformBroadcaster & transform_broadcaster, const ros::Time & current_time, OdomData & odomData) {
  static int64_t id = 1;

  std::string odom_frame = "odom";
  std::string base_link_frame = "base_link";


  double x = odomData.x;
  double y = odomData.y;
  double theta = odomData.theta;
  double v = odomData.v;
  double omega = odomData.omega;
  std::vector<double> cov = std::vector<double>(25, 0.01); // j.at("covariance");

  tf::Quaternion qt;
  tf::Vector3 vt;
  qt.setRPY(0., 0., theta);
  vt = tf::Vector3(x, y, 0.);

  nav_msgs::Odometry msg;
  msg.header.seq = id++;
  msg.header.stamp = current_time;
  msg.header.frame_id = odom_frame;
  
  msg.child_frame_id = base_link_frame;

  msg.pose.pose.position.x = vt.x();
  msg.pose.pose.position.y = vt.y();
  msg.pose.pose.position.z = vt.z();

  msg.pose.pose.orientation.x = qt.x();
  msg.pose.pose.orientation.y = qt.y();
  msg.pose.pose.orientation.z = qt.z();
  msg.pose.pose.orientation.w = qt.w();

  msg.pose.covariance[0] = cov[0]; // cov_x
  msg.pose.covariance[7] = cov[6]; // cov_y
  msg.pose.covariance[14] = 1000000000000.0;
  msg.pose.covariance[21] = 1000000000000.0;
  msg.pose.covariance[28] = 1000000000000.0;
  msg.pose.covariance[35] = cov[12]; // cov_theta
  
  msg.twist.twist.linear.x = v;
  msg.twist.twist.angular.z = omega;
  msg.twist.covariance[0] = cov[18]; // cov_v_x
  msg.twist.covariance[7] = cov[18]; // cov_v_y
  msg.twist.covariance[14] = 1000000000000.0;
  msg.twist.covariance[21] = 1000000000000.0;
  msg.twist.covariance[28] = 1000000000000.0;
  msg.twist.covariance[35] = cov[24]; // cov_omega

  tf::Transform base_link_to_odom(qt, vt);
  transform_broadcaster.sendTransform(tf::StampedTransform(base_link_to_odom, current_time,
    odom_frame, base_link_frame));

  publisher.publish(msg);
}


EulerAngles ToEulerAngles(geometry_msgs::Quaternion const & q) {
  EulerAngles angles;

  double test = q.x*q.y + q.z*q.w;
  if (test > 0.499) { // singularity at north pole
    angles.roll = 2 * std::atan2(q.x,q.w);
    angles.pitch = M_PI/2;
    angles.yaw = 0;
    return angles;
  }
  if (test < -0.499) { // singularity at south pole
    angles.roll = -2 * std::atan2(q.x,q.w);
    angles.pitch = - M_PI/2;
    angles.yaw = 0;
    return angles;
  }
  double sqx = q.x*q.x;
  double sqy = q.y*q.y;
  double sqz = q.z*q.z;
  angles.roll = std::atan2(2*q.y*q.w-2*q.x*q.z , 1 - 2*sqy - 2*sqz);
  angles.pitch = std::asin(2*test);
  angles.yaw = std::atan2(2*q.x*q.w-2*q.y*q.z , 1 - 2*sqx - 2*sqz);

  return angles;
}
