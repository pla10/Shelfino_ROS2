#include "shelfino_hw_publisher.hpp"

void ShelfinoHWNode::odom_timer_callback()
{
  geometry_msgs::msg::TransformStamped t;

  std::string origin_frame_ = this->ns+"/odom";
  std::string target_frame_ = this->ns+"/t265/pose_frame";

  try {
    t = tf_buffer_->lookupTransform(
      origin_frame_, target_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s",
      origin_frame_.c_str(), target_frame_.c_str(), ex.what());
    return;
  }

  // How it was changed
  // t.transform.translation.x = -pose.translation.z;
  // t.transform.translation.y = -pose.translation.x;
  // t.transform.translations.z = pose.translation.y;
  // t.transform.rotation.x = -pose.rotation.z;
  // t.transform.rotation.y = -pose.rotation.x;
  // t.transform.rotation.z = pose.rotation.y;
  // t.transform.rotation.w = pose.rotation.w;

  // t.transform.translation.z = -t.transform.translation.x;
  // t.transform.translation.x = -t.transform.translation.y;
  // t.transform.translation.y = t.transform.translation.z;
  // t.transform.rotation.z = -t.transform.rotation.x;
  // t.transform.rotation.x = -t.transform.rotation.y;
  // t.transform.rotation.y = t.transform.rotation.z;
  // t.transform.rotation.w = t.transform.rotation.w;

  // t.transform.translation.z = -t.transform.translation.z; 


  // The quaternion for the transformation
  tf2::Quaternion q(
    t.transform.rotation.x,
    t.transform.rotation.y,
    t.transform.rotation.z,
    t.transform.rotation.w);

  // The quaternion describing the roation
  tf2::Quaternion rotation_90_y;
  rotation_90_y.setRPY(0.0, 1.5708, 0.0);
  // The new quaternion once rotated
  tf2::Quaternion tmp_q = rotation_90_y * q;
  tmp_q.normalize();
  
  // Invert roll and yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);
  tmp_q.setRPY(yaw, pitch, roll);

  // The quaternion describing a yaw rotation
  tf2::Quaternion rotation_90_z;
  rotation_90_z.setRPY(0.0, 0.0, 0.0);
  // The new quaternion once rotated
  tf2::Quaternion new_q = rotation_90_z * tmp_q;
  new_q.normalize();

  RCLCPP_INFO(
    this->get_logger(), "Transform %s to %s\n[x: %f, y: %f, z: %f] (x: %f y: %f z: %f w: %f)", // <r: %f p: %f y: %f>",
    origin_frame_.c_str(), target_frame_.c_str(),
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
    // , roll, pitch, yaw
  );

  t.header.stamp = t.header.stamp;

  t.header.frame_id = origin_frame_;
  t.child_frame_id = ns+"/base_footprint";

  double tmp_x = t.transform.translation.x; 
  double tmp_y = t.transform.translation.y; 
  double tmp_z = t.transform.translation.z; 
  t.transform.translation.x = -tmp_y;
  t.transform.translation.y = tmp_x;
  // t.transform.translation.z = t.transform.translation.y;
  // t.transform.translation.y = tmp_z;



  // t.transform.translation.x = t.transform.translation.x + 0.15;
  // t.transform.translation.y += 0.15;
  // t.transform.translation.z = t.transform.translation.z + 0.15;
  t.transform.rotation.x = new_q.getX();
  t.transform.rotation.y = new_q.getY();
  t.transform.rotation.z = new_q.getZ();
  t.transform.rotation.w = new_q.getW();

  if (t.transform.rotation.x == 0 && 
      t.transform.rotation.y == 0 && 
      t.transform.rotation.z == 0 && 
      t.transform.rotation.w == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Quaternions are all set to 0");
    exit(1);
  }

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
  
}