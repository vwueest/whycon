#include "vicon_publisher.h"
#include <angles/angles.h>

whycon::ViconPublisher::ViconPublisher(ros::NodeHandle &n) {
  n.param("cable_length", cable_length_, 0.3);
  n.param("distance_tag_CoG", distance_tag_CoG_, 0.0);
  n.param("transform_to_world_frame", transform_to_world_frame, false);
  n.param("publish_observ_dir", publish_observ_dir, true);

  std::string vicon_quad_topic, vicon_payload_topic;
  n.param("vicon_quad_topic", vicon_quad_topic, std::string(""));
  n.param("vicon_payload_topic", vicon_payload_topic, std::string(""));

  odom_vicon_pub = n.advertise<payload_msgs::PayloadOdom>("odom_pointload_vicon", 1);
  relative_pos_pub = n.advertise<geometry_msgs::Vector3Stamped>("relative_pos", 1);
}

void whycon::ViconPublisher::vicon_callback(const nav_msgs::OdometryConstPtr& msg_quad, const nav_msgs::OdometryConstPtr& msg_payload) {
  time_new_vicon_payload_ = msg_payload->header.stamp.sec + msg_payload->header.stamp.nsec * 1e-9;
  time_new_vicon_quad_ = msg_quad->header.stamp.sec + msg_quad->header.stamp.nsec * 1e-9;

//  if (std::abs(time_new_vicon_payload_ - time_new_vicon_quad_) > 0.5/150.0) {
//    //ROS_INFO("too big");
//    return;
//  }

  //ROS_INFO("timeDiff Vicon: %f", std::abs(time_new_vicon_payload_ - time_new_vicon_quad_));

  vicon_payload_pos_ = {msg_payload->pose.pose.position.x,
                        msg_payload->pose.pose.position.y,
                        msg_payload->pose.pose.position.z};
  vicon_payload_vel_ = {msg_payload->twist.twist.linear.x,
                        msg_payload->twist.twist.linear.y,
                        msg_payload->twist.twist.linear.z};

  R_WB_ = {1 - 2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.y -
               2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.z,
               2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.y -
               2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.w,
               2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.z +
               2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.w,
               2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.y +
               2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.w,
           1 - 2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.x -
               2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.z,
               2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.z -
               2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.w,
               2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.z -
               2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.w,
               2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.z +
               2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.w,
           1 - 2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.x -
               2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.y};

  vicon_quad_pos_    = {msg_quad->pose.pose.position.x,
                        msg_quad->pose.pose.position.y,
                        msg_quad->pose.pose.position.z};
  vicon_quad_angle_  = {msg_quad->pose.pose.orientation.w,
                        msg_quad->pose.pose.orientation.x,
                        msg_quad->pose.pose.orientation.y,
                        msg_quad->pose.pose.orientation.z};
  vicon_quad_vel_    = {msg_quad->twist.twist.linear.x,
                        msg_quad->twist.twist.linear.y,
                        msg_quad->twist.twist.linear.z};
  vicon_quad_angVel_ = {msg_quad->twist.twist.angular.x,
                        msg_quad->twist.twist.angular.y,
                        msg_quad->twist.twist.angular.z};

  if (std::abs(time_new_vicon_payload_ - time_new_vicon_quad_) < 1.0) {// (1.0/150.0)/2.0) {
    vicon_publish_msg(msg_quad->header);
  }
}

void whycon::ViconPublisher::vicon_publish_msg(const std_msgs::Header_<std::allocator<void>> &header) {
  if (publish_observ_dir) {
    // calculate body frame observation direction and normalize
    cv::Vec3d relative_pos_bodyFrame = R_WB_.t() * (vicon_payload_pos_ - vicon_quad_pos_);
    //relative_pos_bodyFrame = relative_pos_bodyFrame/cv::norm(relative_pos_bodyFrame);

    // create message and publish
    geometry_msgs::Vector3Stamped rel_pos_vec;
    rel_pos_vec.header = header;
    rel_pos_vec.vector.x = relative_pos_bodyFrame(0);
    rel_pos_vec.vector.y = relative_pos_bodyFrame(1);
    rel_pos_vec.vector.z = relative_pos_bodyFrame(2);
    rel_pos_vec.header.frame_id = std::string("0");
    relative_pos_pub.publish(rel_pos_vec);
  }
  //ROS_INFO("publish");
  if (transform_to_world_frame) // in world frame
    relative_pos_outputFrame = vicon_payload_pos_ - vicon_quad_pos_;
  else // in body frame
    relative_pos_outputFrame = R_WB_.t() * (vicon_payload_pos_ - vicon_quad_pos_);

  relative_pos_outputFrame = relative_pos_outputFrame *
                             (cable_length_ + distance_tag_CoG_) / cv::norm(relative_pos_outputFrame);

  // calculate relative velocity
  if (transform_to_world_frame) // in world frame
    relative_vel_outputFrame = vicon_payload_vel_ - vicon_quad_vel_;
  else // in body frame
    relative_vel_outputFrame = (R_WB_.t() * (vicon_payload_vel_ - vicon_quad_vel_)) + \
                               (R_WB_.t() * (vicon_payload_pos_ - vicon_quad_pos_).cross(vicon_quad_angVel_));

  // calculate relative angular velocity
  relative_ang_vel = (vicon_payload_pos_ - vicon_quad_pos_).cross(vicon_payload_vel_ - vicon_quad_vel_) /
                     (vicon_payload_pos_ - vicon_quad_pos_).dot(  vicon_payload_pos_ - vicon_quad_pos_);
  if (!transform_to_world_frame)
    relative_ang_vel = R_WB_.t() * (relative_ang_vel - vicon_quad_angVel_);

  // fill in data
  payload_msgs::PayloadOdom payload_vicon;

  payload_vicon.pose_payload.pose.position.x = relative_pos_outputFrame(0)+vicon_quad_pos_(0);
  payload_vicon.pose_payload.pose.position.y = relative_pos_outputFrame(1)+vicon_quad_pos_(1);
  payload_vicon.pose_payload.pose.position.z = relative_pos_outputFrame(2)+vicon_quad_pos_(2);

  payload_vicon.pose_payload.pose.orientation.x =  atan2(relative_pos_outputFrame(1), -relative_pos_outputFrame(2));
  payload_vicon.pose_payload.pose.orientation.y = -atan2(relative_pos_outputFrame(0), -relative_pos_outputFrame(2));
  payload_vicon.pose_payload.pose.orientation.z = 0;
  payload_vicon.pose_payload.pose.orientation.w = 0;

  payload_vicon.twist_payload.twist.linear.x = vicon_payload_vel_(0);
  payload_vicon.twist_payload.twist.linear.y = vicon_payload_vel_(1);
  payload_vicon.twist_payload.twist.linear.z = vicon_payload_vel_(2);

  payload_vicon.twist_payload.twist.angular.x = relative_ang_vel(0);
  payload_vicon.twist_payload.twist.angular.y = relative_ang_vel(1);
  payload_vicon.twist_payload.twist.angular.z = 0;

  payload_vicon.pose_quad.pose.position.x = vicon_quad_pos_(0);
  payload_vicon.pose_quad.pose.position.y = vicon_quad_pos_(1);
  payload_vicon.pose_quad.pose.position.z = vicon_quad_pos_(2);

  payload_vicon.pose_quad.pose.orientation.w = vicon_quad_angle_(0);
  payload_vicon.pose_quad.pose.orientation.x = vicon_quad_angle_(1);
  payload_vicon.pose_quad.pose.orientation.y = vicon_quad_angle_(2);
  payload_vicon.pose_quad.pose.orientation.z = vicon_quad_angle_(3);

  payload_vicon.twist_quad.twist.linear.x = vicon_quad_vel_(0);
  payload_vicon.twist_quad.twist.linear.y = vicon_quad_vel_(1);
  payload_vicon.twist_quad.twist.linear.z = vicon_quad_vel_(2);

  payload_vicon.twist_quad.twist.angular.x = vicon_quad_angVel_(0);
  payload_vicon.twist_quad.twist.angular.y = vicon_quad_angVel_(1);
  payload_vicon.twist_quad.twist.angular.z = vicon_quad_angVel_(2);

  payload_vicon.header = header;
  odom_vicon_pub.publish(payload_vicon);
}


