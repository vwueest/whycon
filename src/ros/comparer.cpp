#include "comparer.h"
#include <angles/angles.h>

whycon::Comparer::Comparer(ros::NodeHandle &n) {
  p_error_pub = n.advertise<geometry_msgs::Vector3>("p_error", 1);
  load_pos_error_pub = n.advertise<geometry_msgs::Vector3>("load_pos_error_pub", 1);
  load_vel_error_pub = n.advertise<geometry_msgs::Vector3>("load_vel_error_pub", 1);
  angles_error_pub = n.advertise<geometry_msgs::Vector3>("angle_error_pub", 1);
  angVel_error_pub = n.advertise<geometry_msgs::Vector3>("angVel_error_pub", 1);

  n.param("dt_max", dt_max_, 0.3);
}

void whycon::Comparer::synced_callback(const payload_msgs::PayloadOdomConstPtr& msg_vicon, const payload_msgs::PayloadOdomConstPtr& msg_estimate) {
//  ROS_INFO("test callback");

  if ( std::abs( (msg_estimate->header.stamp - msg_vicon->header.stamp ).toSec() ) < dt_max_ ) {

    Eigen::Vector3d vicon_quad_pos(msg_vicon->pose_quad.pose.position.x, msg_vicon->pose_quad.pose.position.y, msg_vicon->pose_quad.pose.position.z);
    Eigen::Vector3d estimate_quad_pos(msg_estimate->pose_quad.pose.position.x, msg_estimate->pose_quad.pose.position.y, msg_estimate->pose_quad.pose.position.z);

    Eigen::Vector3d vicon_payload_pos(msg_vicon->pose_payload.pose.position.x, msg_vicon->pose_payload.pose.position.y, msg_vicon->pose_payload.pose.position.z);
    Eigen::Vector3d estimate_payload_pos(msg_estimate->pose_payload.pose.position.x, msg_estimate->pose_payload.pose.position.y, msg_estimate->pose_payload.pose.position.z);

    Eigen::Vector3d vicon_payload_vel(msg_vicon->twist_payload.twist.linear.x, msg_vicon->twist_payload.twist.linear.y, msg_vicon->twist_payload.twist.linear.z);
    Eigen::Vector3d estimate_payload_vel(msg_estimate->twist_payload.twist.linear.x, msg_estimate->twist_payload.twist.linear.y, msg_estimate->twist_payload.twist.linear.z);

    Eigen::Vector3d vicon_p = (vicon_payload_pos - vicon_quad_pos).normalized();
    Eigen::Vector3d estimate_p = (estimate_payload_pos - estimate_quad_pos).normalized();

    Eigen::Vector2d vicon_angles(msg_vicon->pose_payload.pose.orientation.x, msg_vicon->pose_payload.pose.orientation.y);
    Eigen::Vector2d estimate_angles(msg_estimate->pose_payload.pose.orientation.x, msg_estimate->pose_payload.pose.orientation.y);

    Eigen::Vector3d vicon_angVels(msg_vicon->twist_payload.twist.angular.x, msg_vicon->twist_payload.twist.angular.y, msg_vicon->twist_payload.twist.angular.z);
    Eigen::Vector3d estimate_angVels(msg_estimate->twist_payload.twist.angular.x, msg_estimate->twist_payload.twist.angular.y, msg_estimate->twist_payload.twist.angular.z);

    Eigen::Vector3d p_error = estimate_p - vicon_p;
    Eigen::Vector3d load_pos_error = estimate_payload_pos - vicon_payload_pos;
    Eigen::Vector3d load_vel_error = estimate_payload_vel - vicon_payload_vel;
    Eigen::Vector2d angles_error = estimate_angles - vicon_angles;
    Eigen::Vector3d angVel_error = estimate_angVels - vicon_angVels;



    geometry_msgs::Vector3 p_error_msg;
    p_error_msg.x = p_error(0);
    p_error_msg.y = p_error(1);
    p_error_msg.z = p_error(2);
    p_error_pub.publish(p_error_msg);

    geometry_msgs::Vector3 load_pos_error_msg;
    load_pos_error_msg.x = load_pos_error(0);
    load_pos_error_msg.y = load_pos_error(1);
    load_pos_error_msg.z = load_pos_error(2);
    load_pos_error_pub.publish(load_pos_error_msg);

    geometry_msgs::Vector3 load_vel_error_msg;
    load_vel_error_msg.x = load_vel_error(0);
    load_vel_error_msg.y = load_vel_error(1);
    load_vel_error_msg.z = load_vel_error(2);
    load_vel_error_pub.publish(load_vel_error_msg);

    geometry_msgs::Vector3 angles_error_msg;
    angles_error_msg.x = angles_error(0);
    angles_error_msg.y = angles_error(1);
    angles_error_msg.z = 0.0f;
    angles_error_pub.publish(angles_error_msg);

    geometry_msgs::Vector3 angVel_error_msg;
    angVel_error_msg.x = angVel_error(0);
    angVel_error_msg.y = angVel_error(1);
    angVel_error_msg.z = angVel_error(2);
    angVel_error_pub.publish(angVel_error_msg);

  }
}
