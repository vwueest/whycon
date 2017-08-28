#include "vicon_publisher.h"
#include <angles/angles.h>

whycon::ViconPublisher::ViconPublisher(ros::NodeHandle &n) {
  n.param("cable_length", cable_length_, 0.3);
  n.param("distance_tag_CoG", distance_tag_CoG_, 0.0);
  n.param("transform_to_world_frame", transform_to_world_frame, false);
  n.param("publish_observ_dir", publish_observ_dir, true);
  n.param("filter_velocities", filter_velocities, false);

  odom_vicon_pub = n.advertise<payload_msgs::PayloadOdom>("odom_payload_vicon", 1);
  relative_pos_pub = n.advertise<geometry_msgs::Vector3Stamped>("relative_pos", 1);
  vicon_pos_body_pub = n.advertise<geometry_msgs::Vector3Stamped>("vicon_pos_body", 1);
  vicon_vel_body_pub = n.advertise<geometry_msgs::Vector3Stamped>("vicon_vel_body", 1);

  meas_error_pos_pub = n.advertise<geometry_msgs::Vector3Stamped>("meas_error_pos", 1);
  meas_error_vel_pub = n.advertise<geometry_msgs::Vector3Stamped>("meas_error_vel", 1);
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

//  cv::Vec3d relative_pos_bodyFrame = R_WB_.t() * (vicon_payload_pos_ - vicon_quad_pos_);
//  relative_pos_bodyFrame *= cable_length_/cv::norm(relative_pos_bodyFrame);

//  cv::Vec3d p = (vicon_payload_pos_ - vicon_quad_pos_)/cv::norm(vicon_payload_pos_ - vicon_quad_pos_);
//  cv::Vec3d lpdot = vicon_payload_vel_ - vicon_quad_vel_;
//  cv::Vec3d velBodyFrame = lpdot - (cable_length_*R_WB_*vicon_quad_angVel_).cross(R_WB_.t()*p);
////  cv::Vec3d velBodyFrame = lpdot - (cable_length_*vicon_quad_angVel_).cross(R_WB_.t()*p);
////  cv::Vec3d velBodyFrame = lpdot.cross(R_WB_*vicon_quad_angVel_);
////  cv::Vec3d velBodyFrame = lpdot.cross(vicon_quad_angVel_);

//  cv::Vec3d velBodyFrameMeas = (relative_pos_bodyFrame - relative_pos_bodyFrame_old_)/((msg_quad->header.stamp - t_old_).toSec());
//  relative_pos_bodyFrame_old_ = relative_pos_bodyFrame;
//  t_old_ = msg_quad->header.stamp;
//  vel_error_ += cv::norm(velBodyFrameMeas - velBodyFrame);

//  std::cout << "error vel: " << (velBodyFrameMeas - velBodyFrame).t() << std::endl;
//  std::cout << "sum error: " << vel_error_ << std::endl;
  //  std::cout << "time:      " << msg_quad->header.stamp << std::endl;
}

void whycon::ViconPublisher::synced_pos_callback(const geometry_msgs::Vector3StampedConstPtr &msg_vicon, const geometry_msgs::Vector3StampedConstPtr &msg_cam)
{
  bool publish_meas_error_pos = (meas_error_pos_pub.getNumSubscribers() != 0);
  if (publish_meas_error_pos) {
    cv::Vec3d vicon(msg_vicon->vector.x,msg_vicon->vector.y,msg_vicon->vector.z);
    cv::Vec3d cam(msg_cam->vector.x,msg_cam->vector.y,msg_cam->vector.z);
    cv::Vec3d error = cam - vicon;
    geometry_msgs::Vector3Stamped msg_error;
    msg_error.header = msg_cam->header;
    msg_error.vector.x = error(0);
    msg_error.vector.y = error(1);
    msg_error.vector.z = error(2);
    meas_error_pos_pub.publish(msg_error);
  }
}

void whycon::ViconPublisher::synced_vel_callback(const geometry_msgs::Vector3StampedConstPtr &msg_vicon, const geometry_msgs::Vector3StampedConstPtr &msg_cam)
{
  bool publish_meas_error_vel = (meas_error_vel_pub.getNumSubscribers() != 0);
  if (publish_meas_error_vel) {
    cv::Vec3d vicon(msg_vicon->vector.x,msg_vicon->vector.y,msg_vicon->vector.z);
    cv::Vec3d cam(msg_cam->vector.x,msg_cam->vector.y,msg_cam->vector.z);
    cv::Vec3d error = cam - vicon;
    geometry_msgs::Vector3Stamped msg_error;
    msg_error.header = msg_cam->header;
    msg_error.vector.x = error(0);
    msg_error.vector.y = error(1);
    msg_error.vector.z = error(2);
    meas_error_vel_pub.publish(msg_error);
  }
}

void whycon::ViconPublisher::vicon_publish_msg(const std_msgs::Header_<std::allocator<void>> &header) {
  if (publish_observ_dir && rel_pos_is_odd) {
    // calculate body frame observation direction and normalize
    cv::Vec3d relative_pos_bodyFrame = R_WB_.t() * (vicon_payload_pos_ - vicon_quad_pos_);
    relative_pos_bodyFrame *= cable_length_/cv::norm(relative_pos_bodyFrame);
    //relative_pos_bodyFrame = relative_pos_bodyFrame/cv::norm(relative_pos_bodyFrame);

    // create message and publish
    geometry_msgs::Vector3Stamped rel_pos_vec;
    rel_pos_vec.header = header;
    rel_pos_vec.vector.x = relative_pos_bodyFrame(0);
    rel_pos_vec.vector.y = relative_pos_bodyFrame(1);
    rel_pos_vec.vector.z = relative_pos_bodyFrame(2);
    rel_pos_vec.header.frame_id = std::string("0");
    relative_pos_pub.publish(rel_pos_vec);
    rel_pos_is_odd = false;
  } else {
	  rel_pos_is_odd = true;
  }
  //ROS_INFO("publish");
  relative_pos_worldFrame = vicon_payload_pos_ - vicon_quad_pos_;

  relative_pos_worldFrame = relative_pos_worldFrame *
                             (cable_length_ + distance_tag_CoG_) / cv::norm(relative_pos_worldFrame);

  // calculate relative velocity
  relative_vel_worldFrame = vicon_payload_vel_ - vicon_quad_vel_;

  cv::Vec3d relative_pos_bodyFrame = R_WB_.t() * relative_pos_worldFrame;
  cv::Vec3d relative_vel_bodyFrame = R_WB_.t() * relative_vel_worldFrame*(cable_length_ + distance_tag_CoG_) / cv::norm(relative_pos_worldFrame)
                                    - vicon_quad_angVel_.cross(R_WB_.t()*relative_pos_worldFrame);

  bool publish_vicon_pos_body = (vicon_pos_body_pub.getNumSubscribers() != 0);
  if (publish_vicon_pos_body) {
    geometry_msgs::Vector3Stamped msg_pos;
    msg_pos.header = header;
    cv::Vec3d relative_pos_bodyFrame_normalized = relative_pos_bodyFrame/cv::norm(relative_pos_bodyFrame);
    msg_pos.vector.x = relative_pos_bodyFrame_normalized(0);
    msg_pos.vector.y = relative_pos_bodyFrame_normalized(1);
    msg_pos.vector.z = relative_pos_bodyFrame_normalized(2);
    vicon_pos_body_pub.publish(msg_pos);
  }

  bool publish_vicon_vel_body = (vicon_vel_body_pub.getNumSubscribers() != 0);
  if (publish_vicon_vel_body) {
    geometry_msgs::Vector3Stamped msg_vel;
    msg_vel.header = header;
    cv::Vec3d relative_vel_bodyFrame_normalized = relative_vel_bodyFrame/cv::norm(relative_pos_bodyFrame);
    msg_vel.vector.x = relative_vel_bodyFrame_normalized(0);
    msg_vel.vector.y = relative_vel_bodyFrame_normalized(1);
    msg_vel.vector.z = relative_vel_bodyFrame_normalized(2);
    vicon_vel_body_pub.publish(msg_vel);
  }

  // calculate relative angular velocity
  relative_ang_vel = (vicon_payload_pos_ - vicon_quad_pos_).cross(vicon_payload_vel_ - vicon_quad_vel_) /
                     (vicon_payload_pos_ - vicon_quad_pos_).dot(  vicon_payload_pos_ - vicon_quad_pos_);
  if (!transform_to_world_frame)
    relative_ang_vel = R_WB_.t() * (relative_ang_vel - vicon_quad_angVel_);

  if (filter_velocities) {
    if (filter_a == 0.0) {
      double filter_h = 0.02;
      double filter_T = 0.1;
      filter_a = filter_h/(filter_T-filter_h);
    }

    relative_ang_vel = filter_a  * relative_ang_vel +
                    (1-filter_a) * relative_ang_vel_old_;
    relative_ang_vel_old_ = relative_ang_vel;

    vicon_payload_vel_ = filter_a  * vicon_payload_vel_ +
                      (1-filter_a) * vicon_payload_vel_old_;
    vicon_payload_vel_old_ = vicon_payload_vel_;
  }

  // fill in data
  payload_msgs::PayloadOdom payload_vicon;

  payload_vicon.pose_payload.pose.position.x = relative_pos_worldFrame(0) + vicon_quad_pos_(0); //vicon_quad_pos_(0); //relative_pos_outputFrame(0)+vicon_quad_pos_(0);
  payload_vicon.pose_payload.pose.position.y = relative_pos_worldFrame(1) + vicon_quad_pos_(1); //vicon_quad_pos_(1); //relative_pos_outputFrame(1)+vicon_quad_pos_(1);
  payload_vicon.pose_payload.pose.position.z = relative_pos_worldFrame(2) + vicon_quad_pos_(2); //vicon_quad_pos_(2)-cable_length_-distance_tag_CoG_; //relative_pos_outputFrame(2)+vicon_quad_pos_(2);

  payload_vicon.pose_payload.pose.orientation.x =  atan2(relative_pos_worldFrame(1), -relative_pos_worldFrame(2)); //0.0; // atan2(relative_pos_outputFrame(1), -relative_pos_outputFrame(2));
  payload_vicon.pose_payload.pose.orientation.y = -atan2(relative_pos_worldFrame(0), -relative_pos_worldFrame(2)); //0.0; //-atan2(relative_pos_outputFrame(0), -relative_pos_outputFrame(2));
  payload_vicon.pose_payload.pose.orientation.z = 0.0;
  payload_vicon.pose_payload.pose.orientation.w = std::sqrt(1.0-std::pow(payload_vicon.pose_payload.pose.orientation.x,2)-std::pow(payload_vicon.pose_payload.pose.orientation.y,2));

  payload_vicon.twist_payload.twist.linear.x = vicon_payload_vel_(0); //vicon_quad_vel_(0); //vicon_payload_vel_(0);
  payload_vicon.twist_payload.twist.linear.y = vicon_payload_vel_(1); //vicon_quad_vel_(1); //vicon_payload_vel_(1);
  payload_vicon.twist_payload.twist.linear.z = vicon_payload_vel_(2); //vicon_quad_vel_(2); //vicon_payload_vel_(2);

  payload_vicon.twist_payload.twist.angular.x = relative_ang_vel(0); //0.0; //relative_ang_vel(0);
  payload_vicon.twist_payload.twist.angular.y = relative_ang_vel(1); //0.0; //relative_ang_vel(1);
  payload_vicon.twist_payload.twist.angular.z = relative_ang_vel(2);

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


