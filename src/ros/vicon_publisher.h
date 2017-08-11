#ifndef VICON_PUBLISHER_H
#define VICON_PUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <payload_msgs/PayloadOdom.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

using namespace message_filters;

namespace whycon {
    class ViconPublisher {
    private:
        double time_new_vicon_payload_, time_new_vicon_quad_;
        double cable_length_, distance_tag_CoG_;
        bool transform_to_world_frame, publish_observ_dir, filter_velocities;
        cv::Matx33d R_WB_;
        cv::Vec4d vicon_quad_angle_;
        cv::Vec3d vicon_quad_pos_, vicon_quad_vel_, vicon_quad_angVel_, vicon_payload_pos_, vicon_payload_vel_;
        cv::Vec3d relative_pos_outputFrame, relative_vel_outputFrame, relative_ang_vel;

        cv::Vec3d relative_ang_vel_old_, vicon_payload_vel_old_;
        double filter_a = 0.0;

        void vicon_publish_msg(const std_msgs::Header_<std::allocator<void>>& header); //vicon_publish_msg(msg.header);

//        cv::Vec3d relative_pos_bodyFrame_old_;
//        double vel_error_ = 0;
//        ros::Time t_old_;

    public:
        ViconPublisher(ros::NodeHandle &n);

        void vicon_callback(const nav_msgs::OdometryConstPtr& msg_quad, const nav_msgs::OdometryConstPtr& msg_payload);

        ros::Publisher odom_vicon_pub, relative_pos_pub;
    };
}

#endif // VICON_PUBLISHER_H
