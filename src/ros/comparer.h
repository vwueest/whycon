#ifndef COMPARER_H
#define COMPARER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <payload_msgs/PayloadOdom.h>
#include <nav_msgs/Odometry.h>
#include <payload_msgs/PayloadOdom.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Geometry>

using namespace message_filters;

namespace whycon {
    class Comparer {
    private:
        double dt_max_;

    public:
        Comparer(ros::NodeHandle &n);

        void synced_callback(const payload_msgs::PayloadOdomConstPtr& msg_vicon, const payload_msgs::PayloadOdomConstPtr& msg_estimate);

        ros::Publisher p_error_pub,
                       load_pos_error_pub,
                       load_vel_error_pub,
                       angles_error_pub,
                       angVel_error_pub;
    };
}

#endif // COMPARER_H
