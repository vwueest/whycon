#include <ros/ros.h>
#include "vicon_publisher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_publisher");
    ros::NodeHandle n("~");

    whycon::ViconPublisher vicon_publisher(n);

    // stuff to sync signals
    message_filters::Subscriber<nav_msgs::Odometry> vicon_quad_sub(n, "odom_quadrotor", 2);
    message_filters::Subscriber<nav_msgs::Odometry> vicon_payload_sub(n, "odom_payload", 2);

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vicon_quad_sub, vicon_payload_sub);
    sync.registerCallback(boost::bind(&whycon::ViconPublisher::vicon_callback, &vicon_publisher, _1, _2));


    ros::spin();
}

