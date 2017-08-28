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


    message_filters::Subscriber<geometry_msgs::Vector3Stamped> vicon_pos_body_sub(n, "vicon_pos_body", 5);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> vicon_vel_body_sub(n, "vicon_vel_body", 5);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> measurement_pos_body_sub(n, "measurement_pos_body", 5);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> measurement_vel_body_sub(n, "measurement_vel_body", 5);

    typedef sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> PosSyncPolicy;
    Synchronizer<PosSyncPolicy> posSync(PosSyncPolicy(25), vicon_pos_body_sub, measurement_pos_body_sub);
    posSync.registerCallback(boost::bind(&whycon::ViconPublisher::synced_pos_callback, &vicon_publisher, _1, _2));

    typedef sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> velSyncPolicy;
    Synchronizer<velSyncPolicy> velSync(velSyncPolicy(25), vicon_vel_body_sub, measurement_vel_body_sub);
    velSync.registerCallback(boost::bind(&whycon::ViconPublisher::synced_vel_callback, &vicon_publisher, _1, _2));

    ros::spin();
}

