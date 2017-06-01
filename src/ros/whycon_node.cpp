#include <ros/ros.h>
#include "whycon_ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "whycon");
    ros::NodeHandle n("~");

    whycon::WhyConROS whycon_ros(n);


    // stuff to sync signals
    std::string vicon_quad_topic;
    n.param("vicon_quad_topic", vicon_quad_topic, std::string(""));

    if (vicon_quad_topic.empty())
        ROS_WARN("Vicon Topic for Quadrotor not defined");
    else
        ROS_INFO("Subscribed to %s", vicon_quad_topic.c_str());

    message_filters::Subscriber<nav_msgs::Odometry> vicon_quad_sub(n, vicon_quad_topic, 6);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> observ_dir_sub(n, "observ_dir", 2);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::Vector3Stamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vicon_quad_sub, observ_dir_sub);
    sync.registerCallback(boost::bind(&whycon::WhyConROS::calculate_3D_position, &whycon_ros, _1, _2));

    ros::spin();
}
