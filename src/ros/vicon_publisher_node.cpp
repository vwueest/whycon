#include <ros/ros.h>
#include "vicon_publisher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_publisher");
    ros::NodeHandle n("~");

    whycon::ViconPublisher vicon_publisher(n);

    // stuff to sync signals
    std::string vicon_quad_topic, vicon_payload_topic;
    n.param("vicon_quad_topic", vicon_quad_topic, std::string(""));
    n.param("vicon_payload_topic", vicon_payload_topic, std::string(""));

    message_filters::Subscriber<nav_msgs::Odometry> vicon_quad_sub(n, vicon_quad_topic, 2);
    message_filters::Subscriber<nav_msgs::Odometry> vicon_payload_sub(n, vicon_payload_topic, 2);

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vicon_quad_sub, vicon_payload_sub);
    sync.registerCallback(boost::bind(&whycon::ViconPublisher::vicon_callback, &vicon_publisher, _1, _2));


    ros::spin();
}

