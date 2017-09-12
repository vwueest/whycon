#include <ros/ros.h>
#include "comparer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comparer");
    ros::NodeHandle n("~");

    whycon::Comparer comparer(n);

    // stuff to sync signals
    message_filters::Subscriber<payload_msgs::PayloadOdom> vicon_sub(   n, "odom_vicon",    2);
    message_filters::Subscriber<payload_msgs::PayloadOdom> estimate_sub(n, "odom_estimate", 2);

    typedef sync_policies::ApproximateTime<payload_msgs::PayloadOdom, payload_msgs::PayloadOdom> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vicon_sub, estimate_sub);
    sync.registerCallback(boost::bind(&whycon::Comparer::synced_callback, &comparer, _1, _2));

    ros::spin();
}

