#include <ros/ros.h>
#include "vicon_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon_publisher");
  ros::NodeHandle n("~");

  whycon::ViconPublisher vicon_publisher(n);
  ros::spin();
}

