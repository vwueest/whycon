#include <ros/ros.h>
#include <whycon/localization_system.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_srvs/Empty.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "ocam_functions.h"

namespace whycon {
class WhyConROS {
public:
    WhyConROS(ros::NodeHandle& n);

    void on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    bool reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

private:
    void load_transforms(void);
    void publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr);

    whycon::DetectorParameters parameters;
    boost::shared_ptr<whycon::LocalizationSystem> system;
    bool is_tracking, should_reset;
    int max_attempts, max_refine;
    std::string world_frame_id, frame_id;
    int targets;
    double xscale, yscale, cable_length;
    bool use_omni_model;
    bool publish_tf;
    std::vector<double> R_BC_yaml;
    std::vector<double> B_T_BC_yaml;
    std::string calib_file;
    cv::Matx33d R_BC;
    cv::Vec3d B_T_BC;

    ocam_model model;

    std::vector<double> projection;
    tf::Transform similarity;

    image_transport::ImageTransport it;
    image_transport::CameraSubscriber cam_sub;
    ros::ServiceServer reset_service;

    ros::Publisher image_pub, poses_pub, context_pub, projection_pub, pixel_pub;
    boost::shared_ptr<tf::TransformBroadcaster>	transform_broadcaster;
    tf::TransformBroadcaster tf_broadcaster;

    image_geometry::PinholeCameraModel camera_model;

    bool transformation_loaded;
};
}
