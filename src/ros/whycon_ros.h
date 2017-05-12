#include <ros/ros.h>
#include <cmath>
#include <whycon/localization_system.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_srvs/Empty.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Geometry>
#include <ros/callback_queue.h>
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
    int num_meas = 0;
    double avg = 0;

    void load_transforms(void);
    void publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr);

    void vicon_quad_callback(const geometry_msgs::PoseStamped& msg);
    void vicon_payload_callback(const nav_msgs::Odometry& msg);
    void vicon_publish_msg(const std_msgs::Header_<std::allocator<void>>& header); //vicon_publish_msg(msg.header)

    whycon::DetectorParameters parameters;
    boost::shared_ptr<whycon::LocalizationSystem> system;
    bool is_tracking, should_reset;
    int max_attempts, max_refine;
    std::string world_frame_id, frame_id;
    int targets;
    double xscale, yscale, cable_length, distance_tag_CoG, time_older;
    bool use_omni_model;
    bool publish_tf;
    bool publish_vicon;
    bool transform_to_world_frame;
    std::vector<double> R_BC_yaml;
    std::vector<double> B_T_BC_yaml;
    std::string calib_file;
    cv::Matx33d R_BC;
    cv::Vec3d B_T_BC;
    cv::Matx33d R_WB_;
    std::queue<cv::Matx33d> R_WB_queue_;
    std::queue<double> time_R_WB_queue_;

    ros::Time time_old_whycon_, time_new_whycon_, time_new_vicon_payload_, time_old_vicon_quad_, time_new_vicon_quad_;
    ros::Duration time_diff_whycon, time_diff_vicon;
    cv::Vec3d whycon_velocity = {0, 0, 0};
    cv::Vec3d whycon_angVelocity = {0, 0, 0};
    cv::Vec3d whycon_position_bodyFrame_;
    cv::Vec3d whycon_position_outputFrame_old;
    cv::Vec3d whycon_position_old;
    cv::Vec3d whycon_angle;
    cv::Vec3d whycon_angle_outputFrame_old;
    cv::Vec3d whycon_angle_old;
    cv::Vec3d vicon_quad_pos_;
    cv::Vec3d vicon_quad_vel_;
    cv::Vec3d vicon_quad_angVel_;
    cv::Vec3d vicon_payload_pos_;
    cv::Vec3d vicon_payload_vel_;

    // speed up variables
    bool publish_images, publish_odom_whycon, publish_pixels;
    double b_term, c_term, dist;
    cv::Mat output_image;
    cv::Vec3d direction,
              whycon_position_outputFrame, whycon_velocity_outputFrame,
              whycon_angle_outputFrame, whycon_angVel_outputFrame, whycon_angVel_outputFrame_old, whycon_angVel_outputFrame_old2;
    cv::Vec3f coord;
    nav_msgs::Odometry odom_whycon;

    ocam_model model;

    std::vector<double> projection;
    tf::Transform similarity;

    image_transport::ImageTransport it;
    image_transport::CameraSubscriber cam_sub;
    ros::ServiceServer reset_service;

    ros::Publisher image_pub, poses_pub, odom_whycon_pub, odom_vicon_pub, context_pub, projection_pub, pixel_pub;
    boost::shared_ptr<tf::TransformBroadcaster>	transform_broadcaster;
    tf::TransformBroadcaster tf_broadcaster;
    ros::Subscriber vicon_quad_sub, vicon_payload_sub;

    image_geometry::PinholeCameraModel camera_model;

    bool transformation_loaded;
};
}
