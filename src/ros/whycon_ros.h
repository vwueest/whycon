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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <payload_msgs/PayloadOdom.h>
#include "ocam_functions.h"

namespace whycon {
    class WhyConROS {
    public:
        WhyConROS(ros::NodeHandle& n);

        void on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
        bool reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        void publish_odom(const nav_msgs::OdometryConstPtr& msg_quad, const geometry_msgs::Vector3StampedConstPtr &msg_relPos_bodyFrame);

    private:
        int num_meas = 0;
        double avg = 0;
        double dropped_frames = 0;

        void load_transforms(void);

        whycon::DetectorParameters parameters;
        boost::shared_ptr<whycon::LocalizationSystem> system;
        bool is_tracking, should_reset;
        int max_attempts, max_refine;
        std::string world_frame_id, frame_id;
        int targets;
        //double xscale, yscale
        double cable_length_, distance_tag_CoG_;
        bool use_omni_model;
        bool publish_tf;
        bool transform_to_world_frame;
        std::vector<double> R_BC_yaml;
        std::vector<double> B_T_BC_yaml;
        std::string calib_file;
        cv::Matx33d R_BC_;
        cv::Vec3d B_T_BC_;

        double filter_a;

        ros::Time time_old_whycon;
        cv::Vec3d whycon_position_outputFrame_old;
        cv::Vec3d whycon_position_bodyFrame_old;
        cv::Vec3d whycon_angle_outputFrame_old;
        cv::Vec3d whycon_angVel_outputFrame_old;
//        cv::Vec3d whycon_angVel_y1;
//        cv::Vec3d whycon_angVel_y2;
//        cv::Vec3d whycon_angVel_y3;
//        cv::Vec3d whycon_angVel_u1;
//        cv::Vec3d whycon_angVel_u2;
//        cv::Vec3d whycon_angVel_u3;
        cv::Vec3d filter_A = {-1.418983,0.553270};//{-1.142981, 0.412802};//{-1.418983,0.553270};//0.1{-1.142981, 0.412802, 0.0};//0.2{-1.561018, 0.641352, 0.0};
        cv::Vec4d filter_B = {0.033572, 0.067144, 0.033572, 0.0};//{0.067455, 0.134911, 0.067455};//{0.033572, 0.067144, 0.033572, 0.0};//0.1{0.067455, 0.134911, 0.067455};//0.2{0.020083, 0.040167, 0.020083, 0.0};
        cv::Matx34d whycon_angVel_u;
        cv::Matx34d whycon_angVel_y;
        cv::Matx34d whycon_velocity_u;
        cv::Matx34d whycon_velocity_y;

        cv::Mat output_image;
        cv::Vec3f coord;

        ocam_model model;

        std::vector<double> projection;
        tf::Transform similarity;

        image_transport::ImageTransport it;
        image_transport::CameraSubscriber cam_sub;
        ros::ServiceServer reset_service;

        ros::Publisher image_pub, odom_pub, odom_payload_pub, relative_pos_pub; //, poses_pub, odom_vicon_pub, projection_pub;

        image_geometry::PinholeCameraModel camera_model;

        bool transformation_loaded;
    };
}
