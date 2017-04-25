#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <whycon/Projection.h>
#include "whycon_ros.h"

whycon::WhyConROS::WhyConROS(ros::NodeHandle& n) : is_tracking(false), should_reset(true), it(n)
{
    transformation_loaded = false;
    similarity.setIdentity();

    if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");

    n.param("name", frame_id, std::string("whycon"));
    n.param("world_frame", world_frame_id, std::string("world"));
    n.param("max_attempts", max_attempts, 1);
    n.param("max_refine", max_refine, 1);
    n.param("B_T_BC", B_T_BC_yaml, {0,0,0});
    n.param("R_BC", R_BC_yaml, {1,0,0,  0,1,0,  0,0,1});
    n.param("cable_length", cable_length, 1.0);
    n.param("use_omni_model", use_omni_model, false);
    n.param("calib_file", calib_file, std::string(""));
    n.param("publish_tf", publish_tf, false);

    B_T_BC = cv::Vec3d(B_T_BC_yaml[0], B_T_BC_yaml[1], B_T_BC_yaml[2]);
    R_BC = cv::Matx33d(R_BC_yaml[0], R_BC_yaml[1], R_BC_yaml[2],\
                       R_BC_yaml[3], R_BC_yaml[4], R_BC_yaml[5],\
                       R_BC_yaml[6], R_BC_yaml[7], R_BC_yaml[8]);

    n.getParam("outer_diameter", parameters.outer_diameter);
    n.getParam("inner_diameter", parameters.inner_diameter);
    n.getParam("center_distance_tolerance_abs", parameters.center_distance_tolerance_abs);
    n.getParam("center_distance_tolerance_ratio", parameters.center_distance_tolerance_ratio);
    n.getParam("roundness_tolerance", parameters.roundness_tolerance);
    n.getParam("circularity_tolerance", parameters.circularity_tolerance);
    n.getParam("max_size", parameters.max_size);
    n.getParam("min_size", parameters.min_size);
    n.getParam("ratio_tolerance", parameters.ratio_tolerance);
    n.getParam("max_eccentricity", parameters.max_eccentricity);

    load_transforms();
    //transform_broadcaster = boost::make_shared<tf::TransformBroadcaster>();



    /* initialize ros */
    int input_queue_size = 1;
    n.param("input_queue_size", input_queue_size, input_queue_size);
    if (use_omni_model) {
        cam_sub = it.subscribeCamera("/camera/image_raw", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
    } else {
        cam_sub = it.subscribeCamera("/camera/image_rect_color", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
    }


    image_pub = n.advertise<sensor_msgs::Image>("image_out", 1);
    poses_pub = n.advertise<geometry_msgs::PoseArray>("poses", 1);
    pixel_pub = n.advertise<geometry_msgs::PointStamped>("pixel_coord", 1);
    context_pub = n.advertise<sensor_msgs::Image>("context", 1);
    projection_pub = n.advertise<whycon::Projection>("projection", 1);

    reset_service = n.advertiseService("reset", &WhyConROS::reset, this);

    //omni cam model
    get_ocam_model(&model, strdup(calib_file.c_str()));
}

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    camera_model.fromCameraInfo(info_msg);
    if (camera_model.fullResolution().width == 0) { ROS_ERROR_STREAM("camera is not calibrated!"); return; }

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
    const cv::Mat& image = cv_ptr->image;

    if (!system)
        system = boost::make_shared<whycon::LocalizationSystem>(targets, image.size().width, image.size().height, cv::Mat(camera_model.fullIntrinsicMatrix()), cv::Mat(camera_model.distortionCoeffs()), parameters);

    is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);

    if (is_tracking) {
        publish_results(image_msg->header, cv_ptr);
        should_reset = false;
    }
    else if (image_pub.getNumSubscribers() != 0)
        image_pub.publish(cv_ptr);

    if (context_pub.getNumSubscribers() != 0) {
        cv_bridge::CvImage cv_img_context;
        cv_img_context.encoding = cv_ptr->encoding;
        cv_img_context.header.stamp = cv_ptr->header.stamp;
        system->detector.context.debug_buffer(cv_ptr->image, cv_img_context.image);
        context_pub.publish(cv_img_context.toImageMsg());
    }
}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    should_reset = true;
    return true;
}

void whycon::WhyConROS::publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr)
{
    bool publish_images = (image_pub.getNumSubscribers() != 0);
    bool publish_poses  = (poses_pub.getNumSubscribers() != 0);
    bool publish_pixels = (pixel_pub.getNumSubscribers() != 0);

    if (!publish_images && !publish_poses && !publish_pixels) return;

    // prepare image output
    cv::Mat output_image;
    if (publish_images)
        output_image = cv_ptr->image.clone();

    geometry_msgs::PoseArray pose_array;

    // go through detected targets
    for (int i = 0; i < system->targets; i++) {
        const whycon::CircleDetector::Circle& circle = system->get_circle(i);
        whycon::LocalizationSystem::Pose pose;

        if (!use_omni_model) {
            pose = system->get_pose(circle);
        } else {
            //use ocam model to determine direction
            double point3D[3];
            double point2D[2] = {circle.y, circle.x};
            //ROS_INFO("circle position:\n%f %f",point2D[0],point2D[1]);

            cam2world(point3D, point2D, &model);
            //ROS_INFO("direction before rot:\n%f %f %f",point3D[0],point3D[1],point3D[2]);

            // adapt coord sys of ocam to coord sys of whycon
            cv::Vec3d direction(point3D[1],point3D[0],-point3D[2]);
            //ROS_INFO("direction after rot:\n%f %f %f",direction[0],direction[1],direction[2]);

            // rotate to quad frame
            direction = R_BC * direction;
            //ROS_INFO("direction in quad frame:\n%f %f %f",direction[0],direction[1],direction[2]);

            // calculate distance camera-load
            double b_term = 2 * (B_T_BC[0]*direction[0] + B_T_BC[1]*direction[1] + B_T_BC[2]*direction[2]);
            double c_term = (B_T_BC[0]*B_T_BC[0] + B_T_BC[1]*B_T_BC[1] + B_T_BC[2]*B_T_BC[2])-cable_length*cable_length;
            double dist   = (-b_term+sqrt(b_term*b_term-4*c_term))/2;
            //ROS_INFO("\nb_term %f\nc_term %f\ndist %f",b_term,c_term,dist);

            // save to pose
            pose.pos = B_T_BC + dist * direction;
            //ROS_INFO("position in quad frame:\nx:%f y:%f z:%f",pose.pos(0),pose.pos(1),pose.pos(2));
        }
        cv::Vec3f coord = pose.pos;

        // draw each target
        if (publish_images) {
            std::ostringstream ostr;
            ostr << std::fixed << std::setprecision(2);
            ostr << coord << " " << i;
            circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,255));
            /*whycon::CircleDetector::Circle new_circle = circle.improveEllipse(cv_ptr->image);
            new_circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,0));*/
            cv::circle(output_image, camera_model.project3dToPixel(cv::Point3d(coord)), 1, cv::Scalar(255,0,255), 1, CV_AA);
        }

        if (publish_poses) {
            geometry_msgs::Pose p;
            p.position.x = pose.pos(0);
            p.position.y = pose.pos(1);
            p.position.z = pose.pos(2);
            p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
            pose_array.poses.push_back(p);

            //geometry_msgs::Pose p;
            //p.position.x = circle.x;
            //p.position.y = circle.y;
            //p.position.z = 0.0;
            //p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
            //pose_array.poses.push_back(p);
        }

        if (publish_tf) {
            tf::Transform transform;
            geometry_msgs::Pose p;
            p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
            transform.setOrigin( tf::Vector3( pose.pos(0), pose.pos(1), pose.pos(2) ) );
            transform.setRotation( tf::createQuaternionFromRPY(0, pose.rot(0), pose.rot(1)) );
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "tag"+std::to_string(i)));
        }

        if (publish_pixels) {
            geometry_msgs::PointStamped p;
            p.point.x = circle.x;
            p.point.y = circle.y;
            p.point.z = 0;
            p.header = header;
            p.header.frame_id = frame_id;
            pixel_pub.publish(p);
            //ROS_INFO("printing pixel: %f %f %f",p.x,p.y,p.z);
        }
    }

    if (publish_images) {
        cv_bridge::CvImage output_image_bridge = *cv_ptr;
        output_image_bridge.image = output_image;
        image_pub.publish(output_image_bridge);
    }

    if (publish_poses) {
        pose_array.header = header;
        pose_array.header.frame_id = frame_id;
        poses_pub.publish(pose_array);
    }

    if (transformation_loaded)
    {
        transform_broadcaster->sendTransform(tf::StampedTransform(similarity, header.stamp, world_frame_id, frame_id));

        whycon::Projection projection_msg;
        projection_msg.header = header;
        for (size_t i = 0; i < projection.size(); i++) projection_msg.projection[i] = projection[i];
        projection_pub.publish(projection_msg);
    }
}

void whycon::WhyConROS::load_transforms(void)
{
    std::string filename = frame_id + "_transforms.yml";
    ROS_INFO_STREAM("Loading file " << filename);

    std::ifstream file(filename);
    if (!file) {
        ROS_WARN_STREAM("Could not load \"" << filename << "\"");
        return;
    }

    YAML::Node node = YAML::Load(file);

    projection.resize(9);
    for (int i = 0; i < 9; i++)
        projection[i] = node["projection"][i].as<double>();

    std::vector<double> similarity_origin(3);
    for (int i = 0; i < 3; i++) similarity_origin[i] = node["similarity"]["origin"][i].as<double>();

    std::vector<double> similarity_rotation(4);
    for (int i = 0; i < 4; i++) similarity_rotation[i] = node["similarity"]["rotation"][i].as<double>();

    tf::Vector3 origin(similarity_origin[0], similarity_origin[1], similarity_origin[2]);
    tf::Quaternion Q(similarity_rotation[0], similarity_rotation[1], similarity_rotation[2], similarity_rotation[3]);

    similarity.setOrigin(origin);
    similarity.setRotation(Q);

    transformation_loaded = true;

    ROS_INFO_STREAM("Loaded transformation from \"" << filename <<  "\"");
}


