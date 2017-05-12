#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
//#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
//#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <whycon/Projection.h>
#include "whycon_ros.h"

whycon::WhyConROS::WhyConROS(ros::NodeHandle &n) : is_tracking(false), should_reset(true), it(n) {
    transformation_loaded = false;
    similarity.setIdentity();

    if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");

    std::string vicon_quad_topic, vicon_payload_topic;
    n.param("vicon_quad_topic", vicon_quad_topic, std::string(""));
    n.param("vicon_payload_topic", vicon_payload_topic, std::string(""));
    n.param("name", frame_id, std::string("whycon"));
    n.param("world_frame", world_frame_id, std::string("world"));
    n.param("max_attempts", max_attempts, 1);
    n.param("max_refine", max_refine, 1);
    n.param("B_T_BC", B_T_BC_yaml, {0, 0, 0});
    n.param("R_BC", R_BC_yaml, {1, 0, 0, 0, 1, 0, 0, 0, 1});
    n.param("cable_length", cable_length, 1.0);
    n.param("distance_tag_CoG", distance_tag_CoG, 0.0);
    n.param("use_omni_model", use_omni_model, false);
    n.param("calib_file", calib_file, std::string(""));
    n.param("publish_tf", publish_tf, false);
    n.param("publish_vicon", publish_vicon, false);
    n.param("transform_to_world_frame", transform_to_world_frame, false);

    B_T_BC = cv::Vec3d(B_T_BC_yaml[0], B_T_BC_yaml[1], B_T_BC_yaml[2]);
    R_BC = cv::Matx33d(R_BC_yaml[0], R_BC_yaml[1], R_BC_yaml[2], \
                       R_BC_yaml[3], R_BC_yaml[4], R_BC_yaml[5], \
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

    //transform_to_world_frame = !vicon_quad_topic.empty();
    if (transform_to_world_frame || publish_vicon) {
        vicon_quad_sub = n.subscribe(vicon_quad_topic.c_str(), 1, &whycon::WhyConROS::vicon_quad_callback, this);
        ROS_INFO("subscribed to odometry msg quadrotor: %s", vicon_quad_topic.c_str());
    }

    if (publish_vicon) {
        vicon_payload_sub = n.subscribe(vicon_payload_topic.c_str(),1,&whycon::WhyConROS::vicon_payload_callback,this);
        ROS_INFO("subscribed to odometry msg payload: %s", vicon_payload_topic.c_str());
    }

    /* initialize ros */
    int input_queue_size = 1;
    n.param("input_queue_size", input_queue_size, input_queue_size);
    if (use_omni_model)
        cam_sub = it.subscribeCamera("/camera/image_raw", input_queue_size,
                                     boost::bind(&WhyConROS::on_image, this, _1, _2));
    else
        cam_sub = it.subscribeCamera("/camera/image_rect_color", input_queue_size,
                                     boost::bind(&WhyConROS::on_image, this, _1, _2));

    image_pub = n.advertise<sensor_msgs::Image>("image_out", 1);
    odom_whycon_pub = n.advertise<nav_msgs::Odometry>("odom_whycon", 1);
    odom_vicon_pub = n.advertise<nav_msgs::Odometry>("odom_vicon", 1);
    pixel_pub = n.advertise<geometry_msgs::PointStamped>("pixel_coord", 1);
    context_pub = n.advertise<sensor_msgs::Image>("context", 1);
    projection_pub = n.advertise<whycon::Projection>("projection", 1);

    reset_service = n.advertiseService("reset", &WhyConROS::reset, this);

    //omni cam model
    get_ocam_model(&model, strdup(calib_file.c_str()));
}

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr &image_msg,
                                 const sensor_msgs::CameraInfoConstPtr &info_msg) {
    camera_model.fromCameraInfo(info_msg);
    if (camera_model.fullResolution().width == 0) {
        ROS_ERROR_STREAM("camera is not calibrated!");
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
    const cv::Mat &image = cv_ptr->image;

    if (!system)
        system = boost::make_shared<whycon::LocalizationSystem>(targets, image.size().width, image.size().height,
                                                                cv::Mat(camera_model.fullIntrinsicMatrix()),
                                                                cv::Mat(camera_model.distortionCoeffs()), parameters);

    is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);

    if (is_tracking) {
        publish_results(image_msg->header, cv_ptr);
        should_reset = false;
    } else if (image_pub.getNumSubscribers() != 0)
        image_pub.publish(cv_ptr);

    if (context_pub.getNumSubscribers() != 0) {
        cv_bridge::CvImage cv_img_context;
        cv_img_context.encoding = cv_ptr->encoding;
        cv_img_context.header.stamp = cv_ptr->header.stamp;
        system->detector.context.debug_buffer(cv_ptr->image, cv_img_context.image);
        context_pub.publish(cv_img_context.toImageMsg());
    }
}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    should_reset = true;
    return true;
}

void whycon::WhyConROS::publish_results(const std_msgs::Header &header, const cv_bridge::CvImageConstPtr &cv_ptr) {
    publish_images = (image_pub.getNumSubscribers() != 0);
    publish_odom_whycon = (odom_whycon_pub.getNumSubscribers() != 0);
    publish_pixels = (pixel_pub.getNumSubscribers() != 0);
    publish_vicon = (odom_vicon_pub.getNumSubscribers() != 0);

    time_new_whycon_ = header.stamp;
    ros::Duration t_diff_w_v(time_new_whycon_-time_new_vicon_quad_);

    ROS_INFO("w-v:     %f",t_diff_w_v.toSec());

    num_meas++;
    avg = (avg * (num_meas - 1) + t_diff_w_v.toSec())/num_meas;
    ROS_INFO("w-v: avg %f",avg);

    //ros::Time now = ros::Time::now();
    //ros::Duration diff(now - header.stamp);
    //ROS_INFO("delay whycon: %d.%d",diff.sec,diff.nsec);

//    ///////////////////////////////////
//    time_R_WB_queue_.push(1.0);
//    time_R_WB_queue_.push(2.0);
//    time_R_WB_queue_.push(3.0);
//    time_R_WB_queue_.push(4.0);
//    time_R_WB_queue_.push(5.0);
//    time_R_WB_queue_.push(6.0);
//
//    time_new_whycon_ = 3.0;
//
//    R_WB_queue_.push(cv::Matx33d(1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0));
//    R_WB_queue_.push(cv::Matx33d(2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0));
//    R_WB_queue_.push(cv::Matx33d(3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0));
//    R_WB_queue_.push(cv::Matx33d(4.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0));
//    R_WB_queue_.push(cv::Matx33d(5.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0));
//    R_WB_queue_.push(cv::Matx33d(6.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0));
//    ///////////////////////////////////

//    // find correct R_WB_ in R_WB_queue
//    time_older = time_R_WB_queue_.front();
//    time_R_WB_queue_.pop();
//    ROS_INFO("timediff 0: time %f",std::abs(time_older - time_new_whycon_));
//    while ( std::abs(time_R_WB_queue_.front() - time_new_whycon_) <  std::abs(time_older - time_new_whycon_)) {
//            ROS_INFO("timediff 1: time %f",std::abs(time_older - time_new_whycon_));
//            ROS_INFO("timediff 2: time %f",std::abs(time_R_WB_queue_.front() - time_new_whycon_));
//            time_older = time_R_WB_queue_.front();
//            time_R_WB_queue_.pop();
//            R_WB_queue_.pop();
//    }
//    ROS_INFO("chosen: %f",std::abs(time_older - time_new_whycon_));
//    R_WB_ = R_WB_queue_.front();
//    R_WB_queue_.pop();
//    time_R_WB_queue_.push(time_older);
//    ROS_INFO("next in line: R_WB %f, time %f",R_WB_queue_.front()(0,0),time_R_WB_queue_.front());
//    ROS_INFO("sizes: R_WB %d, time %d",R_WB_queue_.size(),time_R_WB_queue_.size());
//    ROS_INFO("values: R_WB %f, time %f",R_WB_(0,0),time_older);
//
//    if( std::abs(time_new_whycon_ - time_older) > 0.015 ) {
//        ROS_INFO("transform too old: %f", time_new_whycon_ - time_older);
//        return;
//    }

    if (!publish_images && !publish_pixels && !publish_odom_whycon && !publish_vicon) return;

    // prepare image output
    cv::Mat output_image;
    if (publish_images)
        output_image = cv_ptr->image.clone();

    // geometry_msgs::PoseArray pose_array;
    // geometry_msgs::PoseArray pose_world_array;

    // go through detected targets
    for (int i = 0; i < system->targets; i++) {
        const whycon::CircleDetector::Circle &circle = system->get_circle(i);
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
            direction = {point3D[1], point3D[0], -point3D[2]};
            //ROS_INFO("direction after rot:\n%f %f %f",direction[0],direction[1],direction[2]);

            // rotate to quad frame
            direction = R_BC * direction;
            //ROS_INFO("direction in quad frame:\n%f %f %f",direction[0],direction[1],direction[2]);

            // calculate distance camera-load
            b_term = 2 * B_T_BC.dot(direction);
            c_term = B_T_BC.dot(B_T_BC) - cable_length * cable_length;
            dist = (-b_term + sqrt(b_term * b_term - 4 * c_term)) / 2;
            //ROS_INFO("\nb_term %f\nc_term %f\ndist %f",b_term,c_term,dist);

            // save to pose
            //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
            //ros::spinOnce();
            whycon_position_bodyFrame_ = (B_T_BC + dist * direction) * (cable_length + distance_tag_CoG) / cable_length;

            // calculate position for output
            if (!transform_to_world_frame)
                whycon_position_outputFrame = whycon_position_bodyFrame_;
            else
                whycon_position_outputFrame = R_WB_ * whycon_position_bodyFrame_;

            // calculate angle for output
            whycon_angle_outputFrame = { atan2(whycon_position_outputFrame(1), -whycon_position_outputFrame(2)),
                                        -atan2(whycon_position_outputFrame(0), -whycon_position_outputFrame(2)),
                                         0};

            // calculate velocity for output
            if (!time_old_whycon_.isZero()) {
                time_diff_whycon = time_new_whycon_ - time_old_whycon_;
                whycon_velocity_outputFrame = (whycon_position_outputFrame - whycon_position_outputFrame_old) /
                                              (time_diff_whycon.toSec());
            }

            // calculate angular velocity for output
            if (!time_old_whycon_.isZero()) {
                whycon_angVel_outputFrame = (whycon_angle_outputFrame - whycon_angle_outputFrame_old) /
                                            (time_diff_whycon.toSec());
//                whycon_angVel_outputFrame = (whycon_angVel_outputFrame +
//                                             whycon_angVel_outputFrame_old +
//                                             whycon_angVel_outputFrame_old2) / 3;
//                whycon_angVel_outputFrame_old = whycon_angVel_outputFrame;
//                whycon_angVel_outputFrame_old2 = whycon_angVel_outputFrame_old;
            }
        }
        coord = whycon_position_bodyFrame_;

        // draw each target
        if (publish_images) {
            std::ostringstream ostr;
            ostr << std::fixed << std::setprecision(2);
            ostr << coord << " " << i;
            circle.draw(output_image, ostr.str(), cv::Vec3b(0, 255, 255));
            /*whycon::CircleDetector::Circle new_circle = circle.improveEllipse(cv_ptr->image);
            new_circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,0));*/
            cv::circle(output_image,camera_model.project3dToPixel(cv::Point3d(coord)),1,cv::Scalar(255,0,255),1,CV_AA);
        }

        if (publish_odom_whycon) {
            odom_whycon.pose.pose.position.x = whycon_position_outputFrame(0);
            odom_whycon.pose.pose.position.y = whycon_position_outputFrame(1);
            odom_whycon.pose.pose.position.z = whycon_position_outputFrame(2);

            odom_whycon.twist.twist.linear.x = whycon_velocity_outputFrame(0);
            odom_whycon.twist.twist.linear.y = whycon_velocity_outputFrame(1);
            odom_whycon.twist.twist.linear.z = whycon_velocity_outputFrame(2);

            odom_whycon.pose.pose.orientation.x = whycon_angle_outputFrame(0);
            odom_whycon.pose.pose.orientation.y = whycon_angle_outputFrame(1);
            odom_whycon.pose.pose.orientation.z = whycon_angle_outputFrame(2);
            odom_whycon.pose.pose.orientation.z = 0;

            odom_whycon.twist.twist.angular.x = whycon_angVel_outputFrame(0);
            odom_whycon.twist.twist.angular.y = whycon_angVel_outputFrame(1);
            odom_whycon.twist.twist.angular.z = whycon_angVel_outputFrame(2);

            // odom_whycon.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose_world.rot(0), pose_world.rot(1));
            odom_whycon.header = header;
            odom_whycon.header.frame_id = frame_id;
            odom_whycon_pub.publish(odom_whycon);
        }

        whycon_position_outputFrame_old = whycon_position_outputFrame;
        whycon_angle_outputFrame_old = whycon_angle_outputFrame;
        time_old_whycon_ = time_new_whycon_;
        //R_WB_old = R_WB_;

        // if (publish_vicon) {
        //     //ROS_INFO("vicon_payload_pos_: %f %f %f",vicon_payload_pos_(0),vicon_payload_pos_(1),vicon_payload_pos_(2));
        //     //ROS_INFO("vicon_quad_pos_: %f %f %f",vicon_quad_pos_(0),vicon_quad_pos_(1),vicon_quad_pos_(2));
        //     cv::Vec3f tempVec = ( R_WB_.t() * (vicon_payload_pos_ - vicon_quad_pos_) );
        //     //ROS_INFO("Vec1: %f %f %f",tempVec(0),tempVec(1),tempVec(2));
        //     tempVec = (cable_length + distance_tag_CoG) * tempVec / sqrt(tempVec(0)*tempVec(0) + tempVec(1)*tempVec(1) + tempVec(2)*tempVec(2));
        //     //ROS_INFO("Vec2: %f %f %f",tempVec(0),tempVec(1),tempVec(2));
        //     geometry_msgs::PoseStamped p_vicon;
        //     p_vicon.pose.position.x = tempVec(0);
        //     p_vicon.pose.position.y = tempVec(1);
        //     p_vicon.pose.position.z = tempVec(2);
        //     // p_vicon.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, p_vicon.rot(0), p_vicon.rot(1));
        //     p_vicon.header = header;
        //     p_vicon.header.frame_id = frame_id;
        //     odom_vicon_pub.publish(p_vicon);
        // }

        if (publish_tf) {
            tf::Transform transform;
            geometry_msgs::Pose p;
            p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
            transform.setOrigin(tf::Vector3(pose.pos(0), pose.pos(1), pose.pos(2)));
            transform.setRotation(tf::createQuaternionFromRPY(0, pose.rot(0), pose.rot(1)));
            tf_broadcaster.sendTransform(
                    tf::StampedTransform(transform, ros::Time::now(), "base_link", "tag" + std::to_string(i)));
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

    // if (publish_poses) {
    //     pose_array.header = header;
    //     pose_array.header.frame_id = frame_id;
    //     poses_pub.publish(pose_array);
    // }

    // if (publish_odom_whycon) {
    //     pose_world_array.header = header;
    //     pose_world_array.header.frame_id = frame_id;
    //     odom_whycon_pub.publish(pose_world_array);
    // }

//    if (transformation_loaded) {
//        transform_broadcaster->sendTransform(tf::StampedTransform(similarity, header.stamp, world_frame_id, frame_id));
//
//        whycon::Projection projection_msg;
//        projection_msg.header = header;
//        for (size_t i = 0; i < projection.size(); i++) projection_msg.projection[i] = projection[i];
//        projection_pub.publish(projection_msg);
//    }

    //ros::Duration diff2(now2 - header.stamp);
    ros::Duration diff(ros::Time::now() - time_new_whycon_);
    ROS_INFO("total delay after calc whycon: %d.%d",diff.sec,diff.nsec);
    //ROS_INFO("calculation time whycon: %d.%d",diff3.sec,diff3.nsec);
}

void whycon::WhyConROS::load_transforms(void) {
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

    ROS_INFO_STREAM("Loaded transformation from \"" << filename << "\"");
}

void whycon::WhyConROS::vicon_quad_callback(const nav_msgs::Odometry &msg) {
    //ros::Time now = ros::Time::now();
    //ros::Duration diff(now - msg.header.stamp);
    //ROS_INFO("delay vicon:  %d.%d",diff.sec,diff.nsec);

    if (transform_to_world_frame || publish_vicon) {
//        Eigen::Quaterniond orientationQ(msg.pose.pose.orientation.w,
//                                        msg.pose.pose.orientation.x,
//                                        msg.pose.pose.orientation.y,
//                                        msg.pose.pose.orientation.z);
//        Eigen::Matrix3d orientationR = orientationQ.toRotationMatrix();
//        R_WB_ = {orientationR(0,0),orientationR(0,1),orientationR(0,2),
//                orientationR(1,0),orientationR(1,1),orientationR(1,2),
//                orientationR(2,0),orientationR(2,1),orientationR(2,2)};

        R_WB_ = {1 - 2 * msg.pose.pose.orientation.y * msg.pose.pose.orientation.y -
                 2 * msg.pose.pose.orientation.z * msg.pose.pose.orientation.z,
                 2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.y -
                 2 * msg.pose.pose.orientation.z * msg.pose.pose.orientation.w,
                 2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.z +
                 2 * msg.pose.pose.orientation.y * msg.pose.pose.orientation.w,
                 2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.y +
                 2 * msg.pose.pose.orientation.z * msg.pose.pose.orientation.w,
                 1 - 2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.x -
                 2 * msg.pose.pose.orientation.z * msg.pose.pose.orientation.z,
                 2 * msg.pose.pose.orientation.y * msg.pose.pose.orientation.z -
                 2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.w,
                 2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.z -
                 2 * msg.pose.pose.orientation.y * msg.pose.pose.orientation.w,
                 2 * msg.pose.pose.orientation.y * msg.pose.pose.orientation.z +
                 2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.w,
                 1 - 2 * msg.pose.pose.orientation.x * msg.pose.pose.orientation.x -
                 2 * msg.pose.pose.orientation.y * msg.pose.pose.orientation.y};
        //R_WB_queue_.push(R_WB_);
        //time_R_WB_queue_.push(time_new_vicon_quad_);

//        ROS_INFO("R_WB1 = \n%f %f %f\n%f %f %f\n%f %f %f",
//                 orientationR(0,0),orientationR(0,1),orientationR(0,2),
//                 orientationR(1,0),orientationR(1,1),orientationR(1,2),
//                 orientationR(2,0),orientationR(2,1),orientationR(2,2));

    }

    time_new_vicon_quad_ = msg.header.stamp;
    time_diff_vicon = (time_new_vicon_quad_ - time_new_vicon_payload_);

    if (publish_vicon) {
//        if (std::abs(time_diff_vicon.toSec()) < 1.0 / 75.0)
//            ROS_INFO("R_WB %f", 1 / (time_new_vicon_quad_ - time_old_vicon_quad_));
//        else
//            ROS_INFO("R_WB -");
//        time_old_vicon_quad_ = time_new_vicon_quad_;

        vicon_quad_pos_ = {msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};
        vicon_quad_vel_ = {msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z};
        vicon_quad_angVel_ = {msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z};

        if (std::abs(time_diff_vicon.toSec()) < 1.5*0.01)
            vicon_publish_msg(msg.header);
    }


    //ros::Duration diff2(now2 - msg.header.stamp);
    //ros::Duration diff3(ros::Time::now() - now);
    //ROS_INFO("delay after calc vicon:  %d.%d",diff2.sec,diff2.nsec);
    //ROS_INFO("calculation time vicon:  %d.%d",diff3.sec,diff3.nsec);
}

void whycon::WhyConROS::vicon_payload_callback(const nav_msgs::Odometry &msg) {
    if (publish_vicon) {
        time_new_vicon_payload_ = msg.header.stamp;
        vicon_payload_pos_ = {msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};
        vicon_payload_vel_ = {msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z};

        if (std::abs(time_diff_vicon.toSec()) < 1.5*0.01)
            vicon_publish_msg(msg.header);
    }
}

void whycon::WhyConROS::vicon_publish_msg(const std_msgs::Header_<std::allocator<void>> &header) {

    //ROS_INFO("t: %f", std::abs(time_new_vicon_payload_ - time_new_vicon_quad_));
    nav_msgs::Odometry payload_vicon;

    // calculate relative position
    //ROS_INFO("vicon_payload_pos_: %f %f %f",vicon_payload_pos_(0),vicon_payload_pos_(1),vicon_payload_pos_(2));
    //ROS_INFO("vicon_quad_pos_: %f %f %f",vicon_quad_pos_(0),vicon_quad_pos_(1),vicon_quad_pos_(2));
    cv::Vec3f relative_pos_outputFrame;
    if (transform_to_world_frame) // in world frame
        relative_pos_outputFrame = vicon_payload_pos_ - vicon_quad_pos_;
    else // in body frame
        relative_pos_outputFrame = R_WB_.t() * (vicon_payload_pos_ - vicon_quad_pos_);
    //ROS_INFO("Vec1: %f %f %f",tempVec(0),tempVec(1),tempVec(2));
    relative_pos_outputFrame =
            (cable_length + distance_tag_CoG) * relative_pos_outputFrame / cv::norm(relative_pos_outputFrame);
    //ROS_INFO("Vec2: %f %f %f",tempVec(0),tempVec(1),tempVec(2));
    payload_vicon.pose.pose.position.x = relative_pos_outputFrame(0);
    payload_vicon.pose.pose.position.y = relative_pos_outputFrame(1);
    payload_vicon.pose.pose.position.z = relative_pos_outputFrame(2);

    // calculate relative angle
    payload_vicon.pose.pose.orientation.x =  atan2(relative_pos_outputFrame(1),-relative_pos_outputFrame(2));
    payload_vicon.pose.pose.orientation.y = -atan2(relative_pos_outputFrame(0),-relative_pos_outputFrame(2));
    payload_vicon.pose.pose.orientation.z = 0;
    payload_vicon.pose.pose.orientation.w = 0;

    // calculate relative velocity
    cv::Vec3f relative_vel_outputFrame;
    if (transform_to_world_frame) { // in world frame
        relative_vel_outputFrame = vicon_payload_vel_ - vicon_quad_vel_;
    } else { // in body frame
        relative_vel_outputFrame = (R_WB_.t() * (vicon_payload_vel_ - vicon_quad_vel_)) + \
                                   (R_WB_.t() * (vicon_payload_pos_ - vicon_quad_pos_).cross(vicon_quad_angVel_));
    }
    // payload_vicon.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, payload_vicon.rot(0), payload_vicon.rot(1));
    payload_vicon.twist.twist.linear.x = relative_vel_outputFrame(0);
    payload_vicon.twist.twist.linear.y = relative_vel_outputFrame(1);
    payload_vicon.twist.twist.linear.z = relative_vel_outputFrame(2);

    // calculate relative angular velocity
    cv::Vec3d relative_ang_vel;
    relative_ang_vel = (vicon_payload_pos_ - vicon_quad_pos_).cross(vicon_payload_vel_ - vicon_quad_vel_) /
                       (cable_length + distance_tag_CoG)*(cable_length + distance_tag_CoG);
    if (!transform_to_world_frame)
        relative_ang_vel = R_WB_ * relative_ang_vel;
    //{cv::Vec3d(1,0,0).cross(relative_pos_outputFrame).dot(relative_vel_outputFrame)/cv::norm(relative_pos_outputFrame),
    //          cv::Vec3d(0,1,0).cross(relative_pos_outputFrame).dot(relative_vel_outputFrame)/cv::norm(relative_pos_outputFrame),
    //        cv::Vec3d(0,0,1).cross(relative_pos_outputFrame).dot(relative_vel_outputFrame)/cv::norm(relative_pos_outputFrame)};
    //if (!transform_to_world_frame)
    //    relative_ang_vel = R_WB_.t()*relative_ang_vel;

    payload_vicon.twist.twist.angular.x = relative_ang_vel(0);
    payload_vicon.twist.twist.angular.y = relative_ang_vel(1);
    payload_vicon.twist.twist.angular.z = 0;

    payload_vicon.header = header;
    //payload_vicon.header.frame_id = msg.header.frame_id;
    odom_vicon_pub.publish(payload_vicon);
}
