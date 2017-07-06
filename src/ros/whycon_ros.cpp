#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <whycon/Projection.h>
#include "whycon_ros.h"

whycon::WhyConROS::WhyConROS(ros::NodeHandle &n) : is_tracking(false), should_reset(true), it(n) {
  transformation_loaded = false;
  similarity.setIdentity();

  if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");

//  std::string vicon_quad_topic, vicon_payload_topic;
//  n.param("vicon_quad_topic", vicon_quad_topic, std::string(""));
//  n.param("vicon_payload_topic", vicon_payload_topic, std::string(""));
  n.param("name", frame_id, std::string("whycon"));
  n.param("world_frame", world_frame_id, std::string("world"));
  n.param("max_attempts", max_attempts, 1);
  n.param("max_refine", max_refine, 1);
  n.param("B_T_BC", B_T_BC_yaml, {0, 0, 0});
  n.param("R_BC", R_BC_yaml, {1, 0, 0, 0, 1, 0, 0, 0, 1});
  n.param("cable_length", cable_length_, 0.3);
  n.param("distance_tag_CoG", distance_tag_CoG_, 0.0);
  n.param("use_omni_model", use_omni_model, false);
  n.param("calib_file", calib_file, std::string(""));
  n.param("publish_tf", publish_tf, false);
  n.param("transform_to_world_frame", transform_to_world_frame, false);
  n.param("filter_velocities", filter_velocities, false);

  B_T_BC_ = cv::Vec3d(B_T_BC_yaml[0], B_T_BC_yaml[1], B_T_BC_yaml[2]);
  R_BC_ = cv::Matx33d(R_BC_yaml[0], R_BC_yaml[1], R_BC_yaml[2], \
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
  odom_pub = n.advertise<payload_msgs::PayloadOdom>("odom_whycon", 1);
  odom_payload_pub = n.advertise<nav_msgs::Odometry>("odom_payload_whycon", 1);
  relative_pos_pub = n.advertise<geometry_msgs::Vector3Stamped>("relative_pos", 1);

  reset_service = n.advertiseService("reset", &WhyConROS::reset, this);

  //omni cam model
  get_ocam_model(&model, strdup(calib_file.c_str()));
}

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr &image_msg,
                                 const sensor_msgs::CameraInfoConstPtr &info_msg) {
  bool publish_images = (image_pub.getNumSubscribers() != 0);

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
    cv::Mat output_image;
    if (publish_images)
      output_image = cv_ptr->image.clone();

    for (int id = 0; id < system->targets; id++) {
      double point3D[3];
      double point2D[2] = {system->get_circle(id).y, system->get_circle(id).x};
      cam2world(point3D, point2D, &model);

      // adapt coord sys of ocam to coord sys of whycon
      cv::Vec3d direction_camFrame = {point3D[1],
                                      point3D[0],
                                     -point3D[2]};
      // rotate from camera frame to quad frame
      cv::Vec3d direction_bodyFrame = R_BC_ * direction_camFrame;

      // calculate distance camera-load
      double b_term = 2 * B_T_BC_.dot(direction_bodyFrame);
      double c_term = B_T_BC_.dot(B_T_BC_) - std::pow(cable_length_-distance_tag_CoG_,2);
      double dist = (-b_term + sqrt(b_term * b_term - 4 * c_term)) / 2;

      // extend vector from quad origin to center of gravity of the load
      cv::Vec3d relative_position_bodyFrame = (B_T_BC_ + dist * direction_bodyFrame) * //calculate direction in quad frame
                                               cable_length_ / (cable_length_ - distance_tag_CoG_); //adjust length

      // publish direction
      geometry_msgs::Vector3Stamped p;
      p.vector.x = relative_position_bodyFrame(0);
      p.vector.y = relative_position_bodyFrame(1);
      p.vector.z = relative_position_bodyFrame(2);
      p.header = image_msg->header;
      p.header.frame_id = std::to_string(0);
      relative_pos_pub.publish(p);

      // draw each target
      if (publish_images) {
        std::ostringstream ostr;
        ostr << std::fixed << std::setprecision(2);
        //ostr << coord << " " << id;
        if (system->targets > 1)
          ostr << " id: " << id;
        else
          ostr << " ";
        system->get_circle(id).draw(output_image, ostr.str(), cv::Vec3b(0, 255, 255));
        cv::circle(output_image,camera_model.project3dToPixel(cv::Point3d(coord)),1,cv::Scalar(255,0,255),1,CV_AA);
      }
    }

    if (publish_images) {
      cv_bridge::CvImage output_image_bridge = *cv_ptr;
      output_image_bridge.image = output_image;
      image_pub.publish(output_image_bridge);
    }

    should_reset = false;

  } else if (publish_images)
    image_pub.publish(cv_ptr);

}

void whycon::WhyConROS::publish_odom(const nav_msgs::OdometryConstPtr& msg_quad,
                                       const geometry_msgs::Vector3StampedConstPtr& msg_relPos_bodyFrame) {

  bool publish_odom_whycon = (odom_pub.getNumSubscribers() != 0);
  bool publish_odom_payload_whycon = (odom_payload_pub.getNumSubscribers() != 0);

  // calculate rotation and angular velocity from vicon data
  cv::Matx33d R_WB =
    {1 - 2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.y -
         2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.z,
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.y -
         2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.w,
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.z +
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.w,
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.y +
         2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.w,
     1 - 2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.x -
         2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.z,
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.z -
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.w,
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.z -
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.w,
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.z +
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.w,
     1 - 2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.x -
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.y};
  cv::Vec3d vicon_quad_angVel_bodyFrame =
    {msg_quad->twist.twist.angular.x,
     msg_quad->twist.twist.angular.y,
     msg_quad->twist.twist.angular.z};
  cv::Vec3d vicon_quad_velocity =
    {msg_quad->twist.twist.linear.x,
     msg_quad->twist.twist.linear.y,
     msg_quad->twist.twist.linear.z};
  cv::Vec3d vicon_quad_position =
    {msg_quad->pose.pose.position.x,
     msg_quad->pose.pose.position.y,
     msg_quad->pose.pose.position.z};

  // calculate time difference between Vicon and Camera frame
  double time_diff_vw  ( ros::Duration(msg_relPos_bodyFrame->header.stamp - msg_quad->header.stamp).toSec() );
  double time_diff_io  ( ros::Duration(ros::Time::now() - msg_relPos_bodyFrame->header.stamp).toSec() );
  double time_diff_last( ros::Duration(msg_relPos_bodyFrame->header.stamp - time_old_whycon).toSec() );

  num_meas++;
  if (time_diff_vw > 1) {//0.5/150.0) {
    dropped_frames++;
    ROS_INFO("----- dropped frame ----- %f -----", dropped_frames/num_meas);
    return;
  }

//  // calculate mean time difference Vicon/Camera
//  avg = (avg * (num_meas - 1) + std::abs(time_diff_vw) )/num_meas;
//  ROS_INFO("w-v: avg    %f", avg);
//  ROS_INFO("t-diff vw   %f", time_diff_vw);
//  ROS_INFO("duration IO %f", time_diff_io);
//  ROS_INFO("since last  %f", time_diff_last);

  cv::Vec3d whycon_relativePosition_bodyFrame = {msg_relPos_bodyFrame->vector.x,
                                                 msg_relPos_bodyFrame->vector.y,
                                                 msg_relPos_bodyFrame->vector.z};

  // calculate position for output
  cv::Vec3d whycon_relativePosition_outputFrame;
  cv::Vec3d whycon_position_outputFrame;
  if (!transform_to_world_frame) { //quad frame
    whycon_relativePosition_outputFrame = whycon_relativePosition_bodyFrame;
    whycon_position_outputFrame = whycon_relativePosition_bodyFrame;
  } else { //world frame
    whycon_relativePosition_outputFrame = R_WB * whycon_relativePosition_bodyFrame;
    whycon_position_outputFrame = whycon_relativePosition_outputFrame + vicon_quad_position;
  }

  // calculate angle for output
  cv::Vec3d whycon_angle_outputFrame = { atan2(whycon_relativePosition_outputFrame(1), -whycon_relativePosition_outputFrame(2)),
                                        -atan2(whycon_relativePosition_outputFrame(0), -whycon_relativePosition_outputFrame(2)),
                                         0.0};

  // calculate velocity for output
  cv::Vec3d whycon_velocity_bodyFrame;
  cv::Vec3d whycon_velocity_worldFrame;
  cv::Vec3d whycon_velocity_outputFrame;
  cv::Vec3d whycon_velocity_outputFrame_filtered;
  cv::Vec3d whycon_angVel_outputFrame_filtered;
  if (!time_old_whycon.isZero()) {
    whycon_velocity_worldFrame = (whycon_relativePosition_outputFrame - whycon_position_outputFrame_old) / time_diff_last;
    whycon_velocity_bodyFrame  = (whycon_relativePosition_bodyFrame   - whycon_position_bodyFrame_old)   / time_diff_last;

    if (transform_to_world_frame)
      whycon_velocity_outputFrame = vicon_quad_velocity +
                                    whycon_velocity_bodyFrame +
                                   (R_WB * vicon_quad_angVel_bodyFrame).cross(whycon_relativePosition_bodyFrame);
    else
      whycon_velocity_outputFrame = whycon_velocity_bodyFrame;

    whycon_velocity_u = whycon_velocity_u * cv::Matx44d(0,1,0,0,  0,0,1,0,  0,0,0,1,  0,0,0,0);
    whycon_velocity_y = whycon_velocity_y * cv::Matx44d(0,1,0,0,  0,0,1,0,  0,0,0,1,  0,0,0,0);

    whycon_velocity_u(0,0) = whycon_velocity_outputFrame(0);
    whycon_velocity_u(1,0) = whycon_velocity_outputFrame(1);
    whycon_velocity_u(2,0) = whycon_velocity_outputFrame(2);

    // fill in new data
    whycon_velocity_outputFrame_filtered = whycon_velocity_u * filter_B - whycon_velocity_y.get_minor<3,3>(0,1) * filter_A;
    whycon_velocity_y(0,0) = whycon_velocity_outputFrame_filtered(0);
    whycon_velocity_y(1,0) = whycon_velocity_outputFrame_filtered(1);
    whycon_velocity_y(2,0) = whycon_velocity_outputFrame_filtered(2);
  }


  // calculate angular velocity for output
  cv::Vec3d whycon_angVel_outputFrame;
  if (!time_old_whycon.isZero()) {
    if (!transform_to_world_frame) {
      whycon_angVel_outputFrame = (whycon_angle_outputFrame - whycon_angle_outputFrame_old) /
                                  (time_diff_last);
    } else {
      // Version 1
      //whycon_angVel_outputFrame = (whycon_angle_outputFrame - whycon_angle_outputFrame_old) /
      //                            (time_diff_last);
      // Version 2
      //whycon_angVel_outputFrame = whycon_position_outputFrame.cross(whycon_velocity_outputFrame) /
      //                            whycon_position_outputFrame.dot(whycon_position_outputFrame);
      // Version 3
      cv::Vec3d whycon_angVel_bodyFrame = whycon_relativePosition_bodyFrame.cross( whycon_velocity_bodyFrame )/
                                          whycon_relativePosition_bodyFrame.dot(whycon_relativePosition_bodyFrame);
      whycon_angVel_outputFrame = R_WB * (
                                  whycon_angVel_bodyFrame +
                                  vicon_quad_angVel_bodyFrame );
      // Version 4
      //whycon_angVel_outputFrame = R_WB *
      //                            (R_WB.t() * whycon_angle_outputFrame - R_WBold.t() * whycon_angle_outputFrame_old) /
      //                            (time_diff_last) +
      //                            vicon_quad_angVel_;

      // up to third order butterwroth low-pass filter
      // update input matrix u and output matrix y
      whycon_angVel_u = whycon_angVel_u * cv::Matx44d(0,1,0,0,  0,0,1,0,  0,0,0,1,  0,0,0,0);
      whycon_angVel_y = whycon_angVel_y * cv::Matx44d(0,1,0,0,  0,0,1,0,  0,0,0,1,  0,0,0,0);

      // fill in new data
      whycon_angVel_u(0,0) = whycon_angVel_outputFrame(0);
      whycon_angVel_u(1,0) = whycon_angVel_outputFrame(1);
      whycon_angVel_u(2,0) = whycon_angVel_outputFrame(2);

      whycon_angVel_outputFrame_filtered = whycon_angVel_u * filter_B - whycon_angVel_y.get_minor<3,3>(0,1) * filter_A;
      whycon_angVel_y(0,0) = whycon_angVel_outputFrame_filtered(0);
      whycon_angVel_y(1,0) = whycon_angVel_outputFrame_filtered(1);
      whycon_angVel_y(2,0) = whycon_angVel_outputFrame_filtered(2);

      // FILTERING
      // first order low-pass filter
      /*if (filter_a == 0.0) {
        double filter_h = 0.02;
        double filter_T = 0.1;
        filter_a = filter_h/(filter_T-filter_h);
      }

      whycon_angVel_outputFrame = filter_a  * whycon_angVel_outputFrame +
                               (1-filter_a) * whycon_angVel_outputFrame_old;
      whycon_angVel_outputFrame_old = whycon_angVel_outputFrame;
      //whycon_angVel_outputFrame_old2 = whycon_angVel_outputFrame_old;*/
    }
  }
  else
    whycon_angVel_outputFrame = {0.0, 0.0, 0.0};

  whycon_position_outputFrame_old = whycon_relativePosition_outputFrame;
  whycon_position_bodyFrame_old = whycon_relativePosition_bodyFrame;
  time_old_whycon = msg_relPos_bodyFrame->header.stamp;
  whycon_angle_outputFrame_old = whycon_angle_outputFrame;




  // publish results
  if (publish_odom_whycon) {
    payload_msgs::PayloadOdom odom;
    if (transform_to_world_frame) { //output absolute, not relative position
      odom.pose_payload.pose.position.x = whycon_position_outputFrame(0);
      odom.pose_payload.pose.position.y = whycon_position_outputFrame(1);
      odom.pose_payload.pose.position.z = whycon_position_outputFrame(2);
    } else {
      odom.pose_payload.pose.position.x = whycon_relativePosition_outputFrame(0);
      odom.pose_payload.pose.position.y = whycon_relativePosition_outputFrame(1);
      odom.pose_payload.pose.position.z = whycon_relativePosition_outputFrame(2);
    }

    if (filter_velocities) {
      odom.twist_payload.twist.linear.x = whycon_velocity_outputFrame_filtered(0);
      odom.twist_payload.twist.linear.y = whycon_velocity_outputFrame_filtered(1);
      odom.twist_payload.twist.linear.z = whycon_velocity_outputFrame_filtered(2);
    } else {
      odom.twist_payload.twist.linear.x = whycon_velocity_outputFrame(0);
      odom.twist_payload.twist.linear.y = whycon_velocity_outputFrame(1);
      odom.twist_payload.twist.linear.z = whycon_velocity_outputFrame(2);
    }

    odom.pose_payload.pose.orientation.x = whycon_angle_outputFrame(0);
    odom.pose_payload.pose.orientation.y = whycon_angle_outputFrame(1);
    odom.pose_payload.pose.orientation.z = whycon_angle_outputFrame(2);
    odom.pose_payload.pose.orientation.w = std::sqrt(1-std::pow(whycon_angle_outputFrame(0),2)-std::pow(whycon_angle_outputFrame(1),2)-std::pow(whycon_angle_outputFrame(2),2));

    if (filter_velocities) {
      odom.twist_payload.twist.angular.x = whycon_angVel_outputFrame_filtered(0);
      odom.twist_payload.twist.angular.y = whycon_angVel_outputFrame_filtered(1);
      odom.twist_payload.twist.angular.z = whycon_angVel_outputFrame_filtered(2);
    } else {
      odom.twist_payload.twist.angular.x = whycon_angVel_outputFrame(0);
      odom.twist_payload.twist.angular.y = whycon_angVel_outputFrame(1);
      odom.twist_payload.twist.angular.z = whycon_angVel_outputFrame(2);
    }

    odom.pose_quad = msg_quad->pose;
    odom.twist_quad = msg_quad->twist;

    odom.header = msg_relPos_bodyFrame->header;
    odom_pub.publish(odom);
  }

  if (publish_odom_payload_whycon) {
    nav_msgs::Odometry odom;

    if (transform_to_world_frame) { //output absolute, not relative position
      odom.pose.pose.position.x = whycon_position_outputFrame(0);
      odom.pose.pose.position.y = whycon_position_outputFrame(1);
      odom.pose.pose.position.z = whycon_position_outputFrame(2);
    } else {
      odom.pose.pose.position.x = whycon_relativePosition_outputFrame(0);
      odom.pose.pose.position.y = whycon_relativePosition_outputFrame(1);
      odom.pose.pose.position.z = whycon_relativePosition_outputFrame(2);
    }

    if (filter_velocities) {
      odom.twist.twist.linear.x = whycon_velocity_outputFrame_filtered(0);
      odom.twist.twist.linear.y = whycon_velocity_outputFrame_filtered(1);
      odom.twist.twist.linear.z = whycon_velocity_outputFrame_filtered(2);
    } else {
      odom.twist.twist.linear.x = whycon_velocity_outputFrame(0);
      odom.twist.twist.linear.y = whycon_velocity_outputFrame(1);
      odom.twist.twist.linear.z = whycon_velocity_outputFrame(2);
    }

    odom.pose.pose.orientation.x = whycon_angle_outputFrame(0);
    odom.pose.pose.orientation.y = whycon_angle_outputFrame(1);
    odom.pose.pose.orientation.z = whycon_angle_outputFrame(2);
    odom.pose.pose.orientation.w = std::sqrt(1-std::pow(whycon_angle_outputFrame(0),2)-std::pow(whycon_angle_outputFrame(1),2)-std::pow(whycon_angle_outputFrame(2),2));

    if (filter_velocities) {
      odom.twist.twist.angular.x = whycon_angVel_outputFrame_filtered(0);
      odom.twist.twist.angular.y = whycon_angVel_outputFrame_filtered(1);
      odom.twist.twist.angular.z = whycon_angVel_outputFrame_filtered(2);
    } else {
      odom.twist.twist.angular.x = whycon_angVel_outputFrame(0);
      odom.twist.twist.angular.y = whycon_angVel_outputFrame(1);
      odom.twist.twist.angular.z = whycon_angVel_outputFrame(2);
    }

    odom.header = msg_relPos_bodyFrame->header;
    odom_payload_pub.publish(odom);
  }
}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  should_reset = true;
  return true;
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
