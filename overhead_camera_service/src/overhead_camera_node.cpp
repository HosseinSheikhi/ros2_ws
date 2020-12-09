//
// Created by hossein on 12/7/20.
//

#include "../include/overhead_camera_service/overhead_camera_node.h"

OverheadCamNode::OverheadCamNode(rclcpp::NodeOptions options): Node("overhead_camera_node", options){

  /*
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;

  message_filters::Subscriber<sensor_msgs::msg::Image> overhead_cam1_sub_(this, "/image", custom_qos_profile);
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_(this, "/camera_info", custom_qos_profile);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image , sensor_msgs::msg::CameraInfo> approximate_policy;
  message_filters::Synchronizer<approximate_policy> syncApproximate(approximate_policy(10), overhead_cam1_sub_, camera_info_sub_);
  message_filters::TimeSynchronizer<sensor_msgs::msg::Image , sensor_msgs::msg::CameraInfo> sync( overhead_cam1_sub_, camera_info_sub_,10);
  syncApproximate.registerCallback(std::bind(&OverheadCamNode::synchronized_images_callback, this,std::placeholders::_1, std::placeholders::_2));
  sync.registerCallback(std::bind(&OverheadCamNode::synchronized_images_callback, this,std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Constructor");
  */

  overhead_cam1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/overhead_cam_1/camera/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&OverheadCamNode::image_1_callback, this, std::placeholders::_1)
      );
  overhead_cam2_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/overhead_cam_2/camera/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&OverheadCamNode::image_2_callback, this, std::placeholders::_1)
  );

  service_ = this->create_service<custom_msg_srv::srv::ImageBatch>("get_images",
                                                                   std::bind(&OverheadCamNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
}

void OverheadCamNode::handle_service(const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Request> request,
                                     const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Response> response) {

  response->image[0] = image_1_;
  response->image[1] = image_2_;
}

void OverheadCamNode::image_1_callback(const sensor_msgs::msg::Image::ConstSharedPtr img) {
  image_1_.height=img->height;
  image_1_.width=img->width;
  image_1_.header=img->header;
  image_1_.step=img->step;
  image_1_.encoding=img->encoding;
  image_1_.is_bigendian=img->is_bigendian;
  image_1_.data = img->data;
}
void OverheadCamNode::image_2_callback(const sensor_msgs::msg::Image::ConstSharedPtr img) {
  image_2_.height=img->height;
  image_2_.width=img->width;
  image_2_.header=img->header;
  image_2_.step=img->step;
  image_2_.encoding=img->encoding;
  image_2_.is_bigendian=img->is_bigendian;
  image_2_.data = img->data;
}






//void OverheadCamNode::synchronized_images_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_1,
//                                                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info) {
//
//  RCLCPP_INFO(this->get_logger(), "in CB");
//  cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_1, sensor_msgs::image_encodings::BGR8);
//  cv::Mat original_frame =cv_image_ptr->image;
//  auto temp =cam_info->height;
//  cv::imshow("overhead_camera"+temp,original_frame);
//  cv::waitKey(1);
//
//}