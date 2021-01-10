//
// Created by hossein on 12/7/20.
//

#include "../include/overhead_camera_service/overhead_camera_node.h"

OverheadCamNode::OverheadCamNode(rclcpp::NodeOptions options): Node("overhead_camera_node", options){

  // define message_filter::subscribers to multiple sources
  // a message_filter::subscriber to start subscribing must be registered
  overhead_cam1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/overhead_cam_1/camera/image_raw", rmw_qos_profile_sensor_data);
  overhead_cam2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/overhead_cam_2/camera/image_raw", rmw_qos_profile_sensor_data);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image , sensor_msgs::msg::Image> approximate_policy;
  message_filters::Synchronizer<approximate_policy>* syncApproximate= new message_filters::Synchronizer<approximate_policy>(approximate_policy(10), *overhead_cam1_sub_, *overhead_cam2_sub_);

  syncApproximate->registerCallback(std::bind(&OverheadCamNode::synchronized_images_callback, this,std::placeholders::_1, std::placeholders::_2));


  // this service is called from image_segmentation package's client
  service_ = this->create_service<custom_msg_srv::srv::ImageBatch>("autonomous_robot/overhead_camera_service",
                                                                   std::bind(&OverheadCamNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Overhead camera node starts working ...");
}


void OverheadCamNode::synchronized_images_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_1,
                                                   const sensor_msgs::msg::Image::ConstSharedPtr image_2) {

  image_1_.height=image_1->height;
  image_1_.width=image_1->width;
  image_1_.header=image_1->header;
  image_1_.step=image_1->step;
  image_1_.encoding=image_1->encoding;
  image_1_.is_bigendian=image_1->is_bigendian;
  image_1_.data = image_1->data;

  image_2_.height=image_2->height;
  image_2_.width=image_2->width;
  image_2_.header=image_2->header;
  image_2_.step=image_2->step;
  image_2_.encoding=image_2->encoding;
  image_2_.is_bigendian=image_2->is_bigendian;
  image_2_.data = image_2->data;

}
void OverheadCamNode::handle_service(const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Request> request,
                                     const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Overhead camera node received a request ...");
  response->image[0] = image_1_;
  response->image[1] = image_2_;
}
