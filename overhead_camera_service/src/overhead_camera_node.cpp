//
// Created by hossein on 12/7/20.
//

#include "../include/overhead_camera_service/overhead_camera_node.h"

nav2_overhead::OverheadCamNode::OverheadCamNode(rclcpp::NodeOptions options, uint overhead_cameras_num):
Node("overhead_camera_node", options), overhead_cameras_num_(overhead_cameras_num)
{
  // define message_filter::subscribers to multiple sources
  // a message_filter::subscriber to start subscribing must be registered
  overhead_cam1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this,
      "/overhead_cam_1/camera/image_raw", rmw_qos_profile_sensor_data);
  overhead_cam2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this,
      "/overhead_cam_2/camera/image_raw", rmw_qos_profile_sensor_data);
  overhead_cam3_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this,
      "/overhead_cam_3/camera/image_raw", rmw_qos_profile_sensor_data);

  using approximate_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                              sensor_msgs::msg::Image,
                                                                              sensor_msgs::msg::Image>;

  message_filters::Synchronizer<approximate_policy>* syncApproximate=
      new message_filters::Synchronizer<approximate_policy>(approximate_policy(10),
                                                            *overhead_cam1_sub_,
                                                            *overhead_cam2_sub_,
                                                            *overhead_cam3_sub_);

  syncApproximate->registerCallback(std::bind(&OverheadCamNode::synchronized_images_callback, this,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3));


  // this service is called from image_segmentation package's client
  service_ = this->create_service<custom_msg_srv::srv::ImageBatch>("autonomous_robot/overhead_camera_service",
                                                                   std::bind(&OverheadCamNode::handle_service,
                                                                             this,
                                                                             std::placeholders::_1,
                                                                             std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Overhead camera node starts working ...");
}


// TODO: read about constSharePtr and see what warning is saying
void nav2_overhead::OverheadCamNode::synchronized_images_callback( sensor_msgs::msg::Image::ConstSharedPtr image_1,
                                                                   sensor_msgs::msg::Image::ConstSharedPtr image_2,
                                                                   sensor_msgs::msg::Image::ConstSharedPtr image_3)
                                                                  {

  copy_image(image_1, image_1_);
  copy_image(image_2, image_2_);
  copy_image(image_3, image_3_);
}


void nav2_overhead::OverheadCamNode::handle_service(const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Request> request,
                                     const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Overhead camera node received a request ...");
  response->image[0] = image_1_;
  response->image[1] = image_2_;
  response->image[2] = image_3_;
}


void nav2_overhead::OverheadCamNode::copy_image(const sensor_msgs::msg::Image::ConstSharedPtr from, sensor_msgs::msg::Image &to  ){
  to.height=from->height;
  to.width=from->width;
  to.header=from->header;
  to.step=from->step;
  to.encoding=from->encoding;
  to.is_bigendian=from->is_bigendian;
  to.data = from->data;

}
