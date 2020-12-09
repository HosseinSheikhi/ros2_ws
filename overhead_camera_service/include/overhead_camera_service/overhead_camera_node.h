//
// Created by hossein on 12/7/20.
//

#ifndef OVERHEAD_CAME_NODE__OVERHEADCAMSRV_H_
#define OVERHEAD_CAME_NODE__OVERHEADCAMSRV_H_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "custom_msg_srv/srv/image_batch.hpp"

class OverheadCamNode : public rclcpp::Node{
 public:
  OverheadCamNode(rclcpp::NodeOptions options);

 private:
  const uint number_of_cameras{2};
  //message_filters::Subscriber<sensor_msgs::msg::Image> overhead_cam1_sub_;
  //message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  //void synchronized_images_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_1, const sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info);

  sensor_msgs::msg::Image image_1_;
  sensor_msgs::msg::Image image_2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr overhead_cam1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr overhead_cam2_sub_;

  void image_1_callback(const sensor_msgs::msg::Image::ConstSharedPtr img);
  void image_2_callback(const sensor_msgs::msg::Image::ConstSharedPtr img);


  void handle_service(const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Request> request,
                      const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Response> response);
  rclcpp::Service<custom_msg_srv::srv::ImageBatch>::SharedPtr service_;
};

#endif //OVERHEAD_CAME_NODE__OVERHEADCAMSRV_H_
