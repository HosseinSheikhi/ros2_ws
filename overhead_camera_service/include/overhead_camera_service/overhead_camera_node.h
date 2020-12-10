//
// Created by hossein on 12/7/20.
//

#ifndef OVERHEAD_CAME_NODE__OVERHEADCAMSRV_H_
#define OVERHEAD_CAME_NODE__OVERHEADCAMSRV_H_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "custom_msg_srv/srv/image_batch.hpp"

class OverheadCamNode : public rclcpp::Node{
 public:
  OverheadCamNode(rclcpp::NodeOptions options);

 private:
  const uint number_of_cameras{2};
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> overhead_cam1_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> overhead_cam2_sub_;
  void synchronized_images_callback(const sensor_msgs::msg::Image::ConstSharedPtr image_1, const sensor_msgs::msg::Image ::ConstSharedPtr image_2);

  sensor_msgs::msg::Image image_1_;
  sensor_msgs::msg::Image image_2_;

  void handle_service(const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Request> request,
                      const std::shared_ptr<custom_msg_srv::srv::ImageBatch::Response> response);
  rclcpp::Service<custom_msg_srv::srv::ImageBatch>::SharedPtr service_;
};

#endif //OVERHEAD_CAME_NODE__OVERHEADCAMSRV_H_
