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

namespace nav2_overhead {
class OverheadCamNode : public rclcpp::Node {
 public:
  OverheadCamNode(rclcpp::NodeOptions options, uint overhead_cameras_num);

 private:
  uint overhead_cameras_num_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> overhead_cameras_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> overhead_cam1_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> overhead_cam2_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> overhead_cam3_sub_;

  void synchronized_images_callback(sensor_msgs::msg::Image::ConstSharedPtr image_1,
                                    sensor_msgs::msg::Image::ConstSharedPtr image_2,
                                    sensor_msgs::msg::Image::ConstSharedPtr image_3);

  sensor_msgs::msg::Image image_1_;
  sensor_msgs::msg::Image image_2_;
  sensor_msgs::msg::Image image_3_;

  void copy_image(sensor_msgs::msg::Image::ConstSharedPtr from, sensor_msgs::msg::Image &to  );

  void handle_service(std::shared_ptr<custom_msg_srv::srv::ImageBatch::Request> request,
                      std::shared_ptr<custom_msg_srv::srv::ImageBatch::Response> response);
  rclcpp::Service<custom_msg_srv::srv::ImageBatch>::SharedPtr service_;
};
}

#endif //OVERHEAD_CAME_NODE__OVERHEADCAMSRV_H_
