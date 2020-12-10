//
// Created by hossein on 12/10/20.
//

#ifndef USER_INTERFACE__USER_INTERFACE_H_
#define USER_INTERFACE__USER_INTERFACE_H_
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"

class UserInterface : public rclcpp::Node {
 public:
  explicit UserInterface(rclcpp::NodeOptions options);
 private:
  std::vector<cv::Mat> original_frames_;
  std::vector<cv::Mat> segmented_frames;
  const uint number_of_cameras{2};
  const uint image_width{640};
  const uint image_height{480};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_1_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_2_subscriber_;

  void original_image_1_callback(const sensor_msgs::msg::Image::ConstSharedPtr image);
  void original_image_2_callback(const sensor_msgs::msg::Image::ConstSharedPtr image);
  static void mouse_callback_1(int event, int x, int y, int flags, void *user_data);
  static void mouse_callback_2(int event, int x, int y, int flags, void *user_data);
  void convert_pixel_crd_to_world_crd(uint pixel_x, uint pixel_y, double &world_x, double &world_y);
  void user_interface(uint pixel_x, uint pixel_y, uint cam_index);
};

#endif //USER_INTERFACE__USER_INTERFACE_H_
