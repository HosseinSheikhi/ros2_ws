//
// Created by hossein on 12/10/20.
//

#ifndef USER_INTERFACE__USER_INTERFACE_H_
#define USER_INTERFACE__USER_INTERFACE_H_
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"

class UserInterface : public rclcpp::Node {
 public:
  explicit UserInterface(rclcpp::NodeOptions options);
 private:
  std::vector<cv::Mat> original_frames_;
  std::vector<cv::Mat> segmented_frames_;
  const uint number_of_cameras_{2};
  std::vector<cv::Point> cameras_pose_;
  const uint image_width_{640};
  const uint image_height_{480};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_1_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_2_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_publisher_;
  void original_image_1_callback(const sensor_msgs::msg::Image::ConstSharedPtr image);
  void original_image_2_callback(const sensor_msgs::msg::Image::ConstSharedPtr image);
  static void mouse_callback_1(int event, int x, int y, int flags, void *user_data);
  static void mouse_callback_2(int event, int x, int y, int flags, void *user_data);
  void convert_pixel_crd_to_world_crd(uint pixel_x, uint pixel_y, double &world_x, double &world_y);
  void user_interface(uint pixel_x, uint pixel_y, uint cam_index);
};

#endif //USER_INTERFACE__USER_INTERFACE_H_
