//
// Created by hossein on 9/9/20.
//

#ifndef GLOBAL_PLANNER__GLOBALPLANNER_H_
#define GLOBAL_PLANNER__GLOBALPLANNER_H_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "custom_msg_srv/srv/image_batch.hpp"
#include "tf2_ros/create_timer_ros.h"
#include <chrono>
#include <iostream>
#include "global_planner/NF2.h"

class GlobalPlanner : public rclcpp::Node {
 public:
  explicit GlobalPlanner(rclcpp::NodeOptions options);
 private:
  bool debug_ = true;
  const uint number_of_cameras_{3};
  const uint image_width_{224*3};
  const uint image_height_{224};
  const uint grid_resolution_{10};

  std::unique_ptr<NF2> nf2_instance;

  std::vector<cv::Point> cameras_pose_;
  cv::Mat stitched_image_;

  rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedPtr segmented_images_client_; //TODO change to sensor_msgs
  rclcpp::TimerBase::SharedPtr update_global_planner_timer;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_goal_ui_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_pose_ui_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_in_grid_publisher_;
  void publish_path_message(std::vector<cv::Point> path);
  void make_request_for_segmented_images();
  void response_received_callback(rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedFuture result_future);
  void robot_goal_ui_callback(geometry_msgs::msg::Point::ConstSharedPtr goal_point);
  void robot_pose_ui_callback(geometry_msgs::msg::Point::ConstSharedPtr pose_point);

  cv::Point robot_goal_;
  cv::Point robot_pose_;

  bool is_goal_set{false};
  bool is_pose_set{false};
};

#endif //GLOBAL_PLANNER__GLOBALPLANNER_H_
