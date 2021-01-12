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
  const uint number_of_cameras_{2};
  const uint image_width_{640};
  const uint image_height_{480};
  const uint grid_resolution_{32};

  std::shared_ptr<NF2> nf2_instance;
  // variables for subscribing images
  std::vector<cv::Point> cameras_pose_;
  std::vector<cv::Mat> original_frames_;

  cv::Point goal_{1,20}; // TODO, if I use (15,20) gives error, why?
  cv::Point robot_pose_{30,20};
  rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedPtr segmented_images_client_;
  rclcpp::TimerBase::SharedPtr update_planning_timer_;


  void show_images( std::string window_name,  std::vector<cv::Mat> images);
  void make_request_for_segmented_images();
  void response_received_callback(rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedFuture result_future);
};

#endif //GLOBAL_PLANNER__GLOBALPLANNER_H_
