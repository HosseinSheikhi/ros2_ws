//
// Created by hossein on 9/9/20.
//

#include "../include/global_planner/GlobalPlanner.h"

using namespace std::chrono_literals;

GlobalPlanner::GlobalPlanner(rclcpp::NodeOptions options)
: Node("global_planner_node", options){

  // this client sends its request to image_segmentation package's service to get the segmented images
  segmented_images_client_ = this->create_client<custom_msg_srv::srv::ImageBatch>("autonomous_robot/get_segmented_images");
  // check the connection to the image_segmentation package's service
  while (!segmented_images_client_->wait_for_service(1s)){
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(this->get_logger(), " segmented_images_service not available, trying again...");
  }

  //subscribe to the goal published with UI
  robot_goal_ui_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
      "autonomous_robot/ui/goal",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GlobalPlanner::robot_goal_ui_callback, this, std::placeholders::_1));

  //subscribe to the robot start pose published with UI
  robot_pose_ui_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
      "autonomous_robot/ui/pose",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GlobalPlanner::robot_pose_ui_callback, this, std::placeholders::_1));

  // a timer to call the global planning
  update_global_planner_timer = this->create_wall_timer(2s,
                                                        std::bind(&GlobalPlanner::make_request_for_segmented_images,
                         this));

  // initialize variables
  original_frames_.reserve(number_of_cameras_);
  nf2_instance = std::make_shared<NF2>(grid_resolution_);
  robot_goal_ = cv::Point();
  robot_pose_ = cv::Point();
}

void GlobalPlanner::make_request_for_segmented_images() {
  auto request = std::make_shared<custom_msg_srv::srv::ImageBatch::Request>();
  RCLCPP_INFO(this->get_logger(), " sending request to image segmentation node...");

  rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedFuture result_future =
      segmented_images_client_->async_send_request(
      request,
      std::bind(&GlobalPlanner::response_received_callback, this, std::placeholders::_1)
      );

}

void GlobalPlanner::response_received_callback(rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedFuture result_future) {
  std::shared_ptr<custom_msg_srv::srv::ImageBatch::Response> response = result_future.get();

  original_frames_.clear();
  for(uint i =0; i<number_of_cameras_;i++) {
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(response->image[i]);
    original_frames_.push_back(cv_image_ptr->image);
  }

  // TODO will change after image stitching is done
  if (is_pose_set && is_goal_set) {// if robots goal and pose are set
    if (robot_pose_camera_index == robot_goal_camera_index) // if both robots goal and pose are in the same camera frame
      nf2_instance->compute_nf2(original_frames_[robot_pose_camera_index], robot_goal_, robot_pose_);
  }
  else
    RCLCPP_INFO(this->get_logger(), " robot goal and/or pose must be set by the user to global planner start working");
}

void GlobalPlanner::robot_goal_ui_callback(geometry_msgs::msg::Point::ConstSharedPtr goal_point) {
  robot_goal_.x = (goal_point->x);
  robot_goal_.y = static_cast<int>(goal_point->y);
  robot_goal_camera_index = static_cast<int>(goal_point->z);
  is_goal_set = true;
}

void GlobalPlanner::robot_pose_ui_callback(geometry_msgs::msg::Point::ConstSharedPtr pose_point) {
  robot_pose_.x = static_cast<int>(pose_point->x);
  robot_pose_.y = static_cast<int>(pose_point->y);
  robot_pose_camera_index = static_cast<int>(pose_point->z);
  is_pose_set = true;
}