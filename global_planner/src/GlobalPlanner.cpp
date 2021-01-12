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

  // a timer to call the global planning
  update_planning_timer_ = this->create_wall_timer(2s,
                       std::bind(&GlobalPlanner::make_request_for_segmented_images,
                         this));

  original_frames_.reserve(number_of_cameras_);

  nf2_instance = std::make_shared<NF2>(grid_resolution_);

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

  //show_images("original segmented", original_frames_);
  nf2_instance->compute_nf2(original_frames_[1], goal_, robot_pose_);
  //nf2();
}

void GlobalPlanner::show_images(const std::string window_name, const std::vector<cv::Mat> images){
  for(uint i=0; i<images.size();i++){
    cv::imshow(window_name + std::to_string(i), images[i]);
    cv::waitKey(1);
  }
}