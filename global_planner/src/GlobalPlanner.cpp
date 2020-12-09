//
// Created by hossein on 9/9/20.
//

#include "../include/global_planner/GlobalPlanner.h"
#include "tf2_ros/create_timer_ros.h"
#include <chrono>
#include <iostream>
using namespace std::chrono_literals;

GlobalPlanner::GlobalPlanner(rclcpp::NodeOptions options)
: Node("global_planner_node", options){
  overhead_cam1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/overhead_cam_1/camera/image_raw",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GlobalPlanner::imageCallback1,this,std::placeholders::_1)
      );

  overhead_cam2_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/overhead_cam_2/camera/image_raw",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GlobalPlanner::imageCallback2,this,std::placeholders::_1)
  );
  nav_goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",10);

  original_frames.resize(number_of_cameras);
  cameras_pose.push_back(cv::Point(-1, -3));
  cameras_pose.push_back(cv::Point(5, -3));
  cv::namedWindow("overhead_camera"+std::to_string(0));
  cv::namedWindow("overhead_camera"+std::to_string(1));

}

void GlobalPlanner::mouseCallback(int event, int x, int y, int flags, void *user_data) {
  if (event == cv::EVENT_LBUTTONDOWN){
    GlobalPlanner &pointer = *((GlobalPlanner *)user_data);
    pointer.globalPlanner(x,y,0);
  }
}

void GlobalPlanner::mouseCallback2(int event, int x, int y, int flags, void *user_data) {
  if (event == cv::EVENT_LBUTTONDOWN){
    GlobalPlanner &pointer = *((GlobalPlanner *)user_data);
    pointer.globalPlanner(x,y,1);
  }
}

void GlobalPlanner::globalPlanner(uint pixel_x, uint pixel_y, uint cam_index){
  RCLCPP_INFO(this->get_logger(),"Global planner triggered with mouse callback");

  cv::circle(original_frames[cam_index],cv::Point(pixel_x,pixel_y),
             4,cv::Scalar(0,0,255),
             -1,cv::LINE_AA);
  cv::imshow("overhead_camera"+std::to_string(cam_index), original_frames[cam_index]);
  cv::waitKey(200);

  double world_x, world_y;
  convertPixelCrdToWorldCrd(pixel_x,pixel_y,world_x,world_y);

  geometry_msgs::msg::PoseStamped goal_pose;

  goal_pose.pose.position.x = world_x + cameras_pose[cam_index].x;
  goal_pose.pose.position.y = world_y + cameras_pose[cam_index].y;

  goal_pose.header.stamp = this->now();
  goal_pose.header.frame_id = "map";
  nav_goal_publisher_->publish(goal_pose);
}

void GlobalPlanner::convertPixelCrdToWorldCrd(uint pixel_x, uint pixel_y, double &world_x, double &world_y){
  world_x = (pixel_x - 320.5)*5.5/381.36 ;
  world_y = -(pixel_y - 240.5)*5.5/381.36;
}
void GlobalPlanner::imageCallback1(const sensor_msgs::msg::Image::ConstSharedPtr msg) {

  cv_bridge::CvImagePtr cv_image_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  original_frames[0] = cv_image_ptr->image;
  cv::setMouseCallback("overhead_camera"+std::to_string(0),GlobalPlanner::mouseCallback, this);

  //cv::imwrite("first_overhead_cam.jpg", original_frames[0]);
  cv::imshow("overhead_camera"+std::to_string(0), original_frames[0]);
  cv::waitKey(1);
}

void GlobalPlanner::imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr msg) {

  cv_bridge::CvImagePtr cv_image_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  original_frames[1] = cv_image_ptr->image;
  cv::setMouseCallback("overhead_camera"+std::to_string(1),GlobalPlanner::mouseCallback2, this);

  //cv::imwrite("second_overhead_cam.jpg", original_frames[1]);
  cv::imshow("overhead_camera"+std::to_string(1), original_frames[1]);
  cv::waitKey(1);
}
