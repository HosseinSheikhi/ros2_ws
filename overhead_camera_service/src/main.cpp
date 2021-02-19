//
// Created by hossein on 12/7/20.
//

#include "rclcpp/rclcpp.hpp"
#include "overhead_camera_service/overhead_camera_node.h"
#include "stdio.h"
int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto overhead_cam_node = std::make_shared<nav2_overhead::OverheadCamNode>(options,3);

  rclcpp::spin(overhead_cam_node);
  rclcpp::shutdown();

  return 0;
}