//
// Created by hossein on 9/9/20.
//

#include "global_planner/GlobalPlanner.h"

int main(int argc, char **argv){
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  auto global_planner = std::make_shared<GlobalPlanner>(options);
  rclcpp::spin(global_planner);
  rclcpp::shutdown();


  return 0;
}