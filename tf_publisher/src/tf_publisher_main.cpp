//
// Created by hossein on 9/22/20.
//

#include "tf_publisher/tf_publisher.h"

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  std::shared_ptr<TFPublisher> tf_publisher_node = std::make_shared<TFPublisher>();

  rclcpp::spin(tf_publisher_node);
  rclcpp::shutdown();
  return 0;
}