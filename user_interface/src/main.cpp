//
// Created by hossein on 12/10/20.
//

#include "user_interface/UserInterface.h"
#include <rclcpp/rclcpp.hpp>
#include <qt5/QtCore/QDebug>

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  qDebug()<<"hi";
  auto UI = std::make_shared<UserInterface>(options);

  rclcpp::spin(UI);
  rclcpp::shutdown();

  return 0;
}
