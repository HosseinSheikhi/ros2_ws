//
// Created by hossein on 9/22/20.
//

#ifndef TF_PUBLISHER__TF_PUBLISHER_H_
#define TF_PUBLISHER__TF_PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class TFPublisher : public rclcpp::Node {
  /*
   * This class shall publish the transoframtion tree of overhead cameras
   *  wrt to the world_fram
   */
 public:
  explicit TFPublisher(std::string name = "tf_publisher");
 private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _broadcaster;



};

#endif //TF_PUBLISHER__TF_PUBLISHER_H_
