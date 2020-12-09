//
// Created by hossein on 9/22/20.
//

#include "../include/tf_publisher/tf_publisher.h"

TFPublisher::TFPublisher(std::string name) : Node(name) {

  _broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this, rclcpp::SystemDefaultsQoS());

  geometry_msgs::msg::TransformStamped msg;
  msg.transform.translation.x = 0.0;
  msg.transform.translation.y = 0.0;
  msg.transform.translation.z = 4.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0,1.57,3.1415);
  msg.transform.rotation.x = quaternion.x();
  msg.transform.rotation.y = quaternion.y();
  msg.transform.rotation.z = quaternion.z();
  msg.transform.rotation.w = quaternion.w();


  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.child_frame_id = "overhead_camera";

  _broadcaster->sendTransform(msg);

}