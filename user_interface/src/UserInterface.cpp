//
// Created by hossein on 12/10/20.
//

#include "user_interface/UserInterface.h"
UserInterface::UserInterface(rclcpp::NodeOptions options)
: Node("user_interface_node", options){
  image_1_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/overhead_cam_1/camera/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&UserInterface::original_image_1_callback, this, std::placeholders::_1));

  image_1_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/overhead_cam_2/camera/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&UserInterface::original_image_2_callback, this, std::placeholders::_1));

  original_frames_.resize(number_of_cameras);
  cv::namedWindow("overhead_camera"+std::to_string(0));
  cv::namedWindow("overhead_camera"+std::to_string(1));
}

void UserInterface::user_interface(uint pixel_x, uint pixel_y, uint cam_index) {

}


void UserInterface::original_image_1_callback(const sensor_msgs::msg::Image::ConstSharedPtr image) {
  cv_bridge::CvImagePtr cv_image_ptr=cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
  original_frames_[0] = cv_image_ptr->image;
  cv::setMouseCallback("overhead_camera"+std::to_string(0),UserInterface::mouse_callback_1, this);

  cv::imshow("overhead_camera"+std::to_string(0), original_frames_[0]);
  cv::waitKey(1);
}
void UserInterface::original_image_2_callback(const sensor_msgs::msg::Image::ConstSharedPtr image) {
  cv_bridge::CvImagePtr cv_image_ptr=cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
  original_frames_[1] = cv_image_ptr->image;
  cv::setMouseCallback("overhead_camera"+std::to_string(1),UserInterface::mouse_callback_2, this);

  cv::imshow("overhead_camera"+std::to_string(1), original_frames_[1]);
  cv::waitKey(1);
}
void UserInterface::mouse_callback_1(int event, int x, int y, int flags, void *user_data) {
  if (event == cv::EVENT_LBUTTONDOWN){
    UserInterface &pointer = *((UserInterface *)user_data);
    pointer.user_interface(x,y,0);
  }
}
void UserInterface::mouse_callback_2(int event, int x, int y, int flags, void *user_data) {
  if (event == cv::EVENT_LBUTTONDOWN){
    UserInterface &pointer = *((UserInterface *)user_data);
    pointer.user_interface(x,y,1);
  }
}

void UserInterface::convert_pixel_crd_to_world_crd(uint pixel_x, uint pixel_y, double &world_x, double &world_y){
  world_x = (pixel_x - 320.5)*5.5/381.36 ;
  world_y = -(pixel_y - 240.5)*5.5/381.36;
}