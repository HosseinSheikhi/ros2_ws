//
// Created by hossein on 9/9/20.
//

#ifndef GLOBAL_PLANNER__GLOBALPLANNER_H_
#define GLOBAL_PLANNER__GLOBALPLANNER_H_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GlobalPlanner : public rclcpp::Node {
 public:
  explicit GlobalPlanner(rclcpp::NodeOptions options);
 private:
  const uint number_of_cameras{2};
  const uint image_width{640};
  const uint image_height{480};
  const uint grid_resolution{32};

  // variables for subscribing images
  std::vector<cv::Point> cameras_pose;
  std::vector<cv::Mat> original_frames;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr overhead_cam1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr overhead_cam2_sub_;
  void imageCallback1(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  static void mouseCallback(int event, int x, int y, int flags, void *user_data);
  static void mouseCallback2(int event, int x, int y, int flags, void *user_data);


  // variables for global planning
  void globalPlanner(uint pixel_x, uint pixel_y, uint cam_index);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_publisher_;
  void convertPixelCrdToWorldCrd(uint pixel_x, uint pixel_y, double &world_x, double &world_y);

  uint erosion_size{2};
  cv::Mat erode_element = getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       cv::Point( erosion_size, erosion_size ) );

  uint dilate_size{2};
  cv::Mat dilate_element = getStructuringElement( cv::MORPH_RECT,
                                                   cv::Size( 2*dilate_size + 1, 2*dilate_size+1 ),
                                                   cv::Point( dilate_size, dilate_size ) );

  std::vector<cv::Point> free_configurations; // all the conf in free space
  std::vector<cv::Point> boundary_configurations; // all the conf in obstacle space which have atleast 1 neighbor in free space
  std::vector<cv::Point> obstacle_configurations; // all the conf excepts ones in free and boundary

  std::vector<cv::Point> skeleton; // includes conf. which make skeleton
  std::vector<std::vector<uint>> potential_values; // includes potential values U for all the grids

};

#endif //GLOBAL_PLANNER__GLOBALPLANNER_H_
