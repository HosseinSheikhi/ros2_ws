//
// Created by hossein on 9/9/20.
//

#ifndef GLOBAL_PLANNER__GLOBALPLANNER_H_
#define GLOBAL_PLANNER__GLOBALPLANNER_H_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_msg_srv/srv/image_batch.hpp"
#include <chrono>
class GlobalPlanner : public rclcpp::Node {
 public:
  explicit GlobalPlanner(rclcpp::NodeOptions options);
 private:
  bool debug_ = true;
  const uint number_of_cameras_{1};
  const uint image_width_{640};
  const uint image_height_{480};
  const uint grid_resolution_{16};

  // variables for subscribing images
  std::vector<cv::Point> cameras_pose_;
  std::vector<cv::Mat> original_frames_;
  std::vector<cv::Mat> original_rgb_frames_;

  uint erosion_size_{1};
  uint dilate_size_{1};
  cv::Mat erode_element_;
  cv::Mat dilate_element_;

  uint const M = 10000;

  std::vector<cv::Point> free_configurations_; // all the conf in free space
  std::vector<cv::Point> boundary_configurations_; // all the conf in obstacle space which at least have 1 neighbor in free space
  std::vector<cv::Point> obstacle_configurations_; // all the conf excepts ones in free and boundary

  std::vector<cv::Point> skeleton_; // includes conf. which make skeleton
  std::vector<cv::Point> goal_skeleton_; // part of the configurations which connects goal to the skeleton
  std::vector<std::vector<uint>> potential_values_; // includes potential values U for all the grids
  std::vector<std::vector<uint>> distance_; // distance from C-obstacle
  std::vector<std::vector<cv::Point>> origin_; // stores config where wave started
  std::vector<std::vector<cv::Point>> L_; //L_[0] stores configs with distance_ 0, L_[1] stores configs with distance_ 1
  std::vector<cv::Point> accessible_skeleton_; //includes that part of skeleton which is accessible from goal
  cv::Point goal_{12,7}; // TODO, if I use (15,20) gives error, why?
  cv::Point robot_pose_{2,8};
  rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedPtr segmented_images_client_;
  rclcpp::TimerBase::SharedPtr update_planning_timer_;

  bool is_grid_free(const uint x_index, const uint y_index, const uint cam_index) const;
  void draw_grids();
  void pre_process_images();
  void nf2();
  void distinguish_grids(uint cam_index);
  void compute_skeleton(uint cam_index);
  void find_boundaries(uint cam_index);
  void find_waves_meet();
  void connect_goal();
  void skeleton_potential();
  void reminder_potential();
  std::vector<std::vector<cv::Point>> best_first_search();
  std::vector<cv::Point> get_1_neighbors(cv::Point config);
  std::vector<cv::Point> get_2_neighbors(cv::Point config);
  cv::Point get_min_vertex(std::vector<std::vector<int>> distance_vertices, std::vector<cv::Point> Q);
  void show_images(const std::string window_name, const std::vector<cv::Mat> images);
  void make_request_for_segmented_images();
  void response_received_callback(rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedFuture result_future);
};

#endif //GLOBAL_PLANNER__GLOBALPLANNER_H_
