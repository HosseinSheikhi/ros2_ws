//
// Created by hossein on 1/11/21.
//

#ifndef GLOBAL_PLANNER_INCLUDE_GLOBAL_PLANNER_NF2_H_
#define GLOBAL_PLANNER_INCLUDE_GLOBAL_PLANNER_NF2_H_
#include "iostream"
#include "opencv2/opencv.hpp" //TODO why not opencv4
#include "opencv2/highgui/highgui.hpp"

class NF2 {

 public:
  explicit NF2( uint grid_resolution, uint image_width = 640, uint image_height=480, bool debug= true);
  std::vector<cv::Point> compute_nf2(cv::Mat frame, cv::Point goal, cv::Point robot_pose); // returns the grids sequence from robot pose to goal
  static uint total_instances_counter; // keeps track of total number of NF2 instances

 private:
  const uint grid_resolution_;
  const uint image_width_;
  const uint image_height_;
  bool debug_ = true;
  uint this_instance_index_;

  cv::Mat original_binary_frame_;
  cv::Mat original_rgb_frame_;
  cv::Mat configs_frame_; // will be used for illustration purpose in debug mode to show free and obstacle configs
  cv::Mat boundary_frame_; // will be used for illustration purpose in debug mode to show configs in boundary
  cv::Mat skeleton_frame_; // will be used for illustration purpose in debug mode to show skeleton
  cv::Mat potential_frame_; // will be used for illustration purpose in debug mode to show potential values

  uint erosion_size_{1};
  uint dilate_size_{1};
  cv::Mat erode_element_;
  cv::Mat dilate_element_;

  uint const M = 10000;

  std::vector<cv::Point> free_configurations_; // all the conf in free space
  std::vector<cv::Point> boundary_configurations_; // all the conf in obstacle space which at least have 1 neighbor in free space
  std::vector<cv::Point> obstacle_configurations_; // all the conf excepts ones in free and boundary

  std::vector<cv::Point> skeleton_; // includes configurations which make skeleton
  std::vector<cv::Point> goal_skeleton_; // part of the configurations which connects goal to the skeleton
  std::vector<std::vector<uint>> potential_values_; // includes potential values U for all the grids
  std::vector<std::vector<uint>> distance_; // distance from C-obstacle
  std::vector<std::vector<cv::Point>> origin_; // stores config where wave started
  std::vector<std::vector<cv::Point>> L_; //L_[0] stores configs with distance_ 0, L_[1] stores configs with distance_ 1
  std::vector<cv::Point> accessible_skeleton_; //includes that part of skeleton which is accessible from goal
  cv::Point goal_{12,7}; // TODO, if I use (15,20) gives error, why?
  cv::Point robot_pose_{2,8};

  // minor functions for computing nf2
  bool is_grid_free(uint x_index,  uint y_index) const;
  void draw_grids();
  void pre_process_image();
  std::vector<cv::Point> get_1_neighbors(cv::Point config);
  std::vector<cv::Point> get_2_neighbors(cv::Point config);

  // main functions for computing nf2
  void distinguish_grids();
  void distinguished_grids_debug();

  void compute_skeleton();
  void compute_skeleton_debug();

  void find_boundaries();
  void find_boundaries_debug();

  void find_waves_meet();

  bool connect_goal();
  void connect_goal_debug();

  void skeleton_potential();
  void reminder_potential();
  void potential_debug();

  cv::Point get_min_vertex(std::vector<std::vector<uint>> distance_vertices, std::vector<cv::Point> Q) const;
  std::vector<cv::Point> best_first_search();
  std::vector<cv::Point> find_final_path(std::vector<std::vector<cv::Point>> path);
  void debug_final_path(std::vector<cv::Point> final_path);
  void show_image(std::string window_name, cv::Mat image);

};

#endif //GLOBAL_PLANNER_INCLUDE_GLOBAL_PLANNER_NF2_H_
