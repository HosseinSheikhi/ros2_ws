//
// Created by hossein on 9/9/20.
//

#include "../include/global_planner/GlobalPlanner.h"
#include "tf2_ros/create_timer_ros.h"
#include <chrono>
#include <iostream>
using namespace std::chrono_literals;

GlobalPlanner::GlobalPlanner(rclcpp::NodeOptions options)
: Node("global_planner_node", options){

  // this client sends its request to image_segmentation package's service to get the segmented images
  segmented_images_client_ = this->create_client<custom_msg_srv::srv::ImageBatch>("autonomous_robot/get_segmented_images");

  // check the connection to the image_segmentation package's service
  while (!segmented_images_client_->wait_for_service(1s)){
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(this->get_logger(), " segmented_images_service not available, trying again...");
  }

  // a timer to call the global planning
  update_planning_timer_ = this->create_wall_timer(5s,
                       std::bind(&GlobalPlanner::make_request_for_segmented_images,
                         this));

  erode_element_ = getStructuringElement(cv::MORPH_RECT,
                                         cv::Size(2*erosion_size_ + 1, 2*erosion_size_+1 ),
                                         cv::Point(erosion_size_, erosion_size_ ) );


  dilate_element_ = getStructuringElement(cv::MORPH_RECT,
                                          cv::Size(2*dilate_size_ + 1, 2*dilate_size_+1 ),
                                          cv::Point(dilate_size_, dilate_size_ ) );

  original_frames_.reserve(number_of_cameras_);

}

void GlobalPlanner::make_request_for_segmented_images() {
  auto request = std::make_shared<custom_msg_srv::srv::ImageBatch::Request>();
  RCLCPP_INFO(this->get_logger(), " sending request to image segmentation node...");

  rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedFuture result_future =
      segmented_images_client_->async_send_request(
      request,
      std::bind(&GlobalPlanner::response_received_callback, this, std::placeholders::_1)
      );

}

void GlobalPlanner::response_received_callback(rclcpp::Client<custom_msg_srv::srv::ImageBatch>::SharedFuture result_future) {
  std::shared_ptr<custom_msg_srv::srv::ImageBatch::Response> response = result_future.get();

  original_frames_.clear();
  for(uint i =0; i<number_of_cameras_;i++) {
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(response->image[i]);
    original_frames_.push_back(cv_image_ptr->image);
  }

  //show_images("original segmented", original_frames_);
  nf2();
}

void GlobalPlanner::nf2() {
  /*
   * main function for numerical potential field
   * do not forget to clear all the containers at the beginning
   */
  original_rgb_frames_.clear();
  free_configurations_.clear();
  obstacle_configurations_.clear();


  // do pre-processing on original images
  pre_process_images();

  //convert original_frames to original_rgb_frames for illustration purposes
  original_rgb_frames_.resize(original_frames_.size());
  for(uint i=0; i<original_frames_.size(); i++) {
    original_frames_[i].copyTo(original_rgb_frames_[i]);
    cv::cvtColor(original_rgb_frames_[i], original_rgb_frames_[i], cv::COLOR_GRAY2BGR);
  }

  if (debug_) {
    draw_grids();
  }

  uint temp_cam_index = 0;
  distinguish_grids(temp_cam_index);

//  if(debug_){
//    for(auto& conf : free_configurations_)
//      putText(original_rgb_frames_[temp_cam_index],
//              "1",
//              cv::Point(image_width_/grid_resolution_*conf.x+5,image_height_/grid_resolution_*conf.y+10),
//              cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, cv::Scalar (0, 0, 255),
//              1);
//
//    for(auto& conf : obstacle_configurations_)
//      putText(original_rgb_frames_[temp_cam_index],
//              "0",
//              cv::Point(image_width_/grid_resolution_*conf.x+5,image_height_/grid_resolution_*conf.y+10),
//              cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, cv::Scalar (0, 0, 255),
//              1);
//    show_images("free VS obstacle",original_rgb_frames_);
//  }

  compute_skeleton(temp_cam_index);

  connect_goal();
//  if(debug_) {
//    for (auto var : goal_skeleton_)
//      putText(original_rgb_frames_[temp_cam_index],
//              "+",
//              cv::Point(image_width_ / grid_resolution_ * var.x + 5, image_height_ / grid_resolution_ * var.y + 10),
//              cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
//              0.3,
//              cv::Scalar(0, 255, 0),
//              1);
//    show_images("full skeleton",original_rgb_frames_);
//  }

  // Compute potential restricted to the skeleton
  skeleton_potential();
  // computer potential of reminded configurations
  reminder_potential();
//  if(debug_) {
//    for (int i = 0; i < grid_resolution_; i++)
//      for (int j = 0; j < grid_resolution_; j++)
//        if (potential_values_[i][j] != M)
//          putText(original_rgb_frames_[temp_cam_index],
//                  std::to_string(potential_values_[i][j]),
//                  cv::Point(image_width_ / grid_resolution_ * i + 5, image_height_ / grid_resolution_ * j + 10),
//                  cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
//                  0.3,
//                  cv::Scalar(0, 0, 255),
//                  1);
//    show_images("potential values", original_rgb_frames_);
//  }
  auto result_path = best_first_search();
  int row = goal_.x;
  int col = goal_.y;
  if(result_path.size()>0)
    while (true) {
      auto temp = result_path[row][col];
      putText(original_rgb_frames_[temp_cam_index],
              "*",
              cv::Point(image_width_ / grid_resolution_ * row + 5, image_height_ / grid_resolution_ * col + 10),
              cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
              0.3,
              cv::Scalar(0, 255, 255),
              1);
      if (temp == robot_pose_)
        break;
      else {
        row = temp.x;
        col = temp.y;
      }
    }
  show_images("Global Path", original_rgb_frames_);
}

cv::Point GlobalPlanner::get_min_vertex(std::vector<std::vector<int>> distance_vertices, std::vector<cv::Point> Q){
  /*
   * returns the vertex by minimum distance
   */
  cv::Point temp;
  int dist = INFINITY;
  for(int i=0; i<grid_resolution_; i++){
    for(int j=0; j<grid_resolution_; j++){
      if(distance_vertices[i][j]<dist)
        if(count(Q.begin(), Q.end(), cv::Point(i,j))){
          temp= cv::Point(i,j);
          dist=distance_vertices[i][j];
        }
    }
  }
  return temp;
}

std::vector<std::vector<cv::Point>> GlobalPlanner::best_first_search(){

  /*
   *  1) Create a set spt (shortest path tree ) that keeps track of vertices included in shortest path tree, i.e.,
   *      whose minimum distance from source is calculated and finalized. Initially, this set is empty.
      2) Assign a distance value to all vertices in the input graph. Initialize all distance values as INFINITE.
          Assign distance value as 0 for the source vertex so that it is picked first.
      3) While spt doesn’t include all vertices
        ….a) Pick a vertex u which is not there in spt and has minimum distance value.
        ….b) Include u to sptSet.
        ….c) Update distance value of all adjacent vertices of u.
            To update the distance values, iterate through all adjacent vertices.
            For every adjacent vertex v, if sum of distance value of u (from source) and weight of edge u-v,
            is less than the distance value of v, then update the distance value of v.
   */
  std::vector<cv::Point> spt;

  std::vector<std::vector<int>> distance_vertices; //Assign a distance value to all vertices in the input graph
  distance_vertices.resize(grid_resolution_, std::vector<int>(grid_resolution_,INFINITY));
  distance_vertices[robot_pose_.x][robot_pose_.y] = 0;
  std::vector<std::vector<cv::Point>> result_path;
  result_path.resize(grid_resolution_, std::vector<cv::Point>(grid_resolution_));
  std::vector<cv::Point> Q; // includes all the vertices we have to search
  for(int i=0; i<grid_resolution_; i++)
    for(int j=0; j<grid_resolution_; j++)
      if(potential_values_[i][j]!=M)
        Q.push_back(cv::Point(i,j));

  std::vector<cv::Point> neighbours;

  if(potential_values_[robot_pose_.x][robot_pose_.y]==M){
    RCLCPP_INFO(this->get_logger(),"No free path connects q_init to q_goal at the resolution grid");
    result_path.clear();
    return result_path ;
  }

  while(Q.size()>0){
    cv::Point temp = get_min_vertex(distance_vertices, Q); //Q must be a priority list. We use get_min_vertex() to simulate it
    spt.push_back(temp);
    Q.erase(std::remove(Q.begin(), Q.end(), temp), Q.end());
    neighbours.clear();
    neighbours=get_2_neighbors(temp);
    for(auto neighbor : neighbours) {
      if (potential_values_[neighbor.x][neighbor.y] != M)
        if (distance_vertices[temp.x][temp.y] + potential_values_[neighbor.x][neighbor.y]
            < distance_vertices[neighbor.x][neighbor.y]) {
          distance_vertices[neighbor.x][neighbor.y] =
              distance_vertices[temp.x][temp.y] + potential_values_[neighbor.x][neighbor.y];
          result_path[neighbor.x][neighbor.y] = temp;
          if (neighbor == goal_) {
            RCLCPP_INFO(this->get_logger(), "Goal Reached");
            return result_path;
          }
        }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Could not find the Goal");
  result_path.clear();
  return result_path;
}

void GlobalPlanner::reminder_potential() {
  std::vector<std::vector<cv::Point>> list;
  list.resize(grid_resolution_, std::vector<cv::Point>());
  list[0]=accessible_skeleton_;
  std::vector<cv::Point> neighbours;
  for(int row=0; row<list.size(); row++){
    for (auto var : list[row]) {
      neighbours.clear();
      neighbours = get_2_neighbors(var);
      for (auto neighbor : neighbours)
        if (count(free_configurations_.begin(), free_configurations_.end(), neighbor)) {
          if (potential_values_[neighbor.x][neighbor.y] == M) {
            potential_values_[neighbor.x][neighbor.y] = potential_values_[var.x][var.y] + 1;
            list[row + 1].push_back(neighbor);
          }
        }
    }
  }
}

void GlobalPlanner::compute_skeleton(uint cam_index) {
  boundary_configurations_.clear();
  distance_.clear();
  origin_.clear();
  L_.clear();
  skeleton_.clear();
  distance_.resize(grid_resolution_, std::vector<uint>(grid_resolution_,M)); // initial distance_ values to M
  origin_.resize(grid_resolution_, std::vector<cv::Point>(grid_resolution_));
  L_.resize(grid_resolution_, std::vector<cv::Point>());

  find_boundaries(cam_index);

//  if(debug_) {
//    for (auto var : boundary_configurations_)
//      putText(original_rgb_frames_[cam_index],
//              "0",
//              cv::Point(var.x * image_width_ / grid_resolution_ + 5, var.y * image_height_ / grid_resolution_ + 10),
//              cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
//              0.3,
//              cv::Scalar(255, 255, 0),
//              1);
//    show_images("boundary",original_rgb_frames_);
//
//  }
  find_waves_meet();
  if(debug_){
    for(auto var : skeleton_)
      putText(original_rgb_frames_[cam_index],"+",
              cv::Point(image_width_/grid_resolution_*var.x+5,image_height_/grid_resolution_*var.y+10),
              cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, cv::Scalar (0, 0, 255), 1);
    //show_images("boundary",original_rgb_frames_);
  }

}

void GlobalPlanner::skeleton_potential(){
  potential_values_.clear();
  accessible_skeleton_.clear();
  potential_values_.resize(grid_resolution_, std::vector<uint>(grid_resolution_,M));
  std::vector<cv::Point> list;
  std::vector<cv::Point> neighbours;
  potential_values_[goal_.x][goal_.y] = 0; // set the goal potential to the zero
  list.push_back(goal_); // and add the goal to the list
  accessible_skeleton_.push_back(goal_);
  for( int i = 0; i<list.size(); i++){
    auto var = list[i];
    neighbours.clear();
    neighbours = get_2_neighbors(var);
    for (auto neighbor : neighbours){
      if(count(skeleton_.begin(), skeleton_.end(), neighbor) || count(goal_skeleton_.begin(), goal_skeleton_.end(),neighbor) )
        if(potential_values_[neighbor.x][neighbor.y]==M){
          potential_values_[neighbor.x][neighbor.y] = potential_values_[var.x][var.y]+1;
          list.push_back(neighbor);
          accessible_skeleton_.push_back(neighbor);
        }
    }
  }
}

void GlobalPlanner::connect_goal() {
  int steepest_ascent = -1;
  goal_skeleton_.clear();
  cv::Point temp = goal_;
  std::vector<cv::Point> neighbours;
  if(!count(free_configurations_.begin(), free_configurations_.end(), goal_)){
    RCLCPP_INFO(this->get_logger(),"Goal is not in the CFree space!");
    return;
  }
  if(count(skeleton_.begin(), skeleton_.end(), goal_)){
    RCLCPP_INFO(this->get_logger(), "Goal already in the skeleton");
    return;
  }

  while (true){
    steepest_ascent = -1;
    neighbours.clear();
    goal_skeleton_.push_back(temp);
    neighbours = get_2_neighbors(temp);
    if(neighbours.size()==0)
      return;
    for(auto neighbour : neighbours){

      if(std::count(skeleton_.begin(), skeleton_.end(), neighbour)) // reached to the skeleton
        return;
      if(int(distance_[neighbour.x][neighbour.y])>steepest_ascent){
        steepest_ascent = distance_[neighbour.x][neighbour.y];
        temp = neighbour;
      }
    }
  }
}

void GlobalPlanner::find_waves_meet() {
  std::vector<cv::Point> neighbors;
  for (auto row = L_.begin(); row!=L_.end(); row++)
    for (auto var = row->begin(); var!=row->end(); var++){
      neighbors.clear();
      neighbors = get_1_neighbors(cv::Point(var->x, var->y));
      // for every 1-neighbor q' of q in CFree
      for(auto neighbor : neighbors){
        if(distance_[neighbor.x][neighbor.y]==M){ //q' not visited yet
          distance_[neighbor.x][neighbor.y] = distance_[var->x][var->y]+1;
          origin_[neighbor.x][neighbor.y] = origin_[var->x][var->y];
          L_[distance_[neighbor.x][neighbor.y]].push_back(neighbor); //L i+1
        }
        else{
          cv::Point origin_q = origin_[var->x][var->y];
          cv::Point origin_neighbor = origin_[neighbor.x][neighbor.y];
          if (sqrt(pow(abs(origin_q.x-origin_neighbor.x),2)+pow(abs(origin_q.y-origin_neighbor.y),2))>=2)
            if(!std::count(skeleton_.begin(), skeleton_.end(),cv::Point(var->x, var->y)))
              skeleton_.push_back(neighbor);
        }
      }
    }
}

void GlobalPlanner::find_boundaries(uint cam_index) {
  /*
   * for every config Q in obstacle_configurations_:
   *    if Q has a 1-neighbor Q_prime in free_configurations_ then Q is a boundary config
   */
  bool is_boundary;
  for(auto var : obstacle_configurations_) {
    is_boundary = false;
    if (std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(var.x - 1, var.y))) {
      boundary_configurations_.push_back(var);
      is_boundary = true;
    }
    else if (std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(var.x + 1, var.y))) {
      boundary_configurations_.push_back(var);
      is_boundary = true;
    }
    else if (std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(var.x, var.y - 1))) {
      boundary_configurations_.push_back(var);
      is_boundary = true;
    }
    else if (std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(var.x, var.y + 1))) {
      boundary_configurations_.push_back(var);
      is_boundary = true;
    }

    if (is_boundary) {
      distance_[var.x][var.y] = 0; // distance from obstacle_configurations_
      origin_[var.x][var.y] = var; // config where wave started
      L_[0].push_back(var); // configs with distance 0 from obstacle_configurations_
    }
  }
}

std::vector<cv::Point> GlobalPlanner::get_1_neighbors(cv::Point config){
  std::vector<cv::Point> neighbors;
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x-1, config.y)))
    neighbors.push_back(cv::Point(config.x-1, config.y));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x+1, config.y)))
    neighbors.push_back(cv::Point(config.x+1, config.y));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x, config.y-1)))
    neighbors.push_back(cv::Point(config.x, config.y-1));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x, config.y+1)))
    neighbors.push_back(cv::Point(config.x, config.y+1));

  return neighbors;
}
std::vector<cv::Point> GlobalPlanner::get_2_neighbors(cv::Point config){
  std::vector<cv::Point> neighbors;
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x-1, config.y)))
    neighbors.push_back(cv::Point(config.x-1, config.y));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x+1, config.y)))
    neighbors.push_back(cv::Point(config.x+1, config.y));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x, config.y-1)))
    neighbors.push_back(cv::Point(config.x, config.y-1));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x, config.y+1)))
    neighbors.push_back(cv::Point(config.x, config.y+1));

  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x-1, config.y-1)))
    neighbors.push_back(cv::Point(config.x-1, config.y-1));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x+1, config.y+1)))
    neighbors.push_back(cv::Point(config.x+1, config.y+1));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x+1, config.y-1)))
    neighbors.push_back(cv::Point(config.x+1, config.y-1));
  if(std::count(free_configurations_.begin(), free_configurations_.end(), cv::Point(config.x-1, config.y+1)))
    neighbors.push_back(cv::Point(config.x-1, config.y+1));
  return neighbors;
}


void GlobalPlanner::distinguish_grids(uint cam_index) {
  /*
   * this function assigns each grid to one of the {free, obstacle} sets
   */
  for(int i=0;i<grid_resolution_;i++)
    for(int j=0;j<grid_resolution_;j++)
      if(is_grid_free(i,j, cam_index))
        free_configurations_.push_back(cv::Point(i,j));
      else{
        obstacle_configurations_.push_back(cv::Point(i,j));
      }

}

void GlobalPlanner::pre_process_images() {
  /*
   * This function has to do followings:
   * 1- convert cv_images to binary_cv_imaegs
   * 2- apply erode and dilate to remove noises
   * 3- resize the images
   */
  for(auto &image : original_frames_){
    cv::threshold(image,image,128,255,CV_THRESH_BINARY);
    cv::erode(image, image, erode_element_);
    cv::dilate(image, image, dilate_element_);
    cv::resize(image, image, cv::Size(image_width_, image_height_));
  }
}

void GlobalPlanner::show_images(const std::string window_name, const std::vector<cv::Mat> images){
  for(uint i=0; i<images.size();i++){
    cv::imshow(window_name + std::to_string(i), images[i]);
    cv::waitKey(1);
  }
}

void GlobalPlanner::draw_grids() {
  for(auto &grid_image : original_rgb_frames_)
    for (int i=0; i<grid_resolution_; i++){
      line(grid_image, cv::Point(image_width_/grid_resolution_*i,0),cv::Point(image_width_/grid_resolution_*i,image_height_), cv::Scalar(255,0,0));
      line(grid_image, cv::Point(0,image_height_/grid_resolution_*i),cv::Point(image_width_, image_height_/grid_resolution_*i), cv::Scalar(255,0,0));
    }
}

bool GlobalPlanner::is_grid_free(const uint x_index, const uint y_index, const uint cam_index) const {
  /*
   * by counting number of white pixels in grid (x_index, y_index) checks whether grid is free or obstacle
   */
    uint white_pixels_counter{0};
    for (uint c = x_index*image_width_/grid_resolution_; c <(x_index+1)*image_width_/grid_resolution_; c++)
      for (uint r = y_index*image_height_/grid_resolution_; r <(y_index+1)*image_height_/grid_resolution_; r++)
        if(int(original_frames_[cam_index].at<float>(r,c))>128) {
          white_pixels_counter++;
        }

    return (white_pixels_counter < 10);
}

