//
// Created by hossein on 1/11/21.
//

#include "global_planner/NF2.h"

uint NF2::total_instances_counter = 0;

NF2::NF2( const uint grid_resolution, const uint image_width, const uint image_height , const bool debug)
: grid_resolution_{grid_resolution}, image_width_{image_width}, image_height_{image_height}, debug_{debug}{ // TODO why will be initialized based on definition sequence
  total_instances_counter +=1;
  this_instance_index_ = total_instances_counter;
  erode_element_ = getStructuringElement(cv::MORPH_RECT,
                                         cv::Size(2*erosion_size_ + 1, 2*erosion_size_+1 ),
                                         cv::Point(erosion_size_, erosion_size_ ) );


  dilate_element_ = getStructuringElement(cv::MORPH_RECT,
                                          cv::Size(2*dilate_size_ + 1, 2*dilate_size_+1 ),
                                          cv::Point(dilate_size_, dilate_size_ ) );
}

void NF2::compute_nf2(cv::Mat frame, cv::Point goal, cv::Point robot_pose) {
  original_binary_frame_ = frame;

  goal_.x = static_cast<int>(goal.x/(image_width_/grid_resolution_)); // navigation goal TODO: is it best time to convert to indexes?
  goal_.y = static_cast<int>(goal.y/(image_height_/grid_resolution_)); // navigation goal
  robot_pose_.x = static_cast<int>(robot_pose.x/(image_width_/grid_resolution_)); // navigation starting point
  robot_pose_.y = static_cast<int>(robot_pose.y/(image_height_/grid_resolution_)); // navigation starting point

  /*
   * main function for numerical potential field
   * NOTE: do not forget to clear all the containers at the beginning of each function
   */

  // do pre-processing on original binary image
  pre_process_image();

  //convert original binary frame to original rgb frame for illustration purposes
  original_binary_frame_.copyTo(original_rgb_frame_);
  cv::cvtColor(original_rgb_frame_, original_rgb_frame_, cv::COLOR_GRAY2BGR);

  // again for illustration purpose
  draw_grids();

  // assign free and obstacle vectors
  distinguish_grids();

  // compute skeleton (first algorithm - Figure2.)
  compute_skeleton();

  // connect goal to the skeleton
  if (!connect_goal()) {
    std::cout << "[WARNING] Could not compute global path!" << std::endl;
    return;
  }
  connect_goal_debug();

  // Compute potential restricted to the skeleton
  skeleton_potential();
  // computer potential of reminded configurations
  reminder_potential();

  auto result_path = best_first_search();
  int row = goal_.x;
  int col = goal_.y;
  if(!result_path.empty()) {
    while (true) {
      auto temp = result_path[row][col];
      putText(original_rgb_frame_,
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
    show_image("Global Path", original_rgb_frame_);
  }
  else
    std::cout<<"[WARNING] Global path could not be find!"<<std::endl;
}

cv::Point NF2::get_min_vertex(std::vector<std::vector<int>> distance_vertices, std::vector<cv::Point> Q) const{
  /*
   * returns the vertex by minimum distance
   */
  cv::Point temp;
  int dist = INFINITY;
  for(uint i=0; i<grid_resolution_; i++){
    for(uint j=0; j<grid_resolution_; j++){
      if(distance_vertices[i][j]<dist)
        if(count(Q.begin(), Q.end(), cv::Point(i,j))){
          temp= cv::Point(i,j);
          dist=distance_vertices[i][j];
        }
    }
  }
  return temp;
}

std::vector<std::vector<cv::Point>> NF2::best_first_search(){

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
  for(uint i=0; i<grid_resolution_; i++)
    for(uint j=0; j<grid_resolution_; j++)
      if(potential_values_[i][j]!=M)
        Q.push_back(cv::Point(i,j));

  std::vector<cv::Point> neighbours;

  if(potential_values_[robot_pose_.x][robot_pose_.y]==M){
    std::cout<<"[WARNING]in best_first_search. No free path connects q_init to q_goal at the resolution grid"<<std::endl;
    result_path.clear();
    return result_path ;
  }

  while(!Q.empty()){
    cv::Point temp = get_min_vertex(distance_vertices, Q); //Q must be a priority list. We use get_min_vertex() to simulate it
    spt.push_back(temp);
    Q.erase(std::remove(Q.begin(), Q.end(), temp), Q.end());
    neighbours.clear();
    neighbours=get_2_neighbors(temp);
    for(const auto& neighbor : neighbours) {
      if (potential_values_[neighbor.x][neighbor.y] != M)
        if (distance_vertices[temp.x][temp.y] + potential_values_[neighbor.x][neighbor.y]
            < distance_vertices[neighbor.x][neighbor.y]) {
          distance_vertices[neighbor.x][neighbor.y] =
              distance_vertices[temp.x][temp.y] + potential_values_[neighbor.x][neighbor.y];
          result_path[neighbor.x][neighbor.y] = temp;
          if (neighbor == goal_) {
            std::cout<<"[INFO] Goal Reached."<<std::endl;
            return result_path;
          }
        }
    }
  }

  std::cout<<"[INFO] Could not find the Goal."<<std::endl;
  result_path.clear();
  return result_path;
}


void NF2::reminder_potential() {
  /*
   * Compute the potential U for all configurations in C_free - Skeleton that are accessible  from goal configuration
   * using  by wave front expansion. Look at Figure4.
   */
  std::vector<std::vector<cv::Point>> list;
  list.resize(grid_resolution_, std::vector<cv::Point>());
  list[0]=accessible_skeleton_;
  std::vector<cv::Point> neighbours;
  for(uint row=0; row<list.size(); row++){
    for (const auto& var : list[row]) {
      neighbours.clear();
      neighbours = get_2_neighbors(var);
      for (const auto& neighbor : neighbours)
        if (count(free_configurations_.begin(), free_configurations_.end(), neighbor)) {
          if (potential_values_[neighbor.x][neighbor.y] == M) {
            /*
             * I have add the following if-else by myself. The intuition is when row=0, means we are computing
             * potential for skeletons' neighbours. In this case, instead of just adding 1 extra value to potential
             * we will add 2, otherwise if we just add 1, the potential value of a neighbour of a skeleton configuration
             * will be equal to the (if there is) another neighbor which is also in the skeleton and its value is
             * already computed in skeleton_potential.
             */
            if (row==0)
              potential_values_[neighbor.x][neighbor.y] = potential_values_[var.x][var.y] + 2;
            else
              potential_values_[neighbor.x][neighbor.y] = potential_values_[var.x][var.y] + 1;
            list[row + 1].push_back(neighbor);
          }
        }
    }
  }
  potential_debug();
}

void NF2::skeleton_potential(){
  /*
   * Use wave front expansion to compute U restricted to skeleton
   * look at Figure3. in README
   */
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
    for (const auto& neighbor : neighbours){
      if(count(skeleton_.begin(), skeleton_.end(), neighbor) || count(goal_skeleton_.begin(), goal_skeleton_.end(),neighbor) )
        if(potential_values_[neighbor.x][neighbor.y]==M){
          potential_values_[neighbor.x][neighbor.y] = potential_values_[var.x][var.y]+1;
          list.push_back(neighbor);
          accessible_skeleton_.push_back(neighbor);
        }
    }
  }
}

bool NF2::connect_goal() {
  /*
   * Connect Goal configuration to the skeleton by a path which follows steepest descent of distances in Free configurations
   */
  int steepest_ascent = -1;
  goal_skeleton_.clear();
  cv::Point temp = goal_;
  std::vector<cv::Point> neighbours;
  if(!count(free_configurations_.begin(), free_configurations_.end(), goal_)){
    std::cout<<"[Warning] In connect_goal function, goal is not in the CFree space!"<<std::endl;
    return false;
  }
  if(count(skeleton_.begin(), skeleton_.end(), goal_)){
    std::cout<<"[INFO] In connect_goal function, goal already in the skeleton!"<<std::endl;
    return true;
  }

  while (true){
    steepest_ascent = -1;
    neighbours.clear();
    goal_skeleton_.push_back(temp);
    neighbours = get_2_neighbors(temp);
    if(neighbours.size()==0)
      return false; // TODO not sure must return false
    for(auto neighbour : neighbours){

      if(std::count(skeleton_.begin(), skeleton_.end(), neighbour)) // reached to the skeleton
        return true;
      if(int(distance_[neighbour.x][neighbour.y])>steepest_ascent){
        steepest_ascent = distance_[neighbour.x][neighbour.y];
        temp = neighbour;
      }
    }
  }
}


void NF2::compute_skeleton() {
  /*
   * Computes the skeleton based on Figure2. in README
   */

  /*
   * Initialization
   */
  boundary_configurations_.clear();
  distance_.clear();
  origin_.clear();
  L_.clear();
  skeleton_.clear();
  distance_.resize(grid_resolution_, std::vector<uint>(grid_resolution_,M)); // initial distance_ values to M
  origin_.resize(grid_resolution_, std::vector<cv::Point>(grid_resolution_));
  L_.resize(grid_resolution_, std::vector<cv::Point>());

  /*
   * Find boundary configs for C-obstacle
   */
  find_boundaries();

  /*
   * Find where waves from boundary nodes meet
   */
  find_waves_meet();

  compute_skeleton_debug();
}

void NF2::find_waves_meet() {
  /*
   * Look at the third part of Figure2. in README
   */
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

void NF2::find_boundaries() {
  /*
   * Look at the second part of Figure2. in Readme
   * for every config Q in obstacle_configurations_:
   *    if Q has a 1-neighbor Q_prime in free_configurations_ then Q is a boundary config
   */
  for(const auto& Q : obstacle_configurations_) {
    auto neighbors = get_1_neighbors(Q);
    if (!neighbors.empty()) { // has a 1-neighbor Q_prime in free_configurations_ then Q is a boundary config
      for (auto conf : neighbors)
        boundary_configurations_.push_back(conf);
      distance_[Q.x][Q.y] = 0; // distance from obstacle_configurations_
      origin_[Q.x][Q.y] = Q; // config where wave started
      L_[0].push_back(Q); // configs with distance 0 from obstacle_configurations_
    }
  }
  find_boundaries_debug();
}



void NF2::distinguish_grids() {
  /*
   * this function assigns each grid to one of the {free, obstacle} sets
   */
  free_configurations_.clear();
  obstacle_configurations_.clear();

  for(uint i=0;i<grid_resolution_;i++)
    for(uint j=0;j<grid_resolution_;j++)
      if(is_grid_free(i,j))
        free_configurations_.push_back(cv::Point(i,j)); // TODO: use emplace_back instead of push_back
      else{
        obstacle_configurations_.push_back(cv::Point(i,j));
      }

  distinguished_grids_debug();
}

void NF2::distinguished_grids_debug() {
  /*
   * For illustration purpose, to depict free vs obstacle grids
   */
  if(debug_){
    original_rgb_frame_.copyTo(configs_frame_);
    for(auto& conf : free_configurations_)
      putText(configs_frame_,
              "1",
              cv::Point(image_width_/grid_resolution_*conf.x+5,image_height_/grid_resolution_*conf.y+10),
              cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, cv::Scalar (0, 0, 255),
              1);

    for(auto& conf : obstacle_configurations_)
      putText(configs_frame_,
              "0",
              cv::Point(image_width_/grid_resolution_*conf.x+5,image_height_/grid_resolution_*conf.y+10),
              cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, cv::Scalar (0, 0, 255),
              1);

    show_image("free VS obstacle (distinguish_grids)",configs_frame_);
  }
}

void NF2::find_boundaries_debug() {
  /*
   * For illustration purpose, to depict boundary configs
   */
  if(debug_) {
    original_rgb_frame_.copyTo(boundary_frame_);
    for (const auto& var : boundary_configurations_)
      putText(boundary_frame_,
              "0",
              cv::Point(var.x * image_width_ / grid_resolution_ + 5, var.y * image_height_ / grid_resolution_ + 10),
              cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
              0.3,
              cv::Scalar(255, 255, 0),
              1);
    show_image("boundaries (find_boundary)",boundary_frame_);
  }
}

void NF2::compute_skeleton_debug() {
  /*
   * For illustration purpose, to depict the computed skeleton
   */
  if(debug_){
    original_rgb_frame_.copyTo(skeleton_frame_);
    for(const auto& var : skeleton_)
      putText(skeleton_frame_,"+",
              cv::Point(image_width_/grid_resolution_*var.x+5,image_height_/grid_resolution_*var.y+10),
              cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, cv::Scalar (0, 0, 255), 1);
    show_image("skeleton (compute_skeleton)",skeleton_frame_);
  }
}

void NF2::connect_goal_debug() {
  /*
   * For illustration purpose, assuming skeleton is already drawn in skeleton_frame, this function just
   * draws the path which connects goal to the skeleton
   */
  if(debug_) {
    for (auto var : goal_skeleton_)
      putText(skeleton_frame_,
              "+",
              cv::Point(image_width_ / grid_resolution_ * var.x + 5, image_height_ / grid_resolution_ * var.y + 10),
              cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
              0.3,
              cv::Scalar(0, 255, 0),
              1);
    show_image("full skeleton (connect_goal)",skeleton_frame_);
  }
}

void NF2::potential_debug(){
  if(debug_) {
    original_rgb_frame_.copyTo(potential_frame_);
    for (int i = 0; i < grid_resolution_; i++)
      for (int j = 0; j < grid_resolution_; j++)
        if (potential_values_[i][j] != M)
          putText(potential_frame_,
                  std::to_string(potential_values_[i][j]),
                  cv::Point(image_width_ / grid_resolution_ * i + 5, image_height_ / grid_resolution_ * j + 10),
                  cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                  0.3,
                  cv::Scalar(0, 0, 255),
                  1);
    show_image("potential values", potential_frame_);
  }
}

void NF2::pre_process_image(){
  /*
  * This function has to do followings:
  * 1- convert cv_images to binary_cv_images
  * 2- apply erode and dilate to remove noises
  * 3- resize the images
  */

    cv::threshold(original_binary_frame_,original_binary_frame_,128,255,cv::THRESH_BINARY);
    cv::erode(original_binary_frame_, original_binary_frame_, erode_element_);
    cv::dilate(original_binary_frame_, original_binary_frame_, dilate_element_);
    cv::resize(original_binary_frame_, original_binary_frame_, cv::Size(image_width_, image_height_));

}

bool NF2::is_grid_free(const uint x_index, const uint y_index) const {
  /*
   * by counting number of white pixels in grid (x_index, y_index) checks whether grid is free or obstacle
   */
  uint white_pixels_counter{0};
  for (uint c = x_index*image_width_/grid_resolution_; c <(x_index+1)*image_width_/grid_resolution_; c++)
    for (uint r = y_index*image_height_/grid_resolution_; r <(y_index+1)*image_height_/grid_resolution_; r++)
      if(int(original_binary_frame_.at<float>(r,c))>128) {
        white_pixels_counter++;
      }

  return (white_pixels_counter < 10);
}

void NF2::draw_grids() {
  /*
   * For illustration purpose. Draw grids on rgb frame
  */
  if (debug_) {
    for (int i = 0; i < grid_resolution_; i++) {
      line(original_rgb_frame_,
           cv::Point(image_width_ / grid_resolution_ * i, 0),
           cv::Point(image_width_ / grid_resolution_ * i, image_height_),
           cv::Scalar(255, 0, 0));
      line(original_rgb_frame_,
           cv::Point(0, image_height_ / grid_resolution_ * i),
           cv::Point(image_width_, image_height_ / grid_resolution_ * i),
           cv::Scalar(255, 0, 0));
    }
    cv::circle(original_rgb_frame_, cv::Point(goal_.x*image_width_ / grid_resolution_,goal_.y*image_height_ / grid_resolution_),6, cv::Scalar(255,255,0), -1);
    cv::circle(original_rgb_frame_, cv::Point(robot_pose_.x*image_width_ / grid_resolution_,robot_pose_.y*image_height_ / grid_resolution_),6, cv::Scalar(255,255,0), -1);
  }
}

void NF2::show_image(std::string window_name, cv::Mat image) {
  if(debug_){
    cv::imshow(window_name + " " +std::to_string(this_instance_index_), image);
    cv::waitKey(1);
  }
}

std::vector<cv::Point> NF2::get_1_neighbors(cv::Point config) {
  /*
   * returns the free 1-neighbors (up, down, right, left) of config
   */
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

std::vector<cv::Point> NF2::get_2_neighbors(cv::Point config) {
  /*
  * returns the free 2-neighbors (up, upper-left, upper-right, ...) of config
  */
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


