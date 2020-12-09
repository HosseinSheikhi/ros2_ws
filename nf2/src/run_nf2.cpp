#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;
int erosion_size = 2;
Mat element = getStructuringElement( MORPH_RECT,
                                     Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                     Point( erosion_size, erosion_size ) );

int width = 640;
int height = 480;
int grid_size = 32;
Mat original_image;
vector<Point> CFree;
vector<Point> CBoundary;
vector<Point> CObstacle;
vector<Point> skeleton;


bool is_free(int x, int y){
  int counter =0;
  for (int c=x*width/grid_size;c<(x+1)*width/grid_size;c++)
    for (int r=y*height/grid_size;r<(y+1)*height/grid_size;r++) {
      if (int(original_image.at<uchar>(r, c)) > 100)
        counter++;
    }
  return (counter<10 )? true : false;
}
int M = 100;
vector<vector<int>> gridCSpace;
vector<vector<int>> potential;
vector<vector<int>> distance_;
vector<vector<Point>> origin;
vector<vector<Point>> L;
vector<Point> goal_segment;
Point goal(21,14);
Point q_init(5,31);
vector<Point> accessible_skeleton; //accessible from goal
vector<Point> free_path;

void find_boundaries(){
  bool is_boundary;
  for(auto var : CObstacle) {
    is_boundary = false;
    if (std::count(CFree.begin(), CFree.end(), Point(var.x - 1, var.y))) {
      CBoundary.push_back(var);
      is_boundary = true;
    }
    else if (std::count(CFree.begin(), CFree.end(), Point(var.x + 1, var.y))) {
      CBoundary.push_back(var);
      is_boundary = true;
    }
    else if (std::count(CFree.begin(), CFree.end(), Point(var.x, var.y - 1))) {
      CBoundary.push_back(var);
      is_boundary = true;
    }
    else if (std::count(CFree.begin(), CFree.end(), Point(var.x, var.y + 1))) {
      CBoundary.push_back(var);
      is_boundary = true;
    }

    if (is_boundary) {
      distance_[var.x][var.y] = 0; // distance from C-obstacles
      origin[var.x][var.y] = var; // config where wave started
      L[0].push_back(var); // configs with distance 0 from C-obstacles
    }
  }
}

void wave_meet(){
  vector<Point> neighbors;
  for (auto row = L.begin(); row!=L.end(); row++)
    for (auto var = row->begin(); var!=row->end(); var++){
      // for every 1-neighbor q' of q in CFree
      neighbors.clear();
      if(std::count(CFree.begin(), CFree.end(), Point(var->x-1, var->y)))
        neighbors.push_back(Point(var->x-1, var->y));
      if(std::count(CFree.begin(), CFree.end(), Point(var->x+1, var->y)))
        neighbors.push_back(Point(var->x+1, var->y));
      if(std::count(CFree.begin(), CFree.end(), Point(var->x, var->y-1)))
        neighbors.push_back(Point(var->x, var->y-1));
      if(std::count(CFree.begin(), CFree.end(), Point(var->x, var->y+1)))
        neighbors.push_back(Point(var->x, var->y+1));

      for(auto neighbor : neighbors){
        if(distance_[neighbor.x][neighbor.y]==M){ //q' not visited yet
          distance_[neighbor.x][neighbor.y] = distance_[var->x][var->y]+1;
          origin[neighbor.x][neighbor.y] = origin[var->x][var->y];
          L[distance_[neighbor.x][neighbor.y]].push_back(neighbor); //L i+1
        }
        else{
          Point origin_q = origin[var->x][var->y];
          Point origin_neighbor = origin[neighbor.x][neighbor.y];
          if (sqrt(pow(abs(origin_q.x-origin_neighbor.x),2)+pow(abs(origin_q.y-origin_neighbor.y),2))>2.5)
            if(!std::count(skeleton.begin(), skeleton.end(),Point(var->x, var->y)))
              skeleton.push_back(neighbor);
        }
      }
    }
}

void connect_goal(Point goal){
  int steepest_ascent = -1;
  Point temp;
  vector<Point> neighbours;
  if(!count(CFree.begin(), CFree.end(), goal)){
    cout<<"Goal is not in the CFree space!";
    return;
  }
  if(count(skeleton.begin(), skeleton.end(), goal)){
    cout<<"Goal already in the skeleton";
    return;
  }

  while (true){
    steepest_ascent = -1;
    neighbours.clear();
    neighbours.push_back(Point(goal.x-1,goal.y));
    neighbours.push_back(Point(goal.x+1,goal.y));
    neighbours.push_back(Point(goal.x,goal.y+1));
    neighbours.push_back(Point(goal.x,goal.y-1));
    goal_segment.push_back(goal);
    for(auto neighbour : neighbours){
      if(count(skeleton.begin(),skeleton.end(),neighbour))
        return;
      if(distance_[neighbour.x][neighbour.y]>steepest_ascent){
        steepest_ascent = distance_[neighbour.x][neighbour.y];
        temp = neighbour;
      }
      goal = temp;
    }
  }
}

void skeleton_potential(){
  vector<Point> list;
  vector<Point> neighbours;
  potential[goal.x][goal.y] = 0;
  list.push_back(goal);
  accessible_skeleton.push_back(goal);
  for( int i = 0; i<list.size(); i++){
    auto var = list[i];
    neighbours.clear();
    neighbours.push_back(Point(var.x-1,var.y));
    neighbours.push_back(Point(var.x+1,var.y));
    neighbours.push_back(Point(var.x,var.y+1));
    neighbours.push_back(Point(var.x,var.y-1));
    neighbours.push_back(Point(var.x-1,var.y-1));
    neighbours.push_back(Point(var.x+1,var.y+1));
    neighbours.push_back(Point(var.x-1,var.y+1));
    neighbours.push_back(Point(var.x+1,var.y-1));
    for (auto neighbor : neighbours){
      if(count(skeleton.begin(), skeleton.end(), neighbor) || count(goal_segment.begin(), goal_segment.end(),neighbor) )
        if(potential[neighbor.x][neighbor.y]==M){
          potential[neighbor.x][neighbor.y] = potential[var.x][var.y]+1;
          list.push_back(neighbor);
          accessible_skeleton.push_back(neighbor);
        }
    }
  }
}

void reminder_potential(){
  vector<vector<Point>> list;
  list.resize(grid_size, vector<Point>());
  list[0]=accessible_skeleton;
  vector<Point> neighbours;
  for(int row=0; row<list.size(); row++){
    for (auto var : list[row]) {
      neighbours.clear();
      neighbours.push_back(Point(var.x - 1, var.y));
      neighbours.push_back(Point(var.x + 1, var.y));
      neighbours.push_back(Point(var.x, var.y + 1));
      neighbours.push_back(Point(var.x, var.y - 1));
      for (auto neighbor : neighbours)
        if (count(CFree.begin(), CFree.end(), neighbor)) {
          if (potential[neighbor.x][neighbor.y] == M) {
            potential[neighbor.x][neighbor.y] = potential[var.x][var.y] + 1;
            list[row + 1].push_back(neighbor);
          }
        }
    }
    }
}

Point get_min_vertex(vector<vector<int>> distance_vertices, vector<Point> Q){
  Point temp;
  int dist = INFINITY;
 for(int i=0; i<grid_size; i++){
   for(int j=0; j<grid_size; j++){
     if(distance_vertices[i][j]<dist)
       if(count(Q.begin(), Q.end(), Point(i,j))){
         temp= Point(i,j);
         dist=distance_vertices[i][j];
       }
   }
 }
  return temp;
}
vector<vector<Point>> best_first_search(Point q_init){
  vector<vector<int>> distance_vertices;
  distance_vertices.resize(grid_size, vector<int>(grid_size,INFINITY));
  vector<vector<Point>> prev;
  prev.resize(grid_size, vector<Point>(grid_size));
  vector<Point> spt;
  vector<Point> Q;
  for(int i=0; i<grid_size; i++)
    for(int j=0; j<grid_size; j++)
      if(potential[i][j]!=M)
        Q.push_back(Point(i,j));

  vector<Point> neighbours;

  if(potential[q_init.x][q_init.y]==M){
    cout<<"No free path connects q_init to q_goal at the resolution grid"<<endl ;
    return prev ;
  }
  distance_vertices[q_init.x][q_init.y] = 0;
  while(Q.size()>0){
    Point temp = get_min_vertex(distance_vertices, Q);
    spt.push_back(temp);
    Q.erase(std::remove(Q.begin(), Q.end(), temp), Q.end());
    int s = Q.size();
    neighbours.clear();
    neighbours.push_back(Point(temp.x - 1, temp.y));
    neighbours.push_back(Point(temp.x + 1, temp.y));
    neighbours.push_back(Point(temp.x, temp.y + 1));
    neighbours.push_back(Point(temp.x, temp.y - 1));
    for(auto neighbor : neighbours){
      if (count(CFree.begin(), CFree.end(), neighbor))
        if(potential[neighbor.x][neighbor.y]!=M)
          if(distance_vertices[temp.x][temp.y]+potential[neighbor.x][neighbor.y]<distance_vertices[neighbor.x][neighbor.y]){
            distance_vertices[neighbor.x][neighbor.y] = distance_vertices[temp.x][temp.y]+potential[neighbor.x][neighbor.y];
            prev[neighbor.x][neighbor.y] = temp;
            if(neighbor==goal) {
              cout << "Goal Reached!"<<endl;
              return prev;
            }
          }
        }
  }

  cout<<"Could not find the goal!"<<endl;
  return prev;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  original_image = imread("/home/hossein/dev_ws/src/nf2/src/predict.jpg", IMREAD_COLOR);
  threshold(original_image, original_image, 128, 255, THRESH_BINARY);
  erode(original_image, original_image, element);
  dilate(original_image, original_image, element);
  resize(original_image, original_image, Size(width,height));
  Mat grid_image ;
  original_image.copyTo(grid_image);
  cvtColor(original_image, original_image,COLOR_BGR2GRAY);

  for (int i=0; i<grid_size; i++){
    line(grid_image, Point(width/grid_size*i,0),Point(width/grid_size*i,height), Scalar(255,0,0));
    line(grid_image, Point(0,height/grid_size*i),Point(width, height/grid_size*i), Scalar(255,0,0));
  }

  gridCSpace.resize(grid_size, std::vector<int>(grid_size,1));
  potential.resize(grid_size, std::vector<int>(grid_size,M));
  distance_.resize(grid_size, std::vector<int>(grid_size,1));
  origin.resize(grid_size, std::vector<Point>(grid_size));
  L.resize(grid_size, vector<Point>());

  for(int i=0;i<grid_size;i++)
    for(int j=0;j<grid_size;j++)
      if(is_free(i,j)) {
        gridCSpace[i][j] = 0;
        CFree.push_back(Point(i,j));
      }else{
        CObstacle.push_back(Point(i,j));
      }

//  for(int i=0;i<grid_size;i++)
//    for(int j=0;j<grid_size;j++)
//      if(gridCSpace[i][j])
//        putText(grid_image,"1",Point(width/grid_size*i+5,height/grid_size*j+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (0, 0, 255), 1);
//      else
//        putText(grid_image,"0",Point(width/grid_size*i+5,height/grid_size*j+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (255, 255, 0), 1);

  // Skeleton Algo begins here
  for(auto var: CFree)
    distance_[var.x][var.y] = M;

  //find boundary configs for C-obstacles
  find_boundaries();

//  for(auto var : CBoundary)
//    putText(grid_image,"0",Point(var.x*width/grid_size+5, var.y*height/grid_size+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (255, 255, 0), 1);

  wave_meet();

//  for(int i=0;i<grid_size;i++)
//    for(int j=0;j<grid_size;j++)
//      if(count(CFree.begin(), CFree.end(),Point(i,j)))
//        putText(grid_image,to_string(distance_[i][j]),Point(width/grid_size*i+5,height/grid_size*j+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (0, 0, 255), 1);

//  for(auto var : skeleton)
//    putText(grid_image,"+",Point(width/grid_size*var.x+5,height/grid_size*var.y+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (0, 0, 255), 1);

  // Connect goal to the skeleton

  connect_goal(goal);
//  for(auto var : goal_segment)
//    putText(grid_image,"+",Point(width/grid_size*var.x+5,height/grid_size*var.y+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (0, 255, 0), 1);

  // Compute potential restricted to the skeleton
  skeleton_potential();
//  for(int i=0;i<grid_size;i++)
//    for(int j=0;j<grid_size;j++)
//      if(potential[i][j]!=M)
//        putText(grid_image,to_string(potential[i][j]),Point(width/grid_size*i+5,height/grid_size*j+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (0, 0, 255), 1);

    reminder_potential();
//    for(int i=0;i<grid_size;i++)
//      for(int j=0;j<grid_size;j++)
//        if(potential[i][j]!=M)
//          putText(grid_image,to_string(potential[i][j]),Point(width/grid_size*i+5,height/grid_size*j+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (0, 0, 255), 1);

  auto res = best_first_search(q_init);
  int row = goal.x;
  int col = goal.y;
  while (true){
    auto temp = res[row][col];
    putText(grid_image,"*",Point(width/grid_size*row+5,height/grid_size*col+10),FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar (0, 0, 255), 1);
    if(temp==q_init)
      break;
    else{
      row = temp.x;
      col = temp.y;
    }
  }


  imshow("original image", original_image);
  imshow("Grid image", grid_image);
  waitKey(0);

  return 0;
}
