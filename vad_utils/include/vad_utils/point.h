//
// Created by hossein on 9/8/20.
//

#ifndef VAD_UTILS_POINT_H
#define VAD_UTILS_POINT_H

template <typename T>
class Point {
public:
  Point():x_{0}, y_{0} {}
  Point(T x, T y):x_{x}, y_{y} {}
  void set(T x, T y){
    x_=x;
    y_=y;
  }
private:
  T x_;
  T y_;
};


#endif //VAD_UTILS_POINT_H
