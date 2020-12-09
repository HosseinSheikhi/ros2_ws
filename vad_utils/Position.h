//
// Created by hossein on 9/9/20.
//

#ifndef VAD_UTILS__POSITION_H_
#define VAD_UTILS__POSITION_H_

class Position {
 public:
  Position();
  Position(float x, float y, float theta);

  float x_;
  float y_;
  float theta_;
};

#endif //VAD_UTILS__POSITION_H_
