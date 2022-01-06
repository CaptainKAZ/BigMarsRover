//
// Created by neo on 2022/1/6.
//

#pragma once
#include "socketcan.h"

class BriterEncoder {
public:
  double q_;
  uint8_t id_;
  std::chrono::high_resolution_clock::time_point rxTime_;
  void attach(SocketCan &can, uint8_t id);

private:
  static bool canProbe(BriterEncoder *self, struct can_frame &frame,
                       SocketCan *sc);
  SocketCan *sc_;
  double ecd2Q;
  void setZero();
  void setFeedbackTime(uint16_t us);
  void readVal();
  void setMode(uint8_t mode);
};
