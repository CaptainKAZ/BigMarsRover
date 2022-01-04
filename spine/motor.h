//
// Created by neo on 2021/12/4.
//

#pragma once

#include <chrono>

class Motor {
public:
  double q_{};
  double qd_{};
  double tau_{};
  int turns_{};
  std::chrono::time_point<std::chrono::high_resolution_clock> rxTime_{};

  virtual void setQDes(double q_des) = 0;
  virtual void setQdDes(double qd_des) = 0;
  virtual void setTauDes(double tau_des) = 0;
  virtual void stop() = 0;
  virtual void control() = 0;

protected:
  double qDes_{};
  double qdDes_{};
  double tauDes_{};
};



