//
// Created by neo on 2022/1/4.
//

#pragma once
#include "motor.h"
#include "socketcan.h"
#include "iostream"

class RoboModuleMotor : public Motor {
public:
  RoboModuleMotor() = default;
  RoboModuleMotor(uint16_t pwmLimit, double q2int, double tau2int,
                  double qd2int)
      : pwmLimit_(pwmLimit), tau2int_(tau2int), qd2int_(qd2int), q2int_(q2int),
        state_(UNKNOWN) {}
  uint8_t group_{};
  uint8_t id_{};

  void setQDes(double qDes) override { this->qDes_ = qDes; }
  void setQdDes(double qdDes) override {
    throw std::runtime_error("qd control for robomodule not implemented");
  }
  void setTauDes(double tauDes) override {
    throw std::runtime_error("tau control for robomodule not implemented");
  }
  void stop() override;
  void control() override;
  void attach(uint8_t group, uint8_t id, SocketCan *sc);
  void faultHandler() const;
  bool isReady();
  // TODO: RESET
  static void goZero();

private:
  void reset();
  enum Mode {
    OPENLOOP_MODE = 0x01,
    CURRENT_MODE,
    SPEED_MODE,
    POSITION_MODE,
    SPEED_POSITION_MODE,
    CURRENT_SPEED_MODE,
    CURRENT_SPEED_POSITION_MODE,
  };
  void setMode(Mode mode);
  void controlPos(uint16_t pwmLimit, int32_t pos);
  void setFeedback(uint8_t ms);
  void ping();

  const uint16_t pwmLimit_ = 5000;
  const double q2int_ = 1;
  const double tau2int_ = 1;
  const double qd2int_ = 1;
  std::chrono::high_resolution_clock::time_point lastCommandTime_{};
  enum State { UNKNOWN, RESET, FAULT, SELECT_MODE, CONTROL_POS } state_;
  SocketCan *sc_ = nullptr;
  static void init(RoboModuleMotor *self);
  static bool canProbe(RoboModuleMotor *self, struct can_frame &frame,
                       SocketCan *sc);
};

class ElectricPutter : public RoboModuleMotor {
public:
  ElectricPutter() : RoboModuleMotor(5000, 640.0 / 3.0, 1, 1) {}
};