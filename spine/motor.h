//
// Created by neo on 2021/12/4.
//

#pragma once

#include "socketcan.h"
#include "chrono"

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

class MiniCheetahMotor : public Motor {
 public:
  unsigned char id_ = 0;

  MiniCheetahMotor() = default;

  void setQDes(double qDes) override { this->qDes_ = qDes; }

  void setQdDes(double qdDes) override { this->qdDes_ = qdDes; }

  void setTauDes(double tauDes) override { this->tauDes_ = tauDes; }

  void stop() override;

  void control() override { controlRaw(qDes_, qdDes_, tauDes_, kP_, kD_); }

  void setRawDes(double qDes, double qdDes, double tauDes, double kP,
                 double kD) {
    qDes_ = qDes;
    qdDes_ = qdDes;
    tauDes_ = tauDes;
    kP_ = kP;
    kD_ = kD;
  }

  void controlRaw(double qDes, double qdDes, double tauDes, double kP,
                  double kD);

  void setKpKd(double kP, double kD) {
    kP_ = kP;
    kD_ = kD;
  };

  void attach(unsigned char id, SocketCan *sc);

 protected:
  static bool canProbe(MiniCheetahMotor *self, struct can_frame &frame,
                       SocketCan *sc);

  bool firstTimeRx_ = false;

  static uint16_t double2uint(double val, double min, double max, int bits);

  static double uint2double(int val, double min, double max, int bits);

  virtual double current2Tau(uint16_t current) = 0;

  virtual uint16_t tau2Current(double tau) = 0;

  virtual double vel2Qd(uint16_t vel) = 0;

  virtual uint16_t qd2Vel(double qd) = 0;

  virtual double pos2Q(uint16_t pos) = 0;

  virtual uint16_t q2Pos(double q) = 0;

  virtual uint16_t kp2uint(double kp) = 0;

  virtual uint16_t kd2uint(double kd) = 0;

  SocketCan *sc_ = nullptr;

  double kP_ = 0, kD_ = 0;
};

class AK10_9Motor : public MiniCheetahMotor {
 private:
  const double kPMax_ = 500, kDMax_ = 5, tauMax_ = 54, velMax_ = 23.24,
      posMax_ = 12.5, kPMin_ = 0, kDMin_ = 0, tauMin_ = -54,
      velMin_ = -23.24, posMin_ = -12.5;

  double current2Tau(uint16_t current) override;

  uint16_t tau2Current(double tau) override;

  double vel2Qd(uint16_t vel) override;

  uint16_t qd2Vel(double qd) override;

  double pos2Q(uint16_t pos) override;

  uint16_t q2Pos(double q) override;

  uint16_t kp2uint(double kp) override;

  uint16_t kd2uint(double kd) override;
};

class RoboModuleMotor : Motor {
 public:
  uint8_t group_;
  uint8_t id_;
  uint16_t pwmLimit_;
  void reset();
  enum Mode {
    OPENLOOP_MODE = 0x01,
    CURRENT_MODE,
    SPEED_MODE,
    POSITION_MODE,
    SPEED_POSITION_MODE,
    CURRENT_SPEED_MODE,
    CURRENT_SPEED_POSITION_MODE,
  }mode_;
  void setMode(Mode mode);
  void controlPos(int16_t pwmLimit,int32_t pos);
  void setFeedback(uint8_t ms);
  void ping();
  void setQDes(double qDes) override { this->qDes_ = qDes; }
  void setQdDes(double qdDes) override { throw std::runtime_error("qd control for robomodule not implemented"); }
  void setTauDes(double tauDes) override { throw std::runtime_error("tau control for robomodule not implemented"); }
  void stop() override;
  void control() override;
  void attach(uint8_t group,uint8_t id, Mode mode,SocketCan *sc);
 protected:
  bool online_=true;
  SocketCan *sc_;
  static bool canProbe(RoboModuleMotor *self, struct can_frame &frame,
                       SocketCan *sc);
};