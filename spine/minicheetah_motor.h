//
// Created by neo on 2022/1/3.
//

#pragma once

#include "motor.h"
#include "socketcan.h"
#include <algorithm>

class MiniCheetahMotor : public Motor {
public:
  unsigned char id_ = 0;

  MiniCheetahMotor() { throw std::runtime_error("unspecified parameter!"); }
  MiniCheetahMotor(double kPMax, double kDMax, double tauMax, double velMax,
                   double posMax, double kPMin, double kDMin, double tauMin,
                   double velMin, double posMin,
                   std::function<double(int)> current2Tau,
                   std::function<int(double)> tau2Current)
      : kDMax_(kDMax), kPMax_(kPMax), tauMax_(tauMax), velMax_(velMax),
        posMax_(posMax), kDMin_(kDMin), kPMin_(kPMin), tauMin_(tauMin),
        velMin_(velMin), posMin_(posMin), current2Tau_(std::move(current2Tau)),
        tau2Current_(std::move(tau2Current)) {
    collection_.push_back(this);
  };
  ~MiniCheetahMotor() {
    stop();
    auto pos = std::find(collection_.begin(), collection_.end(), this);
    if (pos != collection_.end()) {
      collection_.erase(pos);
    }
  }
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
  static void estopAll();

private:
  SocketCan *sc_ = nullptr;
  double kP_ = 0, kD_ = 0;
  const double kPMax_{}, kDMax_{}, tauMax_{}, velMax_{}, posMax_{}, kPMin_{},
      kDMin_{}, tauMin_{}, velMin_{}, posMin_{};
  std::string stopCommand_{};

  static std::vector<MiniCheetahMotor *> collection_;

  const std::function<double(double)> current2Tau_;
  const std::function<double(double)> tau2Current_;

  static bool canProbe(MiniCheetahMotor *self, struct can_frame &frame,
                       SocketCan *sc);
  bool firstTimeRx_ = false;
  static uint16_t double2uint(double val, double min, double max, int bits);
  static double uint2double(int val, double min, double max, int bits);

  double uint2Qd(uint16_t vel) const {
    return uint2double(vel, velMin_, velMax_, 12);
  };
  uint16_t qd2Uint(double qd) const {
    return double2uint(qd, velMin_, velMax_, 12);
  };
  double uint2Q(uint16_t pos) const {
    return uint2double(pos, posMin_, posMax_, 16);
  };
  uint16_t q2uint(double q) const {
    return double2uint(q, posMin_, posMax_, 16);
  };
  uint16_t kp2uint(double kp) const {
    return double2uint(kp, kPMin_, kPMax_, 12);
  };
  uint16_t kd2uint(double kd) const {
    return double2uint(kd, kDMin_, kDMax_, 12);
  };
  uint16_t tau2uint(double tau) const {
    return double2uint(tau2Current_(tau), tauMin_, tauMax_, 12);
  }
  double uint2Tau(uint16_t current) const {
    return current2Tau_(uint2double(current, tauMin_, tauMax_, 12));
  }
};

class AK10_9Motor : public MiniCheetahMotor {
public:
  AK10_9Motor()
      : MiniCheetahMotor(500, 5, 54, 23.24, 12.5, 0, 0, -54, -23.24, -12.5,
                         current2Tau, tau2Current){};

private:
  // Motor curve
  static double current2Tau(double current);
  static double tau2Current(double tau);
};

class AK10_8Motor : public MiniCheetahMotor {
  // TBD
};