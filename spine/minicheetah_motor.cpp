//
// Created by neo on 2021/12/4.
//

#include "minicheetah_motor.h"
#include <cmath>
#include <stdexcept>
#include <cstdio>

void MiniCheetahMotor::controlRaw(double qDes, double qdDes, double tauDes,
                                  double kP, double kD) {
  struct can_frame frame {};
  if (id_ == 0) {
    throw std::runtime_error("Motor is not attach to can id");
  }
  frame.can_id = id_;
  frame.can_dlc = 8;
  uint16_t currentInt = tau2uint(tauDes);
  uint16_t posInt = q2uint(qDes);
  uint16_t velInt = qd2Uint(qdDes);
  uint16_t kpInt = kp2uint(kP);
  uint16_t kdInt = kd2uint(kD);
  frame.data[0] = posInt >> 8;
  frame.data[1] = posInt & 0xFF;
  frame.data[2] = velInt >> 4;
  frame.data[3] = ((velInt & 0xF) << 4) | ((kpInt) >> 8);
  frame.data[4] = kpInt & 0xFF;
  frame.data[5] = kdInt >> 4;
  frame.data[6] = ((kdInt & 0xF) << 4) | (currentInt >> 8);
  frame.data[7] = currentInt & 0xFF;
  if (sc_ != nullptr) {
    *sc_ << frame;
  } else {
    throw std::runtime_error("Motor is not attach to can device");
  }
}

void MiniCheetahMotor::attach(unsigned char id, SocketCan *sc) {
  if (id == 0 || sc == nullptr) {
    throw std::runtime_error("invalid attach args");
  }
  id_ = id;
  sc_ = sc;
  sc_->bindRxCallback([this](auto &&PH1, auto &&PH2) {
    return MiniCheetahMotor::canProbe(this, std::forward<decltype(PH1)>(PH1),
                                      std::forward<decltype(PH2)>(PH2));
  });
  struct can_frame frame = {
      .can_id = id_,
      .can_dlc = 8,
      .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}};
  *sc_ << frame;
  char stopCommand[128];
  sprintf(stopCommand, "cansend %s %03d#FF.FF.FF.FF.FF.FF.FF.FD",
          sc_->interfaceName_.c_str(), id_);
  stopCommand_ = std::string(stopCommand);
}

bool MiniCheetahMotor::canProbe(MiniCheetahMotor *self, struct can_frame &frame,
                                SocketCan *sc) {
  if (self->sc_ == sc && self->id_ == frame.data[0] && frame.can_dlc == 8 &&
      frame.can_id == 0) {
    self->rxTime_ = std::chrono::high_resolution_clock::now();
    double q =
        self->uint2Q((uint16_t)(frame.data[1] << 8u) | (frame.data[2] & 0xFF));
    if (!self->firstTimeRx_) {
      if (q - self->q_ > M_PI) {
        self->turns_--;
      } else if (q - self->q_ < -M_PI) {
        self->turns_++;
      }
    } else {
      self->firstTimeRx_ = true;
    }
    self->q_ = q;
    self->qd_ = self->uint2Qd((uint16_t)(frame.data[3] << 4u) |
                              ((frame.data[4]) >> 4u));
    self->tau_ =
        self->uint2Tau(((uint16_t)(frame.data[4] & 0xF) << 8u) | frame.data[5]);
    return true;
  } else {
    return false;
  }
}

uint16_t MiniCheetahMotor::double2uint(double val, double min, double max,
                                       int bits) {
  if (min > max) {
    throw std::runtime_error("min>max? you are insane!");
  }
  if (val > max) {
    val = max;
  } else if (val < min) {
    val = min;
  }
  return (int16_t)((val - min) * ((float)(1 << bits) / (max - min)));
}

double MiniCheetahMotor::uint2double(int val, double min, double max,
                                     int bits) {
  if (min > max) {
    throw std::runtime_error("min>max? you are insane!");
  }
  return ((double)val * (max - min) / ((float)(1 << bits) - 1)) + min;
}
void MiniCheetahMotor::stop() {
  if (id_ != 0 && sc_ != nullptr) {
    struct can_frame frame {
      .can_id = id_, .can_dlc = 8, .data = {
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFD
      }
    };
    *sc_ << (frame);
  } else {
    throw std::runtime_error("Motor is not attached");
  }
}
void MiniCheetahMotor::estopAll() {
  for (auto motor : collection_) {
    if (!motor->stopCommand_.empty()) {
      (void)system(motor->stopCommand_.c_str());
    }
  }
}

double AK10_9Motor::current2Tau(double current) { return current; }
double AK10_9Motor::tau2Current(double tau) { return tau; }