//
// Created by neo on 2022/1/4.
//

#include "robomodule_motor.h"
#include <thread>

void RoboModuleMotor::reset() {
  can_frame frame{
      .can_id = (uint32_t)(((group_ & 0x7) << 8) | ((id_ & 0xF) << 4) | (0)),
      .can_dlc = 8,
      .data = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}};
  if (sc_ != nullptr && sc_->isOpen()) {
    *sc_ << frame;
  } else {
    throw std::runtime_error("Socket can error or not attached!");
  }
  lastCommandTime_ = std::chrono::high_resolution_clock::now();
}

void RoboModuleMotor::setMode(RoboModuleMotor::Mode mode) {
  can_frame frame{
      .can_id = (uint32_t)(((group_ & 0x7) << 8) | ((id_ & 0xF) << 4) | (1)),
      .can_dlc = 8,
      .data = {(uint8_t)mode, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}};
  if (sc_ != nullptr && sc_->isOpen()) {
    *sc_ << frame;
  } else {
    throw std::runtime_error("Socket can error or not attached!");
  }
  lastCommandTime_ = std::chrono::high_resolution_clock::now();
}
void RoboModuleMotor::controlPos(uint16_t pwmLimit, int32_t pos) {
  if (pwmLimit > 5000) {
    throw std::runtime_error("PWM limit invalid!");
  }
  can_frame frame{
      .can_id = (uint32_t)(((group_ & 0x7) << 8) | ((id_ & 0xF) << 4) | (5)),
      .can_dlc = 8,
      .data = {(uint8_t)((pwmLimit >> 8) & 0xFF), (uint8_t)(pwmLimit & 0xFF),
               0x55, 0x55, (uint8_t)((pos >> 24) & 0xFF),
               (uint8_t)((pos >> 16) & 0xFF), (uint8_t)((pos >> 8) & 0xFF),
               (uint8_t)(pos & 0xFF)}};
  if (sc_ != nullptr && sc_->isOpen()) {
    *sc_ << frame;
  } else {
    throw std::runtime_error("Socket can error or not attached!");
  }
  lastCommandTime_ = std::chrono::high_resolution_clock::now();
}
void RoboModuleMotor::setFeedback(uint8_t ms) {
  can_frame frame{
      .can_id = (uint32_t)(((group_ & 0x7) << 8) | ((id_ & 0xF) << 4) | (0xA)),
      .can_dlc = 8,
      .data = {ms, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}};
  if (sc_ != nullptr && sc_->isOpen()) {
    *sc_ << frame;
  } else {
    throw std::runtime_error("Socket can error or not attached!");
  }
  lastCommandTime_ = std::chrono::high_resolution_clock::now();
}
void RoboModuleMotor::ping() {
  can_frame frame{
      .can_id = (uint32_t)(((group_ & 0x7) << 8) | ((id_ & 0xF) << 4) | (0xF)),
      .can_dlc = 8,
      .data = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}};
  if (sc_ != nullptr && sc_->isOpen()) {
    *sc_ << frame;

  } else {
    throw std::runtime_error("Socket can error or not attached!");
  }
  lastCommandTime_ = std::chrono::high_resolution_clock::now();
}
void RoboModuleMotor::stop() {
  if (state_ != RESET) {
    reset();
    state_ = RESET;
  }
}
void RoboModuleMotor::control() {
  if (state_ == CONTROL_POS) {
    std::this_thread::sleep_until(lastCommandTime_ +
                                  std::chrono::milliseconds(1));
    controlPos(pwmLimit_, (int32_t)(q2int_ * qDes_));
  } else {
    throw std::runtime_error("Not inited! State=" + std::to_string(state_));
  }
}
void RoboModuleMotor::init(RoboModuleMotor *self) {
  // Use co-routine may be better
  if (self->state_ == UNKNOWN) {
    self->reset();
    self->state_ = RESET;
  }
  if (self->state_ == RESET) {
    std::this_thread::sleep_until(self->lastCommandTime_ +
                                  std::chrono::milliseconds(500));
    self->setMode(POSITION_MODE);
    self->state_ = SELECT_MODE;
  }
  if (self->state_ == SELECT_MODE) {
    std::this_thread::sleep_until(self->lastCommandTime_ +
                                  std::chrono::milliseconds(500));
    self->setFeedback(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    self->sc_->bindRxCallback([self](auto &&PH1, auto &&PH2) {
      return RoboModuleMotor::canProbe(self, std::forward<decltype(PH1)>(PH1),
                                       std::forward<decltype(PH2)>(PH2));
    });
    self->ping();
  }
}
void RoboModuleMotor::attach(uint8_t group, uint8_t id, SocketCan *sc) {
  group_ = group;
  id_ = id;
  sc_ = sc;
  // Multi-thread non-blocking init
  pInitThread_ = new std::thread(RoboModuleMotor::init, this);
  // init(this);
}
bool RoboModuleMotor::canProbe(RoboModuleMotor *self, can_frame &frame,
                               SocketCan *sc) {
  if ((self->sc_ == sc) && (self->group_ == (frame.can_id >> 8)) &&
      (self->id_ == ((frame.can_id >> 4) & 0xF)) && (frame.can_dlc == 8)) {
    uint8_t command = (frame.can_id & 0xF);
    switch (command) {
    case 0xF:
      // Pong
      self->state_ = CONTROL_POS;
      break;
    case 0xB:
      // Feedback
      self->tau_ = (double)((int16_t)(frame.data[0] << 8 | frame.data[1])) /
                   self->tau2int_;
      self->qd_ = (double)((int16_t)((frame.data[2] << 8) | frame.data[3])) /
                  self->qd2int_;
      self->q_ =
          (double)((int32_t)((frame.data[4] << 24) | (frame.data[5] << 16) |
                             (frame.data[6] << 8) | frame.data[7])) /
          self->q2int_;
      // printf("%f\n",self->q2int_);
      break;
    case 0xD:
      self->state_ = FAULT;
      self->faultHandler();
      break;
    default:
      break;
    }
    return true;
  }
  return false;
}
void RoboModuleMotor::faultHandler() const {
  throw std::runtime_error("Motor at group " + std::to_string(group_) + " id " +
                           std::to_string(id_) + " stalled");
}
bool RoboModuleMotor::isReady() {
  if (state_ == CONTROL_POS) {
    pInitThread_->join();
    return true;
  }
  return false;
}
