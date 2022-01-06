//
// Created by neo on 2022/1/6.
//

#include "briter_encoder.h"
void BriterEncoder::attach(SocketCan &can, uint8_t id) {
  id_ = id;
  sc_ = &can;
  setMode(0xAA);
  setFeedbackTime(2000);
  can.bindRxCallback([this](auto &&PH1, auto &&PH2) {
    return BriterEncoder::canProbe(this, std::forward<decltype(PH1)>(PH1),
                                   std::forward<decltype(PH2)>(PH2));
  });
}
bool BriterEncoder::canProbe(BriterEncoder *self, can_frame &frame,
                             SocketCan *sc) {
  if (frame.can_id == self->id_ && frame.data[1] == self->id_) {
    if (frame.data[2] == 0x01 && frame.data[0] == 0x07) {
      int32_t encoderValue = frame.data[3] | frame.data[4] << 8 |
                             frame.data[5] << 16 | frame.data[6] << 24;
      self->q_ = encoderValue * self->ecd2Q;
      self->rxTime_ = std::chrono::high_resolution_clock::now();
    }
    return true;
  }
  return false;
}
void BriterEncoder::setZero() {
  can_frame frame{.can_id = id_, .can_dlc = 4, .data = {0x04, id_, 0x06, 0x00}};
  *sc_ << frame;
}
void BriterEncoder::setFeedbackTime(uint16_t us) {
  can_frame frame{.can_id = id_,
                  .can_dlc = 5,
                  .data = {0x05, id_, 0x05, (uint8_t)(us & 0xFF),
                           (uint8_t)((us >> 8) & 0xFF)}};
  *sc_ << frame;
}
void BriterEncoder::readVal() {
  can_frame frame{.can_id = id_, .can_dlc = 4, .data = {0x04, id_, 0x01, 0x00}};
  *sc_ << frame;
}
void BriterEncoder::setMode(uint8_t mode) {
  can_frame frame{.can_id = id_, .can_dlc = 4, .data = {0x04, id_, 0x04, mode}};
  *sc_ << frame;
}
