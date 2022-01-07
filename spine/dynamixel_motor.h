//
// Created by neo on 2022/1/6.
//

#pragma once
#include "dynamixel_sdk.h"
#include "motor.h"
#include <stdexcept>
#include <string>
#include <thread>

class DynamixelMotor : public Motor {
public:
  DynamixelMotor(uint8_t id, const std::string &portName);
  ~DynamixelMotor();
  std::string portName_{};
  uint8_t id_;

  void setQDes(double q_des) override { qDes_ = q_des; };
  void setQdDes(double qd_des) override {
    throw std::runtime_error("Not implemented");
  };
  void setTauDes(double tau_des) override {
    throw std::runtime_error("Not implemented");
  };
  void stop() override { torqueControl(TORQUE_DISABLE); };
  void control() override { newCommand_ = true; };
  static void stopThread();

  volatile bool newCommand_ = false;

private:
  constexpr const static int BAUDRATE = 57600;
  constexpr const static double PROTOCOL_VERSION = 2.0;
  constexpr const static int TORQUE_ENABLE = 1;
  constexpr const static int TORQUE_DISABLE = 0;
  constexpr const static int ADDR_TORQUE_ENABLE = 512;
  constexpr const static int ADDR_GOAL_POSITION = 564;
  constexpr const static int ADDR_PRESENT_POSITION = 580;
  constexpr const static int ADDR_POSITION_GAIN = 532;
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  static std::vector<dynamixel::PortHandler *> ports_;
  [[noreturn]] static void readWriteThread();
  static std::thread *pReadThread_;
  static std::vector<DynamixelMotor *> collection_;
  // TODO:Actual ratio
  const double pos2Angle_ = (double)251417.0/180.0;
  void torqueControl(uint32_t command);
  void readPos();
  void setPosGain(uint16_t gain);
  void controlRaw();
};
