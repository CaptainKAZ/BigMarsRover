//
// Created by neo on 2022/1/6.
//

#include "dynamixel_motor.h"
#include "pthread.h"
#include <algorithm>
#include <iostream>

std::vector<dynamixel::PortHandler *> DynamixelMotor::ports_;
std::thread *DynamixelMotor::pReadThread_;
std::vector<DynamixelMotor *> DynamixelMotor::collection_;

DynamixelMotor::DynamixelMotor(uint8_t id, const std::string &portName) {
  bool portOpened = false;
  dynamixel::PortHandler *portHandler = nullptr;
  id_ = id;
  for (auto ph : ports_) {
    portOpened |= (std::string(ph->getPortName()) == portName);
    if (portOpened) {
      portHandler = ph;
      break;
    }
  }
  if (!portOpened) {
    portHandler = dynamixel::PortHandler::getPortHandler(portName.c_str());
    if (!portHandler->openPort()) {
      throw std::runtime_error("Failed to open port");
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
      throw std::runtime_error("Failed to set baudrate");
    }
    ports_.push_back(portHandler);
  }
  portHandler_ = portHandler;
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  setPosGain(30);
  torqueControl(TORQUE_ENABLE);
  std::cout << "Successfully opened " << id << " at port " + portName
            << std::endl;
  collection_.push_back(this);
  if (pReadThread_ == nullptr) {
    pReadThread_ = new std::thread(readWriteThread);
  }
}
void DynamixelMotor::torqueControl(uint32_t command) {
  uint8_t dxl_error = 0;
  auto dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_, id_, ADDR_TORQUE_ENABLE, command, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    throw std::runtime_error(
        "Failed to enable torque: " +
            std::string(packetHandler_->getTxRxResult(dxl_comm_result)));
  } else if (dxl_error != 0) {
    throw std::runtime_error(
        "Failed to enable torque: " +
            std::string(packetHandler_->getRxPacketError(dxl_error)));
  }
}
[[noreturn]] void DynamixelMotor::readWriteThread() {
  while (true) {
    for (auto m : collection_) {
      m->controlRaw();
      m->readPos();
    }
    // std::thread is implemented by pthread on linux, so we can use pthread API
    // combined with std::thread.
    pthread_testcancel();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
void DynamixelMotor::readPos() {
  uint8_t dxl_error = 0;
  uint32_t presentPos = 0;
  auto dxl_comm_result = packetHandler_->read4ByteTxRx(
      portHandler_, id_, ADDR_PRESENT_POSITION, (uint32_t *) &presentPos,
      &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    throw std::runtime_error(
        "Failed to readPos present position: " +
            std::string(packetHandler_->getTxRxResult(dxl_comm_result)));
  } else if (dxl_error != 0) {
    throw std::runtime_error(
        "Failed to readPos present position: " +
            std::string(packetHandler_->getRxPacketError(dxl_error)));
  }
  q_ = presentPos / pos2Angle_;
}
void DynamixelMotor::controlRaw() {
  if (newCommand_) {
    uint8_t dxl_error = 0;
    auto dxl_comm_result = packetHandler_->write4ByteTxRx(
        portHandler_, id_, ADDR_GOAL_POSITION, (uint32_t) (qDes_ * pos2Angle_),
        &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      throw std::runtime_error(
          "Failed to write goal position: " +
              std::string(packetHandler_->getTxRxResult(dxl_comm_result)));
    } else if (dxl_error != 0) {
      throw std::runtime_error(
          "Failed to write goal position: " +
              std::string(packetHandler_->getRxPacketError(dxl_error)));
    }
    newCommand_ = false;
  }
}
DynamixelMotor::~DynamixelMotor() {
  torqueControl(TORQUE_DISABLE);
  std::vector<DynamixelMotor *>::iterator pos;
  if (collection_.end() !=
      (pos = std::find(collection_.begin(), collection_.end(), this))) {
    collection_.erase(pos);
    if (pos == collection_.begin()) {
      stopThread();
      for (auto p : ports_) {
        p->closePort();
      }
    }
  }
  // WARN: IT WILL CLOSE THE PORT
  //  portHandler_->closePort();
}
void DynamixelMotor::stopThread() {
  if (pReadThread_ != nullptr) {
    pthread_cancel(pReadThread_->native_handle());
    pReadThread_->join();
    delete pReadThread_;
    pReadThread_ = nullptr;
  }
}
void DynamixelMotor::setPosGain(uint16_t gain) {
  uint8_t dxl_error = 0;
  auto dxl_comm_result = packetHandler_->write2ByteTxRx(
      portHandler_, id_, ADDR_POSITION_GAIN, gain, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    throw std::runtime_error(
        "Failed to set position gain: " +
            std::string(packetHandler_->getTxRxResult(dxl_comm_result)));
  } else if (dxl_error != 0) {
    throw std::runtime_error(
        "Failed to set position gain: " +
            std::string(packetHandler_->getRxPacketError(dxl_error)));
  }
}
