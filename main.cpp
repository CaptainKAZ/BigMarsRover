#include "spine/dynamixel_motor.h"
#include "spine/robomodule_motor.h"
#include "spine/socketcan.h"
#include <iostream>

int main_electricPutter() {
  SocketCan can("can0", 1e6);
  ElectricPutter electricPutter[4];
  for (int i = 0; i < 4; i++) {
    electricPutter[i].attach(2, i + 1, &can);
    std::cout << "Booting up putter" << i + 1 << std::endl;
    while (!electricPutter[i].isReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  std::cout << "All putters booted up" << std::endl;
  //(void)getchar();
  for (auto &i : electricPutter) {
    i.setQDes(-5);
    i.control();
  }

  return 0;
}
int main() {
  DynamixelMotor motor[4] = {
      DynamixelMotor(1, "/dev/ttyUSB0"), DynamixelMotor(2, "/dev/ttyUSB0"),
      DynamixelMotor(3, "/dev/ttyUSB0"), DynamixelMotor(4, "/dev/ttyUSB0")};
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  for (auto m : motor) {
    m.setQDes(0);
    m.control();
    printf("new command set\n");
    printf("%d\n", m.newCommand_);
  }
  getchar();
  for (auto m : motor) {
    m.setQDes(45);
    m.control();
  }
  getchar();
  return 0;
}