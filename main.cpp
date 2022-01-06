#include "spine/dynamixel_motor.h"
#include "spine/robomodule_motor.h"
#include "spine/socketcan.h"
#include <iostream>

int main() {
  SocketCan can("can0", 1e6);
  ElectricPutter electricPutter;
  electricPutter.attach(2, 4, &can);
  std::cout << (int)electricPutter.id_ << std::endl;
  std::cout << (int)electricPutter.group_ << std::endl;
  while (!electricPutter.isReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "Waiting for boot..." << std::endl;
  }
  electricPutter.setQDes();
  while (1) {
    electricPutter.control();
    std::cout << electricPutter.q_ << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
