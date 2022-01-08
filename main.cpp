#include "spine/dynamixel_motor.h"
#include "spine/robomodule_motor.h"
#include "spine/socketcan.h"
#include "spine/minicheetah_motor.h"
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
int main2() {
  DynamixelMotor motor[4] = {
      DynamixelMotor(1, "/dev/ttyUSB0"), DynamixelMotor(2, "/dev/ttyUSB0"),
      DynamixelMotor(3, "/dev/ttyUSB0"), DynamixelMotor(4, "/dev/ttyUSB0")};
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  for (auto &m : motor) {
    m.setQDes(45);
    m.control();
    printf("new command set\n");
    printf("%d\n", m.newCommand_);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  printf("___________________________________new command "
         "set_________________________\n");
  for (auto &m : motor) {
    m.setQDes(45);
    m.control();
  }
  getchar();
  return 0;
}

int main() {
  SocketCan can("can0", 1e6);
  AK10_9Motor motor0, motor1, motor2, motor3;
  motor0.attach(1, &can);
  // std::this_thread::sleep_for(std::chrono::milliseconds(20));
  motor1.attach(2, &can);
  // std::this_thread::sleep_for(std::chrono::milliseconds(20));
  motor2.attach(3, &can);
  // std::this_thread::sleep_for(std::chrono::milliseconds(20));
  motor3.attach(4, &can);
  // std::chrono::high_resolution_clock::time_point
  // startTime=std::chrono::high_resolution_clock::now();
  while (std::chrono::high_resolution_clock::now() - startTime <
         std::chrono::seconds(10)) {
    motor0.setQdDes(-2);
    motor1.setQdDes(2);
    motor2.setQdDes(-2);
    motor3.setQdDes(2);
    motor0.setKpKd(0, 1);
    motor1.setKpKd(0, 1);
    motor2.setKpKd(0, 1);
    motor3.setKpKd(0, 1);
    motor0.control();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    motor1.control();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    motor2.control();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    motor3.control();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return 0;
}