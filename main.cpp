#include <iostream>
#include "spine/dynamixel_motor.h"



int main() {
  DynamixelMotor motor(1,"/dev/ttyUSB0");
  motor.setQDes(10);
  while(1){
    std::cout<<motor.q_<<std::endl;
    //motor.control();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
