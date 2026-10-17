#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <atomic>
#include <csignal>

damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4310,0x02, 0x12);
damiao::Motor M3(damiao::DM4310,0x03, 0x13);

std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

std::atomic<bool> running{true};

void signalHandler(int)
{
    running = false;
}

int main(int argc, char  *argv[])
{
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
  dm = damiao::Motor_Control(serial);

  dm.addMotor(&M1);
  dm.addMotor(&M2);
  dm.addMotor(&M3);
  dm.printMotorIds();

  dm.disable(M1);
  dm.disable(M2);
  dm.disable(M3);
  if (!dm.switchControlMode(M1, damiao::MIT_MODE)) {
    throw std::runtime_error("motor 1 failed to switch to MIT_MODE");
  }
  if (!dm.switchControlMode(M2, damiao::MIT_MODE)) {
    throw std::runtime_error("motor 2 failed to switch to MIT_MODE");
  }
  if (!dm.switchControlMode(M3, damiao::MIT_MODE)) {
    throw std::runtime_error("motor 3 failed to switch to MIT_MODE");
  }
  dm.enable(M1);
  dm.enable(M2);
  dm.enable(M3);
  sleep(1);

  std::size_t cycle = 0;
  while(running)
  {
    // MIT纯力矩控制：kp=0、kd=0、目标位置=0、目标速度=0。
    dm.control_mit(M1, 0.0F, 0.0F, 0.0F, 0.0F, 0.2F);
    dm.control_mit(M2, 0.0F, 0.0F, 0.0F, 0.0F, 0.3F);
    dm.control_mit(M3, 0.0F, 0.0F, 0.0F, 0.0F, 0.4F);

    if (++cycle % 100 == 0) {
      std::cout<<"motor1--- POS:"<<M1.Get_Position()<<" VEL:"<<M1.Get_Velocity()<<" TAU:"
          <<M1.Get_tau()<<" ERR:"<<M1.Get_Err()<<std::endl;
      std::cout<<"motor2--- POS:"<<M2.Get_Position()<<" VEL:"<<M2.Get_Velocity()<<" TAU:"
          <<M2.Get_tau()<<" ERR:"<<M2.Get_Err()<<std::endl;
      std::cout<<"motor3--- POS:"<<M3.Get_Position()<<" VEL:"<<M3.Get_Velocity()<<" TAU:"
          <<M3.Get_tau()<<" ERR:"<<M3.Get_Err()<<std::endl;
    }
    usleep(1000);
  }
  dm.control_mit(M1, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  dm.control_mit(M2, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  dm.control_mit(M3, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  usleep(10000);

  dm.disable(M1);
  dm.disable(M2);
  dm.disable(M3);

  return 0;
}
