#include "damiao.h"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unistd.h>

namespace {

constexpr float kDefaultTorqueNm = 0.1F;
constexpr float kTestTorqueLimitNm = 1.0F;
std::atomic<bool> running{true};

void signal_handler(int)
{
  running = false;
}

}  // namespace

int main(int argc, char* argv[])
{
  const float requested_torque_1 =
      (argc > 1) ? std::strtof(argv[1], nullptr) : kDefaultTorqueNm;
  const float requested_torque_2 =
      (argc > 2) ? std::strtof(argv[2], nullptr) : kDefaultTorqueNm;
  const auto limit_torque = [](float torque) {
    return torque < -kTestTorqueLimitNm ? -kTestTorqueLimitNm
           : torque > kTestTorqueLimitNm ? kTestTorqueLimitNm
                                          : torque;
  };
  const float target_torque_1 = limit_torque(requested_torque_1);
  const float target_torque_2 = limit_torque(requested_torque_2);

  if (requested_torque_1 != target_torque_1 ||
      requested_torque_2 != target_torque_2) {
    std::cerr << "Torque commands were limited to ["
              << target_torque_1 << ", " << target_torque_2
              << "] N.m for this test.\n";
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Motor 1: CAN/Slave ID 0x01, Master/feedback ID 0x00.
  // Motor 2: CAN/Slave ID 0x02, Master/feedback ID 0x01.
  damiao::Motor motor_1(damiao::DM4310, 0x01, 0x11);
  damiao::Motor motor_2(damiao::DM4310, 0x02, 0x12);
  auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
  damiao::Motor_Control dm(serial);
  dm.addMotor(&motor_1);
  dm.addMotor(&motor_2);

  try {
    dm.disable(motor_1);
    dm.disable(motor_2);

    if (!dm.switchControlMode(motor_1, damiao::MIT_MODE)) {
      throw std::runtime_error("motor 1 failed to switch to MIT_MODE");
    }
    if (!dm.switchControlMode(motor_2, damiao::MIT_MODE)) {
      throw std::runtime_error("motor 2 failed to switch to MIT_MODE");
    }

    dm.enable(motor_1);
    dm.enable(motor_2);
    std::cout << "MIT torque control started: motor1=" << target_torque_1
              << " N.m, motor2=" << target_torque_2
              << " N.m. Press Ctrl+C to stop.\n";

    std::size_t cycle = 0;
    while (running) {
      dm.control_mit(motor_1, 0.0F, 0.0F, 0.0F, 0.0F,
                     target_torque_1);
      dm.control_mit(motor_2, 0.0F, 0.0F, 0.0F, 0.0F,
                     target_torque_2);

      // control_mit() receives and decodes one feedback frame internally.
      // Print at 10 Hz while keeping the command loop near 1 kHz.
      if (++cycle % 100 == 0) {
        std::cout << std::fixed << std::setprecision(3)
                  << "M1 POS:" << motor_1.Get_Position()
                  << " VEL:" << motor_1.Get_Velocity()
                  << " TAU:" << motor_1.Get_tau()
                  << " ERR:" << motor_1.Get_Err() << '\n';
        std::cout << std::fixed << std::setprecision(3)
                  << " M2 POS:" << motor_2.Get_Position()
                  << " VEL:" << motor_2.Get_Velocity()
                  << " TAU:" << motor_2.Get_tau()
                  << " ERR:" << motor_2.Get_Err() << '\n';
      }

      usleep(1000);
    }

    // Remove the torque command before disabling the drive.
    dm.control_mit(motor_1, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    dm.control_mit(motor_2, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    usleep(10000);
    dm.disable(motor_1);
    dm.disable(motor_2);
    std::cout << "Both motors stopped and disabled.\n";
  } catch (const std::exception& e) {
    std::cerr << "Motor control failed: " << e.what() << '\n';
    try {
      dm.control_mit(motor_1, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
      dm.control_mit(motor_2, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
      dm.disable(motor_1);
      dm.disable(motor_2);
    } catch (...) {
      // Preserve the original exception while making a best-effort stop.
    }
    return 1;
  }

  return 0;
}
