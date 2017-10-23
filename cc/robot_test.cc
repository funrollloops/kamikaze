#include "robot.h"
#include "logging.h"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

std::ostream &operator<<(std::ostream &os, Robot::Pos pos) {
  return os << pos.first << "," << pos.second;
}

int main(int argc, char *argv[]) {
  QCHECK(argc > 1) << ": usage: " << argv[0] << " /dev/spidev0.0 100000\n";

  RobotLinuxSPI robot(argv[1], atoi(argv[2]));
#define TELL() std::cout << "pos=" << robot.tell() << std::endl
  while (true) {
    TELL();
    robot.fire(std::chrono::milliseconds(1500));
    robot.moveTo({-500, 1024});
    TELL();
    sleep_ms(500);
    TELL();
    robot.moveTo({500, 1024});
    sleep_ms(2500);
    TELL();
    robot.moveTo({200, -568});
    sleep_ms(5000);
  }
  return 0;
}
