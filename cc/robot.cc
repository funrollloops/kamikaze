#include "robot.h"

RobotSerial::RobotSerial(const std::string &tty, int baud) : io_(tty, baud) {}

RobotSerial::~RobotSerial() {}

Robot::Pos RobotSerial::tell() {
  std::string response;
  while (!io_.SendAndRead("t", 1, &response) ||
         response.size() != sizeof(Pos)) {
    std::cerr << "warning: bad response to tell() cmd: " << AsBytes(response)
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return *reinterpret_cast<const Pos*>(response.data());
}

void RobotSerial::moveTo(Pos pos) {
  struct __attribute__((packed)) {
    char h;
    int16_t first, second;
  } cmd{'m', pos.first, pos.second};
  io_.SendLine(reinterpret_cast<const char*>(&cmd), sizeof(cmd));
}

void RobotSerial::fire(std::chrono::milliseconds time) {
  struct __attribute__((packed)) {
    char h;
    uint16_t ms;
  } cmd{'f', static_cast<uint16_t>(time / std::chrono::milliseconds(1))};
  io_.SendLine(reinterpret_cast<const char*>(&cmd), sizeof(cmd));
}
