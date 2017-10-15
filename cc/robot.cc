#include "robot.h"

RobotSerial::RobotSerial(const std::string &tty, int baud) : io_(tty, baud) {}

RobotSerial::~RobotSerial() {}

std::pair<int16_t, int16_t> RobotSerial::tell() {
  std::string response;
  do {
    if (!response.empty()) {
      std::cerr << "warning: bad response to tell() cmd: " << AsBytes(response);
    }
    io_.ClearReadBuffer();
    io_.SendLine("t", 1);
    response = io_.ReadLine();
  } while (response.size() != sizeof(Pos));
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
