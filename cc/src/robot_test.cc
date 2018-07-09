#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "robot.h"
#include "robot_from_flags.h"

DEFINE_bool(interactive, true, "Run repl?");

void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void scripted_loop(Robot* robot) {
#define TELL() std::cout << "pos=" << robot->tell() << std::endl
  while (true) {
    TELL();
    robot->fire(std::chrono::milliseconds(1500));
    robot->moveTo({-500, 1024});
    TELL();
    sleep_ms(500);
    TELL();
    robot->moveTo({500, 1024});
    sleep_ms(2500);
    TELL();
    robot->moveTo({200, -568});
    sleep_ms(5000);
  }
}

void repl(Robot* robot) {
  char cmd;
  while (true) {
    std::cin >> cmd;
    switch (cmd) {
      case 't':
        std::cout << "tell -> " << robot->tell() << std::endl;
        break;
      case 'f': {
        int fire_time;
        std::cin >> fire_time;
        std::cout << "fire(" << fire_time << " ms) -> void" << std::endl;
        robot->fire(std::chrono::milliseconds(fire_time));
        break;
      }
      case 'm': {
        int16_t x, y;
        std::cin >> x >> y;
        std::cout << "move(" << x << ", " << y << ")" << std::endl;
        robot->moveTo(Robot::Pos{x, y});
        break;
      }
      case 's':
        scripted_loop(robot);
      case 'r': {
        const Robot::Pos pos = robot->tell();
        int16_t dx, dy;
        std::cin >> dx >> dy;
        std::cout << "relative(" << pos.first << "+" << dx << ", "
                  << pos.second << "+" << dy << ")" << std::endl;
        robot->moveTo(
            Robot::Pos{int16_t(pos.first + dx), int16_t(pos.second + dy)});
      }
    }
  }
}

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::unique_ptr<Robot> robot = RobotFromFlags();
  std::cout << "commands (hit enter after command):" << std::endl
            << "t: tell current position" << std::endl
            << "f <time:int>: fire for time milliseconds" << std::endl
            << "m <x:int> <y:int>: move to position (x, y)" << std::endl
            << "s: run test script (move up down left right)" << std::endl
            << "r <dx:int> <dy:int>: move to relative position (+dx, +dy)"
            << std::endl;
  if (FLAGS_interactive) {
    repl(robot.get());
  } else {
    scripted_loop(robot.get());
  }
  return 0;
}
