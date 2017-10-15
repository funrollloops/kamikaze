#include <stdio.h>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "arduinoio.h"

class Robot {
public:
  virtual ~Robot() {}

  using Pos = std::pair<int16_t, int16_t>;
  virtual Pos tell() = 0;
  virtual void moveTo(Pos pos) = 0;
  virtual void fire(std::chrono::milliseconds time) = 0;
};

class RobotSerial final : public Robot {
public:
  RobotSerial(const std::string& tty, int baud);
  ~RobotSerial() final;

  virtual Pos tell();
  virtual void moveTo(Pos pos);
  virtual void fire(std::chrono::milliseconds time);

private:
  ArduinoSerialLineIO io_;
};

class NoOpRobot final : public Robot {
public:
  virtual Pos tell() { return pos_; }
  virtual void moveTo(Pos pos) { pos_ = pos; }
  void fire(std::chrono::milliseconds time) final {
    std::this_thread::sleep_for(time);
  }

private:
  Pos pos_;
};
