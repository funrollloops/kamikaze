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

  struct Pos {
    int16_t first, second;

    bool operator==(const Pos& b) const {
      return first == b.first && second == b.second;
    }
    bool operator!=(const Pos& b) const {
      return !(*this == b);
    }
  } __attribute__((__packed__));
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

class RobotLinuxSPI final : public Robot {
public:
  RobotLinuxSPI(const std::string& device, int baud);
  ~RobotLinuxSPI() final;

  virtual Pos tell();
  virtual void moveTo(Pos pos);
  virtual void fire(std::chrono::milliseconds time);

private:
  enum Tag : uint8_t {
    TELL = 1,
    SEEK = 2,
    FIRE = 3,
  };
  std::string sendMessage(const void* send, uint16_t len);

  template <typename T>
  T sendMessage(const T& t);

  int fd_;
};
