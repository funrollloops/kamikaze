#include <stdio.h>

#include <chrono>
#include <iosfwd>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <memory>
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

  static std::unique_ptr<Robot> FromFlags();
};

std::ostream& operator<<(std::ostream& os, const Robot::Pos& pos);

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
  virtual Pos tell() {
    // Simulate moving at 1 step / ms.
    if (pos_ != target_) {
      auto now = std::chrono::system_clock::now();
      int ms = (now - last_set_) / std::chrono::milliseconds(1);
      if (abs(pos_.first - target_.first) > ms)
        pos_.first += target_.first > pos_.first ? ms : -ms;
      else
        pos_.first = target_.first;
      if (abs(pos_.second - target_.second) > ms)
        pos_.second += target_.second > pos_.second ? ms : -ms;
      else
        pos_.second = target_.second;
      last_set_ = now;
    }
    return pos_;
  }
  virtual void moveTo(Pos pos) {
    VLOG(2) << "moveTo(" << pos.first << ", " << pos.second << ")" << std::endl;
    tell();
    target_ = pos;
  }
  void fire(std::chrono::milliseconds time) final {
    std::cout << "fire(" << (time / std::chrono::milliseconds(1)) << " ms)"
              << std::endl;
    std::this_thread::sleep_for(time);
  }

private:
  Pos pos_{0,0};
  Pos target_{0, 0};
  std::chrono::time_point<std::chrono::system_clock> last_set_ =
    std::chrono::system_clock::now();
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
