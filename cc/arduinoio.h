#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

class ArduinoIO {
public:
  using Pin = char;
  static constexpr Pin kUnconnected1 = 17;
  static constexpr Pin kUnconnected2 = 18;
  static constexpr Pin kUnconnected3 = 12;

  ArduinoIO(const std::string &port, int baud_rate);

  void WriteOutput(Pin pin, bool value);

  struct MoveCmd {
    Pin dir_pin;
    Pin pulse_pin;
    Pin neg_trigger_pin;
    Pin pos_trigger_pin;
    Pin done_pin;
    bool forward;
    uint8_t max_wait = 0;
    int32_t steps;                  // little-endian -- fine on x86.
    int32_t temp_pin_threshold = 0; // little-endian -- fine on x86.
  };

  void Move(const MoveCmd &cmd) {
    SendMessage("MOVE" + std::string((const char *)&cmd, sizeof(cmd)));
  }

  ~ArduinoIO() { serial_port_.close(); }

private:
  void SendMessage(std::string command);

  boost::asio::io_service io_;
  boost::asio::serial_port serial_port_{io_};
  std::mutex mu_;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_message_;
};

struct Motor {
  ArduinoIO &io;
  ArduinoIO::MoveCmd cmd;

  ArduinoIO::Pin ms1;
  ArduinoIO::Pin ms2;
  ArduinoIO::Pin ms3;


  enum : bool { RIGHT = 0, LEFT = 1, UP = 0, DOWN = 1 };

  enum Speed : char {
    FULL = 0,
    HALF = 1,
    QUARTER = 2,
    EIGHTH = 3,
    SIXTEENTH = 7
  };

  void SetSpeed(Speed s) {
    io.WriteOutput(ms1, s & 1);
    io.WriteOutput(ms2, s & 2);
    io.WriteOutput(ms3, s & 4);
  }

  void Move(bool forward, int steps) {
    cmd.forward = forward;
    cmd.steps = steps;
    io.Move(cmd);
  }

  void MoveAndSetSpeed(bool forward, Speed speed, int steps) {
    SetSpeed(speed);
    Move(forward, steps);
  }

  void MoveAutoSpeed(bool forward, int steps) {
    const int kCoarseness = 256;
    for (Speed speed : {SIXTEENTH, EIGHTH, QUARTER, HALF}) {
      if (steps < kCoarseness) {
        MoveAndSetSpeed(forward, speed, steps);
        return;
      }
      steps >>= 1;
    }
    MoveAndSetSpeed(forward, FULL, steps);
  }
};

class ArduinoSerialLineIO {
 public:
  ArduinoSerialLineIO(const std::string &port, int baud_rate);
  void ClearReadBuffer();
  bool SendLine(const char *cmd, std::size_t len);
  std::string ReadLine();

 private:
  boost::asio::io_service io_;
  boost::asio::serial_port serial_port_{io_};
};

struct AsBytes {
  AsBytes(const std::string& s) : s(s) {}
  const std::string& s;
};

std::ostream& operator<<(std::ostream& os, const AsBytes& b);
