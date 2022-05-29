#include "arduinoio.h"

#include <boost/bind.hpp>
#include <mutex>

#include <glog/logging.h>

namespace {

std::string Encode(const std::string &command) {
  const char address = 0;
  const char addr_size = 1;
  const char timeout = 0;
  std::ostringstream buf;
  buf << addr_size << char(command.size()) << timeout << address << command;
  // Append fletcher16 checksum.
  char chk1 = 0;
  char chk2 = 0;
  for (char c : buf.str()) {
    chk1 += c;
    chk2 += chk1;
  }
  buf << chk2 << chk1;
  VLOG(4) << AsBytes(buf.str());
  return buf.str();
}

class port_wrapper {
public:
 port_wrapper(boost::asio::serial_port &port) : port(port) {}

 // Reads a character or times out. val is not modified unless the function
 // returns true.  returns false if the read times out.
 bool read_char(char &val, std::chrono::milliseconds timeout) {
   std::unique_lock<std::mutex> lock(mutex_);
   char c;
   std::future<std::size_t> bytes_read = boost::asio::async_read(
       port, boost::asio::buffer(&c, 1), boost::asio::use_future);
   auto deadline = std::chrono::steady_clock::now() + timeout;
   if (bytes_read.wait_until(deadline) == std::future_status::ready &&
       bytes_read.get() == 1) {
     val = c;
     return true;
   }
   return false;
  }

  bool write_msg(std::string msg, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    std::future<std::size_t> bytes_written = boost::asio::async_write(
        port, boost::asio::buffer(msg), boost::asio::use_future);
    auto deadline = std::chrono::steady_clock::now() + timeout;
    return bytes_written.wait_until(deadline) == std::future_status::ready &&
           bytes_written.get() == msg.size();
  }

private:
  std::mutex mutex_;
  boost::asio::serial_port &port;
};

enum class State : char {
  READY = 'R',
  ERROR = 'E',
  UNKNOWN = 0,
};

State GetLatestState(boost::asio::serial_port &port) {
  char c = 0;
  port_wrapper reader(port);
  while (reader.read_char(c, std::chrono::seconds(c ? 0 : 1)) || !c) {
    if (c != char(State::READY) && c != char(State::ERROR))
      c = 0;
  }
  return State(c);
}

} // namespace

std::ostream& operator<<(std::ostream& os, const AsBytes& b) {
  char buf[4];
  for (char c : b.s) {
    CHECK(snprintf(buf, sizeof(buf), "%02x ", c) == 3);
    os << buf;
  }
  return os;
}

ArduinoIO::ArduinoIO(const std::string &port, int baud_rate) {
  serial_port_.open(port);
  serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  mu_.lock();
  std::thread ready_thread([&] {
      LOG(INFO) << "Waiting for --tty=" << port << " to be ready.";
      WaitReadyForever();
      mu_.unlock();
      LOG(INFO) << "First WaitReadyForever successful.";
      // Make sure we communicate at least once per second.
      while (true) {
        auto to_wait =
            last_message_ + std::chrono::seconds(1) -
            std::chrono::high_resolution_clock::now();
        if (to_wait > std::chrono::seconds::zero()) {
          std::this_thread::sleep_for(to_wait);
          continue;
        }
        std::unique_lock<std::mutex> lock(mu_);
        WaitReadyForever();
      }
  });
  ready_thread.detach();
}

void ArduinoIO::WaitReadyForever() {
  while (GetLatestState(serial_port_) != State::READY)
    std::cerr << "error: waiting for ready" << std::endl;
  last_message_  = std::chrono::high_resolution_clock::now();
}

void ArduinoIO::SendMessage(std::string command) {
  command += "\0";
  command = Encode(command);
  port_wrapper writer{serial_port_};
  std::unique_lock<std::mutex> lock(mu_);
  for (int retry = 0; retry < 1000; ++retry) {
    if (retry > 0)
      std::cerr << "retry " << retry << " for msg " << AsBytes(command);
    if (!writer.write_msg(command, std::chrono::seconds(1))) {
      std::cerr << "write error; waiting for ready" << std::endl;
      WaitReadyForever();
      continue;
    }
    if (GetLatestState(serial_port_) == State::READY) {
      break;
    }
    WaitReadyForever();
  }
  last_message_ = std::chrono::high_resolution_clock::now();
}

void ArduinoIO::WriteOutput(Pin pin, bool value) {
  std::string command = "SET_IO";
  command.insert(command.end(), pin);
  command.insert(command.end(), value? 1 : 0);
  SendMessage(command);
}

ArduinoSerialLineIO::ArduinoSerialLineIO(const std::string &port, int baud_rate) {
  serial_port_.open(port);
  serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

void ArduinoSerialLineIO::ClearReadBufferLocked() {
  port_wrapper reader{serial_port_};
  tcflush(serial_port_.lowest_layer().native_handle(), TCIFLUSH);
  char byte;
  while (reader.read_char(byte, std::chrono::milliseconds(1))) { }
}

bool ArduinoSerialLineIO::SendLine(const char* cmd, std::size_t len) {
  std::string command(cmd, len);
  VLOG(3) << "SendLine(" << AsBytes(command) << ")" << std::endl;
  command += "\n";
  std::unique_lock<std::mutex> lock(mu_);
  return SendBytesLocked(command);
}

bool ArduinoSerialLineIO::SendBytesLocked(const std::string& command) {
  port_wrapper writer{serial_port_};
  if (writer.write_msg(command, std::chrono::milliseconds(100))) {
    tcflush(serial_port_.lowest_layer().native_handle(), TCOFLUSH);
    return true;
  }
  return false;
}

bool ArduinoSerialLineIO::SendAndRead(const char *cmd, std::size_t len, std::string* response) {
  response->clear();
  std::string command(cmd, len);
  VLOG(3) << "SendAndRead(" << AsBytes(command) << ")";
  command += "\n";
  std::unique_lock<std::mutex> lock(mu_);
  ClearReadBufferLocked();
  if (!SendBytesLocked(command)) {
    LOG(WARNING) << ": send failed" << std::endl;
    return false;
  }
  *response = ReadLineLocked();
  VLOG(3) << ": " << AsBytes(*response) << std::endl;
  return true;
}

std::string ArduinoSerialLineIO::ReadLine() {
  std::unique_lock<std::mutex> lock(mu_);
  return ReadLineLocked();
}

std::string ArduinoSerialLineIO::ReadLineLocked() {
  std::string line;
  port_wrapper reader{serial_port_};
  char byte;
  while (reader.read_char(byte, std::chrono::milliseconds(50))) {
    if (byte == '\n') return line;
    line.append(1, byte);
  }
  return line;
}
