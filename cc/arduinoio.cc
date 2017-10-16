#include "arduinoio.h"

#include <boost/bind.hpp>
#include <mutex>

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
  std::cout << AsBytes(buf.str());
  return buf.str();
}

class port_wrapper {
public:
  // Constructs a blocking reader, pass in an open serial_port and
  // a timeout in milliseconds.
  port_wrapper(boost::asio::serial_port &port)
      : port(port), timer(port.get_io_service()),
        error(true) {}

  // Reads a character or times out
  // returns false if the read times out
  bool read_char(char &val, boost::posix_time::time_duration timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    val = c = '\0';
    port.get_io_service().reset();
    std::size_t bytes_read;
    boost::asio::async_read(
        port, boost::asio::buffer(&c, 1),
        boost::bind(&port_wrapper::read_complete, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred,
                    &bytes_read));
    // Setup a deadline time to implement our timeout.
    timer.expires_from_now(timeout);
    timer.async_wait(boost::bind(&port_wrapper::time_out, this,
                                 boost::asio::placeholders::error));
    port.get_io_service().run();
    if (bytes_read == 1) {
      val = c;
      return true;
    }
    return false;
  }

  bool write_msg(std::string msg, boost::posix_time::time_duration timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    port.get_io_service().reset();
    std::size_t bytes_written = 0;
    boost::asio::async_write(
        port, boost::asio::buffer(msg),
        boost::bind(&port_wrapper::read_complete, this, 
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred,
          &bytes_written));
    timer.expires_from_now(timeout);
    timer.async_wait(boost::bind(&port_wrapper::time_out, this,
                                 boost::asio::placeholders::error));
    port.get_io_service().run();
    return bytes_written == msg.size();
  }

private:
  std::mutex mutex_;
  boost::asio::serial_port &port;
  char c;
  boost::asio::deadline_timer timer;
  bool error;

  // Called when an async read completes or has been cancelled
  void read_complete(const boost::system::error_code &error,
                     size_t bytes_transferred,
                     size_t *copy_bytes_to) {
    *copy_bytes_to = error ? 0 : bytes_transferred;
    timer.cancel();
  }

  void time_out(const boost::system::error_code &error) {
    if (error) return;  // timer was cancelled.
    port.cancel();
  }
};

enum class State : char {
  READY = 'R',
  ERROR = 'E',
  UNKNOWN = 0,
};

State GetState(boost::asio::serial_port &port) {
  char c;
  port_wrapper reader(port);
  if (!reader.read_char(c, boost::posix_time::seconds(1)))
    return State::UNKNOWN;
  return c == char(State::READY) ? State::READY : State::ERROR;
}

void WaitReadyForever(boost::asio::serial_port& port) {
  while (GetState(port) != State::READY)
    std::cerr << "error: waiting for ready" << std::endl;
}

} // namespace

std::ostream& operator<<(std::ostream& os, const AsBytes& b) {
  for (char c : b.s) {
    if (isprint(c))
      os << c;
    else
      os << '\\' << uint16_t(uint8_t(c));
  }
  return os;
}

ArduinoIO::ArduinoIO(const std::string &port, int baud_rate) {
  serial_port_.open(port);
  serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  mu_.lock();
  std::thread ready_thread([&] {
      bool first = true;
      while (true) {
        if (first) {
          first = false;
        } else {
          auto since_last_cmd =
              std::chrono::high_resolution_clock::now() - last_message_;
          if (since_last_cmd < std::chrono::seconds(1)) {
            std::this_thread::sleep_for(std::chrono::seconds(1) -
                                        since_last_cmd);
            continue;
          }
          mu_.lock();
        }
        WaitReadyForever(serial_port_);
        last_message_  = std::chrono::high_resolution_clock::now();
        mu_.unlock();
      }
  });
  ready_thread.detach();
}

void ArduinoIO::SendMessage(std::string command) {
  command += "\0";
  command = Encode(command);
  port_wrapper writer{serial_port_};
  std::unique_lock<std::mutex> lock(mu_);
  for (int retry = 0; retry < 1000; ++retry) {
    if (retry > 0)
      std::cerr << "retry " << retry << " for msg " << AsBytes(command);
    if (!writer.write_msg(command, boost::posix_time::seconds(1))) {
      std::cerr << "write error; waiting for ready" << std::endl;
      WaitReadyForever(serial_port_);
      continue;
    }
    if (GetState(serial_port_) == State::READY) {
      break;
    }
    WaitReadyForever(serial_port_);
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
  while (reader.read_char(byte, boost::posix_time::milliseconds(1))) { }
}

bool ArduinoSerialLineIO::SendLine(const char* cmd, std::size_t len) {
  std::string command(cmd, len);
  std::cout << "SendLine(" << AsBytes(command) << ")" << std::endl;
  command += "\n";
  std::unique_lock<std::mutex> lock(mu_);
  return SendBytesLocked(command);
}

bool ArduinoSerialLineIO::SendBytesLocked(const std::string& command) {
  port_wrapper writer{serial_port_};
  if (writer.write_msg(command, boost::posix_time::milliseconds(100))) {
    tcflush(serial_port_.lowest_layer().native_handle(), TCOFLUSH);
    return true;
  }
  return false;
}

bool ArduinoSerialLineIO::SendAndRead(const char *cmd, std::size_t len, std::string* response) {
  response->clear();
  std::string command(cmd, len);
  std::cout << "SendAndRead(" << AsBytes(command) << ")";
  command += "\n";
  std::unique_lock<std::mutex> lock(mu_);
  ClearReadBufferLocked();
  if (!SendBytesLocked(command)) {
    std::cout << ": send failed" << std::endl;
    return false;
  }
  *response = ReadLineLocked();
  std::cout << ": " << AsBytes(*response) << std::endl;
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
  while (reader.read_char(byte, boost::posix_time::milliseconds(50))) {
    if (byte == '\n') return line;
    line.append(1, byte);
  }
  return line;
}
