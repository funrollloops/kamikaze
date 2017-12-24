#include "robot.h"

#include <iostream>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>

#include "logging.h"

std::ostream& operator<<(std::ostream& os, const Robot::Pos& pos) {
  return os << pos.first << ", " << pos.second;
}

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

RobotLinuxSPI::RobotLinuxSPI(const std::string& device, int baud) {
  fd_ = open(device.c_str(), O_RDWR);
  QCHECK(fd_ >= 0) << " failed to open device: " << device;
  uint8_t mode = 0;
  uint8_t bits = 8;
  uint32_t speed = 100000;
  QCHECK(ioctl(fd_, SPI_IOC_WR_MODE, &mode) != -1);
	QCHECK(ioctl(fd_, SPI_IOC_RD_MODE, &mode) != -1);
	QCHECK(ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) != -1);
	QCHECK(ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits) != -1);
	QCHECK(ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) != -1);
	QCHECK(ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed) != -1);
  std::cout << "SPI configured to mode=" << +mode << " bits=" << +bits
            << " speed=" << speed << "Hz" << std::endl;
}

RobotLinuxSPI::~RobotLinuxSPI() { close(fd_); }

Robot::Pos RobotLinuxSPI::tell() {
  struct TellCmd { Tag tag; Pos pos; } __attribute__((__packed__));
  const TellCmd m{TELL};
  auto r = sendMessage(m);
  for (int i = 1; m.tag != r.tag; ++i) {
    std::cerr << "tell() retry " << i << " r[1]=" << r.tag << std::endl;
    r = sendMessage(m);
  }
  return r.pos;
}

void RobotLinuxSPI::moveTo(Pos pos) {
  struct MoveToCmd { Tag tag; Pos pos; } __attribute__((__packed__));
  const MoveToCmd m{SEEK, pos};
  auto r = sendMessage(m);
  for (int i = 1; r.tag != m.tag || r.pos != m.pos; ++i) {
    std::cerr << "moveTo() retry " << i << std::endl;
    r = sendMessage(m);
  }
}

void RobotLinuxSPI::fire(std::chrono::milliseconds time) {
  struct FireCmd { Tag tag; uint16_t ticks; } __attribute__((__packed__));
  const FireCmd m{FIRE, uint16_t(time / std::chrono::milliseconds(1))};
  std::cerr << "fire ticks=" << (time / std::chrono::milliseconds(1))
            << std::endl;
  auto r = sendMessage(m);
  for (int i = 1; r.tag != m.tag || r.ticks != m.ticks; ++i) {
    std::cerr << "fire(" << m.ticks << ") retry " << i << std::endl;
    r = sendMessage(m);
  }
}

std::string RobotLinuxSPI::sendMessage(const void* send, uint16_t len) {
  std::string response(len, '\0');
  spi_ioc_transfer tr{
    .tx_buf = reinterpret_cast<uint64_t>(send),
    .rx_buf = reinterpret_cast<uint64_t>(response.data()),
    .len = len,
    .speed_hz = 0,
    .delay_usecs = 500,
  };
  QCHECK(ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) >= 1);
  std::cout
      << "i2c r/w: "
      << AsBytes(std::string(reinterpret_cast<const char*>(send), len))
      << " -> " << AsBytes(response)
      << std::endl;
  return response;
}

template <typename T>
T RobotLinuxSPI::sendMessage(const T& t) {
  std::string m(reinterpret_cast<const char*>(&t), sizeof(t));
  m.push_back('\0');
  auto r = sendMessage(m.data(), m.size());
  r.erase(0, 1);
  return *reinterpret_cast<const T*>(r.data());
}
