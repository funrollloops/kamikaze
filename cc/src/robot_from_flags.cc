#include <gflags/gflags.h>
#include <glog/logging.h>

#include "robot_proxy_wrapper.h"

DEFINE_string(rpc_proxy, "",
              "host:port for gRPC robot proxy. If not given, use fake.");
DEFINE_string(arduinoio_tty, "",
              "Path to Arduino device node. If not given, use fake.");
DEFINE_string(line_tty, "",
              "Path to Arduino device node. If not given, use fake.");
DEFINE_string(spi, "", "Path to spi device node. If not given, use fake.");
DEFINE_int32(spi_speed, 100000, "bits/sec for SPI interface.");

std::unique_ptr<Robot> RobotFromFlags() {
  if (!FLAGS_spi.empty()) {
    LOG(INFO) << "Connecting to SPI; --spi=" << FLAGS_spi
              << " --spi_speed=" << FLAGS_spi_speed;
    return std::make_unique<RobotLinuxSPI>(FLAGS_spi, FLAGS_spi_speed);
  } else if (!FLAGS_rpc_proxy.empty()) {
    LOG(INFO) << "Connecting to gRPC server --rpc_proxy=" << FLAGS_rpc_proxy;
    return MakeRobotProxyWrapper(FLAGS_rpc_proxy);
  } else if (!FLAGS_line_tty.empty()) {
    LOG(INFO) << "Connecting to simple serial; --line_tty=" << FLAGS_line_tty;
    return std::make_unique<RobotSerialLineIO>(FLAGS_line_tty, 9600);
  } else if (!FLAGS_arduinoio_tty.empty()) {
    LOG(INFO) << "Connecting to ArduinoIO serial; --arduinoio_tty="
              << FLAGS_arduinoio_tty;
    return std::make_unique<RobotArduinoIO>(FLAGS_arduinoio_tty, 9600);
  }
  LOG(ERROR) << "using fake robot. Provide --tty to connect to an arduino.";
  return std::make_unique<NoOpRobot>();
}
