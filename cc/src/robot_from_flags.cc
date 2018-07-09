#include <gflags/gflags.h>
#include <glog/logging.h>

#include "robot_proxy_wrapper.h"

DEFINE_string(rpc_proxy, "",
              "host:port for gRPC robot proxy. If not given, use fake.");
DEFINE_string(tty, "", "Path to Arduino device node. If not given, use fake.");
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
  } else if (!FLAGS_tty.empty()) {
    LOG(INFO) << "Connecting to serial (DEPRECATED); --tty=" << FLAGS_tty;
    return std::make_unique<RobotSerial>(FLAGS_tty, 9600);
  }
  LOG(ERROR) << "using fake robot. Provide --tty to connect to an arduino.";
  return std::make_unique<NoOpRobot>();
}
