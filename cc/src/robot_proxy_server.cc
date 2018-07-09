#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>
#include <grpcpp/grpcpp.h>

#include "protocol.grpc.pb.h"
#include "robot.h"
#include "robot_from_flags.h"

DEFINE_string(host, "0.0.0.0", "Host to bind to");
DEFINE_int32(port, 5500, "Port to bind to.");

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using google::protobuf::Duration;
using google::protobuf::Empty;

void CopyTo(const Robot::Pos &pos, RobotProxyPos *proto) {
  proto->set_first(pos.first);
  proto->set_second(pos.second);
}

Robot::Pos Deserialize(const RobotProxyPos &proto) {
  return Robot::Pos{static_cast<int16_t>(proto.first()),
                    static_cast<int16_t>(proto.second())};
}

// Logic and data behind the server's behavior.
class RobotProxyImpl final : public RobotProxy::Service {
public:
  RobotProxyImpl(std::unique_ptr<Robot> robot) : robot_(std::move(robot)) {
    robot_->tell(); // Fail fast if robot is broken.
  }

  Status tell(ServerContext *, const Empty *, RobotProxyPos *reply) override {
    Robot::Pos pos = robot_->tell();
    VLOG(3) << "tell() => " << pos;
    CopyTo(pos, reply);
    return Status::OK;
  }

  Status moveTo(ServerContext *, const RobotProxyPos *pos, Empty *) override {
    VLOG(2) << "moveTo(" << Deserialize(*pos) << ")";
    robot_->moveTo(Deserialize(*pos));
    return Status::OK;
  }

  Status fire(ServerContext *, const Duration *d, Empty *) override {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::seconds(d->seconds()) +
        std::chrono::nanoseconds(d->nanos()));
    VLOG(1) << "fire(" << (duration / std::chrono::milliseconds(1)) << ")";
    robot_->fire(duration);
    return Status::OK;
  }

private:
  std::unique_ptr<Robot> robot_;
};

void RunServer() {
  std::string server_address(FLAGS_host + ":" + std::to_string(FLAGS_port));
  RobotProxyImpl service(RobotFromFlags());

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  RunServer();
  return 0;
}
