#include <iostream>
#include <memory>
#include <string>

#include <glog/logging.h>
#include <grpcpp/grpcpp.h>

#include "protocol.grpc.pb.h"
#include "robot.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using google::protobuf::Empty;
using google::protobuf::Duration;

#define CHECK_OK(status)                                                       \
  do {                                                                         \
    for (auto _s = (status); !_s.ok();)                                        \
      LOG(FATAL) << #status "=" << _s.error_message() << " ok=" << _s.ok();    \
  } while (0)

class RobotProxyWrapper : public Robot {
public:
  RobotProxyWrapper(std::shared_ptr<Channel> channel)
      : stub_(RobotProxy::NewStub(channel)) {}

  Pos tell() override {
    ClientContext context;
    Empty request;
    RobotProxyPos response;
    CHECK_OK(stub_->tell(&context, request, &response));
    return {int16_t(response.first()), int16_t(response.second())};
  }

  void moveTo(Pos pos) override {
    ClientContext context;
    RobotProxyPos request;
    Empty response;
    request.set_first(pos.first);
    request.set_second(pos.second);
    CHECK_OK(stub_->moveTo(&context, request, &response));
  }

  void fire(std::chrono::milliseconds time) override {
    ClientContext context;
    Duration request;
    request.set_seconds(time / std::chrono::seconds(1));
    request.set_nanos((time - std::chrono::seconds(1) * request.seconds()) /
                      std::chrono::nanoseconds(1));
    Empty response;
    CHECK_OK(stub_->fire(&context, request, &response));
  }

private:
  std::unique_ptr<RobotProxy::Stub> stub_;
};

std::unique_ptr<Robot> MakeRobotProxyWrapper(const std::string &address) {
  // We indicate that the channel isn't authenticated (use of
  // InsecureChannelCredentials()).
  return std::make_unique<RobotProxyWrapper>(
      grpc::CreateChannel(address, grpc::InsecureChannelCredentials()));
}
