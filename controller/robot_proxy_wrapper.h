#ifndef __BLASTER_ROBOT_PROXY_WRAPPER_H__
#define __BLASTER_ROBOT_PROXY_WRAPPER_H__

#include <string>

#include "robot.h"

std::unique_ptr<Robot> MakeRobotProxyWrapper(const std::string& address);

#endif
