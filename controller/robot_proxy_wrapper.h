#pragma once

#include <string>

#include "robot.h"

std::unique_ptr<Robot> MakeRobotProxyWrapper(const std::string& address);
