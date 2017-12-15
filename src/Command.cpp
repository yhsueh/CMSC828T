#include <ros/ros.h>
#include "ardrone_autonomy/Navdata.h"
#include "Command.hpp"

void Command::stateCallback(const ardrone_autonomy::Navdata &msg) {
  state = msg.state;
  rotz = msg.rotZ;
  //ROS_INFO("QR is at %d state", state);
}
