#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "walker.hpp"

walker::walker() {
  isObstacle = false;
}

walker::~walker() {}

void walker::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (auto i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < 0.6) {
            isObstacle = true;
            ROS_DEBUG_STREAM("Range " << msg->ranges[i] << " less than 0.6m");
            return;
        }
    }

    isObstacle = false;
    ROS_DEBUG_STREAM("No Obstacles");
    return;
}

bool walker::isClose() {
  return isObstacle;
}