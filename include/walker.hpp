#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


class walker {
 public:
  walker();
  ~walker();
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  bool isClose();

 private:
    bool isObstacle;
};

#endif