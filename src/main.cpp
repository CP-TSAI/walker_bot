#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "walker.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");

  walker simpleWalker;
  ros::NodeHandle n;

  ros::Subscriber sensor = n.subscribe<sensor_msgs::
          LaserScan>("/scan", 50, &walker::sensorCallback, &simpleWalker);

  ros::Publisher velocity = n.advertise<geometry_msgs::Twist>
                             ("/mobile_base/commands/velocity", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    geometry_msgs::Twist msg;

    // Initiaizing msg with 0
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    if (simpleWalker.isClose() == true) {
      // rotate until obstacle out of the range
      msg.angular.z = 1;
    } else {
      // move forward
      msg.linear.x = 0.3;
    }

    velocity.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}