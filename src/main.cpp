/********************************************************************
 *  MIT License
 *  Copyright <2018> <Chin-Po Tsai>
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 *  documentation files (the "Software"), to deal in the Software without restriction, including without limitation 
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
 *  and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all copies or substantial portions
 *  of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
 *  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 *  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 *  IN THE SOFTWARE.
 ********************************************************************/


/** @file main.cpp
 *  @brief publisher publishes velocity commands according to the subscribed sensor information
 *  @copyright MIT License
 *  @author Chin-Po Tsai
 */

#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "walker.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  ros::NodeHandle n;

  /// the object for walker class
  walker simpleWalker;
  
  /// the subscriber for laser
  ros::Subscriber sensor = n.subscribe<sensor_msgs::
          LaserScan>("/scan", 50, &walker::sensorCallback, &simpleWalker);

  /// the publisher for velocity
  ros::Publisher velocity = n.advertise<geometry_msgs::Twist>
                             ("/mobile_base/commands/velocity", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    
    /// the velocity command for turtlebot
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;


    /// the velocity decision process according to the laser information
    if (simpleWalker.isClose() == true) {
      msg.angular.z = 1;
    } else {
      msg.linear.x = 0.3;
    }

    /// publish the velocity
    velocity.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}