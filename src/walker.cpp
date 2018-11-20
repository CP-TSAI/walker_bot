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


/** @file walker.cpp
 *  @brief the implementation of the walker algorithm
 *  @copyright MIT License
 *  @author Chin-Po Tsai
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "walker.hpp"


/// the constructor of the class
walker::walker() {
  isObstacle = false;
}


/// the destructor of the class
walker::~walker() {}


/**
 * @brief A callback function for laser data
 * @param msg
 * @return void
 */
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


/**
 * @brief returns if an obstacle is ahead
 * @param void
 * @return bool
 */
bool walker::isClose() {
  return isObstacle;
}