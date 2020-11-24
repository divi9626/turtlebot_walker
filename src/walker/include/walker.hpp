/**
 * @file walker.hpp : This is a header file which initialises Walker class to
 * avoid global variable while publishing
 * @author Divyam Garg (dgarg1@umd.edu)
 * @brief
 * @version 0.1
 * @date 2020-11-22
 * @license MIT License
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @copyright Copyright (c) 2020
 *
 */
/**
 * this behaves as pragma once
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_
/**
 * Ros headers and message headers
 */
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

class Walker {
private:
  geometry_msgs::Twist twist;
  ros::Publisher move_pub;
  ros::Subscriber laser_sub;

public:
  /**
   * @brief Construct a new Walker object
   *
   * @param n
   */
  explicit Walker(ros::NodeHandle n);
  /**
   * @brief Destroy the Walker object
   *
   */
  ~Walker();
  /**
   * @brief callback function to read scan messages
   *
   * @param scan
   */
  void laserCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);
};
#endif // include Walker.hpp