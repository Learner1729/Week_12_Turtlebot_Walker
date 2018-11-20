/**
 * @file       walker.hpp
 * @author     Ashish Patel
 * @version    1.0
 * @copyright  MIT License (c) 2018 Ashish Patel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @brief ROS walker class header file
 * @date 11-20-2018
 */
#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

/**
 * Includes all the headers necessary to use the most common public pieces of
 * the ROS system
 */
#include "ros/ros.h"

/**
 * Includes the sensor_msgs/LaserScan message, which resides in the sensor_msgs
 * package. The header file is generated automatically from the LaserScan.msg file
 * in that package.
 */
#include "sensor_msgs/LaserScan.h"

/**
 * @brief Walker class is used to implment simple walker algorithm on turtlebot
 * @description It has two functions laserCallback function and the 
 *              collisiionDetected function. The first one is used to check
 *              whether the distance between obstacle and robot is within the
 *              permissible limit. If not it updates the private variable of
 *              Walker class (i.e. collision) & the other function outputs the
 *              the state of collision class variable.
 */
class Walker {
 private:
  /**
   * @brief Boolean variable denoting if a collision is about to occur or not
   * @default false
   */
  bool collision;

 public:
  /**
   * @brief Walker constructor
   */
  Walker();

  /**
   * @brief Walker destructor
   */
  ~Walker();

  /**
   * @brief Callback function for the laser scan reported from turtlebot
   * @param const reference variable of sensor_msgs::LaserScan type
   * @return none
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief This function is used to return the value of private variable named
   *        collision
   * @param none
   * @return true/false
   */
  bool collisionDetected();
};

#endif  // INCLUDE_WALKER_HPP_
