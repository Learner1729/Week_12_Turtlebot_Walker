/**
 * @file       walker.cpp
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
 * @brief Definition of walker class methods
 * @date 11-20-2018
 */

// Including user-defined header file
#include "walker.hpp"

/**
 * @brief Walker constructor
 */
Walker::Walker():collision(false) {
  ROS_DEBUG_STREAM("Turtlebot walker object is initialized successfully");
}

/**
 * @brief Walker destructor
 */
Walker::~Walker() {
  ROS_DEBUG_STREAM("Turtlebot walker node is shutting down");
}

/**
 * @brief Callback function for the laser scan reported from turtlebot
 * @param const reference variable of sensor_msgs::LaserScan type
 * @return none
 */
void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  /**
   * Check if any scan data from the laser is less than range_min + 0.2 m from
   * the front of the robot. If so, a collision is about to occur, update the
   * collision variable.
   */
  for (auto i : msg->ranges) {
    if (i < msg->range_min + 0.2) {
      collision = true;
      ROS_WARN_STREAM("About to hit... Obstacle ahead!!!");
      return;
    }
  }
  // Reset collision flag
  collision = false;
}

/**
 * @brief This function is used to return the value of private variable named
 *        collision
 * @param none
 * @return true/false
 */
bool Walker::collisionDetected() {
  return collision;
}
