/**
 * @file       turtlebot_walker_node.cpp
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
 * @brief ROS 
 * @date 11-20-2018
 */

/**
 * Includes the geometry_msgs/Twist message header file, which resides in the
 * geometry_msgs package. The header file is generated automatically from the 
 * Twist.msg file in that package.
 */
#include "geometry_msgs/Twist.h"

// Including user-defined header file
#include "walker.hpp"

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command 
   * line. For programmatic remappings you can use a different version of 
   * init() which takes remappings directly, but for most command-line programs
   * passing argc and argv is the easiest way to do it.  The third argument to
   * init() is the name of the node. Node names must be unique.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   * 
   * Initializing "walker" node for turtlebot
   */
  ros::init(argc, argv, "walker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the 
   * last NodeHandle destructed will close down the node.
   *
   * Initializing the node handler for "walker" node.
   */
  ros::NodeHandle nh;

  // Creating the Walker object
  Walker w;

  /**
   * Subscribing to the "/scan" topic to listen for any messages being 
   * published on that topic.
   * - The first parameter to subscribe function is the topic name
   * - The second parameter is buffer size (Set to 100 messages)
   * - Finally the third & forth parameter is the callback to the 
   *   laserCallback method of walker class & object of walker class
   */
  ros::Subscriber sub = nh.subscribe <sensor_msgs::LaserScan>
  ("/scan", 100, &Walker::laserCallback, &w);

  /**
   * Publish to the "/mobile_base/commands/velocity" topic to control 
   * the turtlebot. 
   * - The first parameter to advertise() function is the topic name
   * - The second parameter to advertise() function is the size of the message
   *   queue used for publishing messages.
   */
  ros::Publisher velocityPub = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);

  /**
   * A ros::Rate object allows us to specify a frequency that we would like to
   * loop at. Here the loop rate is 10 Hz, that means the data is published 10
   * times per second
   */
  ros::Rate loop_rate(10);

  // Declare the twist message variable and initialize it to zero
  geometry_msgs::Twist velocityMessage;
  velocityMessage.linear.x = 0.0;
  velocityMessage.linear.y = 0.0;
  velocityMessage.linear.z = 0.0;
  velocityMessage.angular.x = 0.0;
  velocityMessage.angular.y = 0.0;
  velocityMessage.angular.z = 0.0;

  /**
   * ros::ok(), it is used to check if the node should continue running or not
   * ros::ok(), will return false if 
   *  - a SIGINT is received (Ctrl + C)
   *  - if ros::shutdown() has been called by another part of the code
   *  - if all ros::NodeHandles have been destroyed
   * Thus, it will keep spinning until above events occurs
   */
  while (ros::ok()) {
    /**
     * While moving forward, check whether a collision is about to occur or not.
     * - If yes, stop moving forward and rotate about the z-axis
     * - If no, stop rotating and move forward
     */
    if (w.collisionDetected()) {
      ROS_INFO_STREAM("Making a turn.....");
      // Set linear velocity to zero
      velocityMessage.linear.x = 0.0;
      // Set turn rate about the z-axis
      velocityMessage.angular.z = 1.0;
    } else {
      // Set turn rate to zero
      velocityMessage.angular.z = 0.0;
      // Move forward slowly
      velocityMessage.linear.x = 0.1;
    }

    // Publish the twist message to everyone listening
    velocityPub.publish(velocityMessage);

    // "Spin" a callback in case we set up any callbacks
    ros::spinOnce();

    /**
     * Needs to call sleep() function over here, so that it makes the system
     * sleep for the remaining duration, to enforce the loop rate.
     */
    loop_rate.sleep();
  }
  return 0;
}
