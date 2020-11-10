/**
 *  Copyright 2020 Nidhi Bhojak
 *  @file talker_test.cpp
 *  @author Nidhi Bhojak
 *  @date 11/10/2020
 * 
 *  @brief Source file for Unit tests for talker
 *
 *  @section LICENSE
 *  
 * MIT License
 * Copyright (c) 2020 Nidhi Bhojak
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
 *  @section DESCRIPTION
 *
 *  Source file for talker node 
 *
 */
#include <sstream>
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"

// Funtion to provide service to add two ints
bool add(const beginner_tutorials::AddTwoInts::Request &req,
         const beginner_tutorials::AddTwoInts::Response &res) {
        res.sum = req.a + req.b;
        ROS_INFO_STREAM("Request: x = " << (long int)req.a
                              << "y: " << (long int)req.b);
        ROS_INFO_STREAM("Sending Response: " << (long int)res.sum);
        return true;
        }

/**
 * @brief Main function
 * @param argc Commandline number of arguments
 * @param argv Commandline arguments vector
 * @return 0
 */
int main(int argc, char **argv) {
  // Initialise ROS node
  ros::init(argc, argv, "talker");

  // ROS handle object
  ros::NodeHandle n;

  // Set frequency to 10Hz
  ros::Rate loop_rate(10);

ROS_DEBUG_STREAM("Talker started...");
ros::Publisher chatterPub = n.advertise<std_msgs::String>("chatter", 1000);

// Call service
ros::ServiceServer service = n.advertiseService("add_two_ints", add);
ROS_WARN_STREAM("ROS Service might take time to start...");
ROS_INFO_STREAM("Adding two ints");

while (ros::ok()) {
  // Transform braodcaster
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(10.0, 20.0, 30.0));
  tf::Quaternion q;
  q.setRPY(1.0, 0, 1.0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,
                                        ros::Time::now(),
                                        "world",
                                        "talk"));
  std_msgs::String msg;
  std::stringstream ss;
  ss << "HELLO";
  msg.data = ss.str();
  chatterPub.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();
}
ros::spin();
return 0;
}
// %EndTag(FULLTEXT)%
