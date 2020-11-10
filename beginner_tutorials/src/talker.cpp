/*
 *  Copyright 2020 Nidhi Bhojak
 *  @file talker.cpp
 *  @author Nidhi Bhojak
 *  @date 11/10/2020
 * 
 *  Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include <sstream>
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"

// Funtion to provide service to add two ints
bool add(beginner_tutorials::AddTwoInts::Request &req,
         beginner_tutorials::AddTwoInts::Response &res){
        res.sum = req.a + req.b;
        ROS_INFO_STREAM("Request: x = " << (long int)req.a << "y: " << (long int)req.b);
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
