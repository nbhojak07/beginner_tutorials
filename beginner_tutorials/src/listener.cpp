/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 * Copyright 2020 Nidhi Bhojak 
 * @file listener.cpp 
 * @author Nidhi Bhojak
 * @date 11/02/2020
 * 
 * @brief ROS Client 
 * 
 * @section LICENSE
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
 * @section DESCRIPTION 
 * 
 * Source file for ROS client to add two integers 
 * */
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
/**
 * @brief Main function
 * @param argc Command line number of arguments
 * @param argv Command line arguments vector 
 * @return 0
 *  **/
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  // Initialise ROS Node
  ros::init(argc, argv, "listener");
  // Check argument length vector
  if (argc != 3) {
    ROS_INFO_STREAM("Usage: add_two_ints_client X Y");
    return 1;
  }

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  // Create ROS Node handle object
  ros::NodeHandle n;

// ROS Service Client

ros::ServiceClient client = n.serviceClient<
              beginner_tutorials::AddTwoInts>("add_two_ints");
beginner_tutorials::AddTwoInts srv;
srv.request.a = atoll(argv[1]);
srv.request.b = atoll(argv[2]);
// If call to service successful then display sum
if ( client.call(srv) ) {
  ROS_INFO_STREAM("Sum: " << (long int)srv.response.sum);
} else {
  ROS_ERROR_STREAM("Failed to call service add_two_ints");
  ROS_FATAL_STREAM("Failed to call service add_two_ints!!");
  return -1;
}
return 0;
}
