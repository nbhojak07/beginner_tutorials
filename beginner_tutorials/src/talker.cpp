/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 * Copyright 2020 Nidhi Bhojak 
 * @file talker.cpp 
 * @author Nidhi Bhojak
 * 
 * @brief ROS Service
 * 
 * @section DESCRIPTION 
 * 
 * Source file for ROS service to add two integers 
 * */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include <sstream>
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include "beginner_tutorials/AddTwoInts.h"

// Funtion to provide service to add two ints
/**
 * @brief Add two integers function
 * @param req Add two integers request object 
 * @param res Add two integers response object
 * @return True on addition success
 *  **/
bool add(beginner_tutorials::AddTwoInts::Request &req,
         beginner_tutorials::AddTwoInts::Response &res) {
        res.sum = req.a + req.b;
        ROS_INFO_STREAM("Request: x = " << (long int)req.a << "y: "
        << (long int)req.b);
        ROS_INFO_STREAM("Sending Response: " << (long int)res.sum);
        return true;
         }

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
/** 
 * @brief Main function
 * @param argc: Command line number of arguments
 * @param argv: Command line arguments vector
 * @return 0
 * **/
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
// %Tag(INIT)%
// Initialise ROS Node
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
// Create ROS node handle object
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
// Loop rate set to 10Hz
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

ROS_DEBUG_STREAM("Talker started...");
// Call service
// Advertise ROS Service
ros::ServiceServer service = n.advertiseService("add_two_ints", add);
ROS_WARN_STREAM("ROS Service might take time to start...");
ROS_INFO_STREAM("Adding two ints");
ros::spin();

return 0;
}
// %EndTag(FULLTEXT)%
