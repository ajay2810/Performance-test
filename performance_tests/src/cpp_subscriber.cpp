/*********************************************************************
*
*  Copyright (c) 2011, Ajay Kumar
*  All rights reserved.
*
*********************************************************************/

#include "ros/ros.h"
#include "performance_tests/SuperAwesome.h"  // Custom created string message
#include "std_msgs/Float64.h"  // Double message used for real rate publication for plot usage

// GLOBAL VARIABLES
double previous_time = 0;  // Initialize previous time for the first incoming message
double time_now, elapsed_time, freq_rate;
// The publisher variable should be declared global, as the message will be sent inside the Subscriber callback
ros::Publisher chatter_pub;

/**
 * This demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const performance_tests::SuperAwesome::ConstPtr& msg)
{
  time_now = ros::Time::now().toSec();  // Get actual time
  // Obtain elapsed time from the difference between actual and previous time
  elapsed_time = time_now - previous_time;
  // Assign the actual time to the previous one in order to use it on the next incoming message
  previous_time = time_now;
  freq_rate = 1 / elapsed_time;  // Obtain frequency rate from elapsed time
  ROS_INFO("REAL RATE %f", freq_rate);

  std_msgs::Float64 msg_;  // A double message is declared
  msg_.data = freq_rate;  // Assign the frequency rate to the message
  chatter_pub.publish(msg_);  // Publish the frequency rate to be read by the rqt_plot

  // ROS_INFO("I heard: [%s]", msg->SuperAwesome.c_str());
}

int main(int argc, char **argv)
{
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
  ros::init(argc, argv, "cpp_subscriber");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  chatter_pub = n.advertise<std_msgs::Float64>("/RealRate", 1000);  // Real Frequency Rate Publisher

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/SuperAwesomeTopic", 1000, chatterCallback);  // Custom string message subscriber

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
