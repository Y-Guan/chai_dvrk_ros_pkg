#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "chai/demo_msg.h"
#include <iostream>

void number_callback(const std_msgs::Int32::ConstPtr & msg)
{
    // display the msg when callback
    ROS_INFO("Received [%d]", msg->data);
}

void number_callback2(const chai::demo_msg::ConstPtr & msg)
{
    // display the msg when callback
    ROS_INFO("Received string [%s]", msg->greeting.c_str());
}

int main(int argc, char **argv)
{
    // init ros subscriber node with name "demo_topic_subscriber_node"
    ros::init(argc, argv, "demo_topic_subscriber_node");
    // create object for the node
    ros::NodeHandle node_obj;
    // create the ros subscriber and subscribe
    ros::Subscriber number_subscriber = node_obj.subscribe("/numbers",10,number_callback);
    ros::Subscriber string_subscriber = node_obj.subscribe("/string",10,number_callback2);
    // read and update topics
    ros::spin();
    return 0;
}
