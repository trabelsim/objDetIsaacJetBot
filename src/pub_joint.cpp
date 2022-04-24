#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

using namespace std;


void readJointcallback(const sensor_msgs::JointState::ConstPtr& msg){
    // ROS_INFO("Name: %s", msg->name);
    // ROS_INFO("Velocity: %s", msg->velocity);
}


int main(int argc, char **argv)
{
    /* ROS INITIALIZER */
    ros::init(argc, argv, "publish_joint");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/joint_command",10);
    ros::Subscriber sub = n.subscribe("/joint_states",10, readJointcallback);
    ros::Rate loop_rate(100);
    sensor_msgs::JointState msg;
    float vel = 12;

    while(ros::ok()){
        vel += 0.1;
        ROS_INFO("Vel: %f", vel);
        // if(num == 0){
            msg.name.push_back("right_wheel_joint");
            msg.velocity.push_back(vel);
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            // ROS_INFO("Publish from num: %d", num);
            // num = 1;
        // }
        // else{
        //     msg.name.push_back("left_wheel_joint");
        //     msg.velocity.push_back(10);
        //     pub.publish(msg);
        //     ros::spinOnce();
        //     loop_rate.sleep();
        //     ROS_INFO("Publish from num: %d", num);
        //     num = 0;   
        // }
        // ROS_INFO("end");
    }
    //ros::spin();

    return 0;
}
