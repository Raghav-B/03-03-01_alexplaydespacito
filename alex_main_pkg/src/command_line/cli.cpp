#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

int main(int argc, char**argv) {
    ros::init(argc, argv, "cli");
    ros::NodeHandle n;

    ros::Publisher cli_pub = n.advertise<std_msgs::String>("cli_keystrokes", 0);
    
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        //char input[50];
        //gets(input);
        std::string input;
        //std::cin >> input;
        std::getline(std::cin, input);

        ROS_INFO("Data sent: %s", input.c_str());

        std_msgs::String msg;
        msg.data = input;

        cli_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}