#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alex_main_pkg/cli_messages.h"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

int main(int argc, char**argv) {
    ros::init(argc, argv, "cli");
    ros::NodeHandle cli_handle;
    ros::ServiceClient cli_client = cli_handle.serviceClient<alex_main_pkg::cli_messages>("cli_command");
    alex_main_pkg::cli_messages msg;
    ROS_INFO("Command Line Node Started");

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        std::string input;
        std::getline(std::cin, input);
        std::string original_input = input;

        if (input != "P" && input != "p") {
            std::vector<std::string> input_vector; std::string temp;
            std::istringstream detoken(input);
            while (std::getline(detoken, temp, ' ')) {
                input_vector.push_back(temp);
            }   

            // Sending input data in appropriate format
            msg.request.action = input_vector[0];
            msg.request.distance = std::stoi(input_vector[1]);
            msg.request.speed = std::stoi(input_vector[2]);
        } else {
            msg.request.action = input;
            msg.request.distance = 0;
            msg.request.speed = 0;
        }
        // Performing some string manipulation to get the appropriate command format.
        
        
        if (cli_client.call(msg)) {
            //if (msg.response.result == "success") {
            //    ROS_INFO("%s - Performed Successfully", original_input.c_str());
            //} else {
            //    ROS_ERROR("%s - Action Unsuccessful", original_input.c_str());
            //}
            ROS_INFO("%s", msg.response.result.c_str());
        } else {
            ROS_ERROR("Failed to contact main_node");
        }

        //ROS_INFO("Data sent: %s", input.c_str());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}