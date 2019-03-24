#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alex_main_pkg/camera.h"
#include "alex_main_pkg/cli_messages.h"
#include <string>

bool execute_cli_command(alex_main_pkg::cli_messages::Request &req, alex_main_pkg::cli_messages::Response &res) {
    std::string command = req.action;
    int16_t distance = req.distance;
    int16_t speed = req.speed;

    if (command == "W" || command == "w") { // Forward movement
        ROS_INFO("Moving Forward");
        std::string temp = "Success - Forward by " + std::to_string(distance) + " at speed " + std::to_string(speed);
        res.result = temp;
        // Add errors here as well as need be.

    } else if (command == "S" || command == "s") { // Backward movement
        ROS_INFO("Moving Back");
        std::string temp = "Success - Backward by " + std::to_string(distance) + " at speed " + std::to_string(speed);
        res.result = temp;
        // Add errors here as well as need be.

    } else if (command == "A" || command == "a") { // Left turn
        ROS_INFO("Turning Left");
        std::string temp = "Success - Left by " + std::to_string(distance) + " at speed " + std::to_string(speed);
        res.result = temp;
        // Add errors here as well as need be.

    } else if (command == "D" || command == "d") { // Right turn
        ROS_INFO("Turning Right");
        std::string temp = "Success - Right by " + std::to_string(distance) + " at speed " + std::to_string(speed);
        res.result = temp;
        // Add errors here as well as need be.

    } else if (command == "P" || command == "p") { // Take photo
        ros::NodeHandle camera_command_handle;
        ros::ServiceClient client = camera_command_handle.serviceClient<alex_main_pkg::camera>("take_photo");
        alex_main_pkg::camera msg;
        ROS_INFO("Starting Colour Detection");

        msg.request.input = "start detection";

        if (client.call(msg)) {
            if (msg.response.output == "Detection Failed") {
                res.result = "Unknown detection error";
            } else {
                res.result = msg.response.output;
            }
        } else {
            ROS_ERROR("Failed to contact camera_node");
        }
        // Add errors here as well as need be.

    } else { // Error or unknown command
        res.result = "Unknown command entered";
    }

    return true;
}

//void cli_callback(const std_msgs::String::ConstPtr& msg) {
//char temp_char_star[50] = msg->data.c_str();
//    std::string input = msg->data;  
    //ROS_INFO("Received command from cli: %s", input.c_str());
//}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_node");
    ros::NodeHandle main_node_handle;
    ros::ServiceServer cli_server = main_node_handle.advertiseService("cli_command", execute_cli_command);
    ROS_INFO("Main Node Started");

    //ros::Subscriber cli_sub = n.subscribe("cli_keystrokes", 0, cli_callback);
    //ros::ServiceClient client = n.serviceClient<alex_main_pkg::camera>("camera_node");
    //alex_main_pkg::camera srv;

    ros::spin();

    return 0;
}