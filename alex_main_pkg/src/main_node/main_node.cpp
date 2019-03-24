#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alex_main_pkg/camera.h"
#include <string>


void cli_callback(const std_msgs::String::ConstPtr& msg) {
//char temp_char_star[50] = msg->data.c_str();
    std::string input = msg->data;

    if (input.find("forward") != std::string::npos) {
        ROS_INFO("Moving forward...");

    } else if (input.find("left") != std::string::npos) {
        ROS_INFO("Turning left...");

    } else if (input.find("right") != std::string::npos) {
        ROS_INFO("Turning right...");

    } else if (input.find("back") != std::string::npos) {
        ROS_INFO("Moving back...");

    } else if (input == "photo") {
        ros::NodeHandle temp;
        ros::ServiceClient client = temp.serviceClient<alex_main_pkg::camera>("take_photo");
        alex_main_pkg::camera srv;

        ROS_INFO("Running OpenCV service");

        srv.request.input = "whatever";

        if (client.call(srv)) {
            ROS_INFO("%s", srv.response.output.c_str());
        } else {
            ROS_ERROR("Failed to contact camera node.");
        }

    } else {
        ROS_INFO("Invalid output");
    }
    //ROS_INFO("Received command from cli: %s", input.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_node");
    ros::NodeHandle n;

    ros::Subscriber cli_sub = n.subscribe("cli_keystrokes", 0, cli_callback);

    ros::ServiceClient client = n.serviceClient<alex_main_pkg::camera>("camera_node");
    alex_main_pkg::camera srv;

    
    ros::spin();

    return 0;
}