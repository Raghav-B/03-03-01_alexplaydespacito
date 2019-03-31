#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alex_main_pkg/cli_messages.h"
#include <ctype.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

/**
 * Function to check and parse command entered by user.
 *
 * @param[in] input The command as entered by the user.
 * @param[out] msg The cli_message to send to the main node.
 * @return False if invalid, true if valid (and msg will be updated
 * accordingly).
 */
bool parse_command (const std::string &input, alex_main_pkg::cli_messages &msg) {
  char validCmds[] = {'P', 'W', 'A', 'S', 'D', 'X'};
  bool isValid = false;
  std::istringstream detoken(input);
  std::string temp; unsigned int dist, speed; char command;
  input >> temp;
  if (temp.size() != 1) return false;
  command = toupper(temp[0]); //extract first character and capitalise
  for (auto &c : valid) {
    if (command == c) {
      isValid = true;
      break;
    }
  }
  if (!isValid) return false;
  if (command == 'P' || command == 'X') {
    msg.request.action = command;
    msg.request.distance = 0;
    msg.request.speed = 0;
    return true;
  } else {
    command >> dist >> speed;
    if (command.fail()) return false;
    msg.request.action = command;
    msg.request.distance = dist;
    msg.request.speed = speed;
    return true;
  }
}

int main(int argc, char **argv) {
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
    if (!parse_command(input, msg)) {
      //TODO invalid command, feedback to user
      ROS_ERROR("Invalid command.");
    } else if (cli_client.call(msg)) {
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
