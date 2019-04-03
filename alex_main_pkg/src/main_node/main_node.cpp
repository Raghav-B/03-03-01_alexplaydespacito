#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alex_main_pkg/camera.h"
#include "alex_main_pkg/cli_messages.h"
#include <string>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"

#define PORT_NAME			"/dev/ttyACM0" //TODO verify
#define BAUD_RATE			B57600

int exitFlag = 0;
sem_t _xmitSema;

//TODO Send all error messages to cli node

void sendPacket(TPacket *packet) {
  char buffer[PACKET_SIZE];
  int len = serialize(buffer, packet, sizeof(TPacket));
  serialWrite(buffer, len);
}

bool execute_cli_command(alex_main_pkg::cli_messages::Request &req, alex_main_pkg::cli_messages::Response &res) {
  char command = req.action;
  uint32_t speed = req.speed;
  uint32_t distance = req.distance;
  TPacket commandPacket;
  bool valid = true;
  commandPacket.packetType = PACKET_TYPE_COMMAND;
  commandPacket.params[0] = distance, commandPacket.params[1] = speed;

  if (command == 'W') { // Forward movement
    ROS_INFO("Moving Forward");
    commandPacket.command = COMMAND_FORWARD;
    std::string temp = "Success - Forward by " + std::to_string(distance) + " at speed " + std::to_string(speed);
    res.result = temp;
    // Add errors here as well as need be.

  } else if (command == 'S') { // Backward movement
    ROS_INFO("Moving Back");
    commandPacket.command = COMMAND_REVERSE;
    std::string temp = "Success - Backward by " + std::to_string(distance) + " at speed " + std::to_string(speed);
    res.result = temp;
    // Add errors here as well as need be.

  } else if (command == 'A') { // Left turn
    ROS_INFO("Turning Left");
    commandPacket.command = COMMAND_TURN_LEFT;
    std::string temp = "Success - Left by " + std::to_string(distance) + " at speed " + std::to_string(speed);
    res.result = temp;
    // Add errors here as well as need be.

  } else if (command == 'D') { // Right turn
    ROS_INFO("Turning Right");
    commandPacket.command = COMMAND_TURN_RIGHT;
    std::string temp = "Success - Right by " + std::to_string(distance) + " at speed " + std::to_string(speed);
    res.result = temp;
    // Add errors here as well as need be.

  } else if (command == 'X') { // Halt
    ROS_INFO("Stopping");
    commandPacket.command = COMMAND_STOP;
    std::string temp = "Success - Stopped";
    res.result = temp;
    // Add errors here as well as need be.

  } else if (command == 'P') { // Take photo
    ros::NodeHandle camera_command_handle;
    ros::ServiceClient client = camera_command_handle.serviceClient<alex_main_pkg::camera>("take_photo");
    alex_main_pkg::camera msg;
    ROS_INFO("Starting Colour Detection");

    msg.request.input = "Start detection";

    if (client.call(msg)) {
      if (msg.response.output == "Detection failed") {
        res.result = "Unknown detection error";
      } else {
        res.result = msg.response.output;
      }
    } else {
      ROS_ERROR("Failed to contact camera_node");
    }
    // Add errors here as well as need be.
  
  } else if (command == 'C') { // Clear stats
    //NOTE Studio code misleadingly hints that individual counters can be
    //cleared, but can only clear all counters. As we would not need to clear
    //individual counters, we'll just stick to that
    ROS_INFO("Clearing telemetry counters");
    commandPacket.command = COMMAND_CLEAR_STATS;
  
  } else if (command == 'G') { // Get stats
    ROS_INFO("Reading telemetry");
    commandPacket.command = COMMAND_GET_STATS;
  
  } else if (command == 'Q') { // Quit
    /*ROS_INFO("Stopping");
    commandPacket.command = COMMAND_STOP;
    std::string temp = "Success - Stopped";
    res.result = temp;*/
    //TODO How to make it quit?
  
  } else { // Error or unknown command
    valid = false;
    res.result = "Unknown command entered";
  }

  if (valid) sendPacket(&commandPacket);

  return true;
}

//void cli_callback(const std_msgs::String::ConstPtr& msg) {
//char temp_char_star[50] = msg->data.c_str();
//    std::string input = msg->data;  
//ROS_INFO("Received command from cli: %s", input.c_str());
//}

void handleError(TResult error) {
  switch(error) {
    case PACKET_BAD:
      ROS_ERROR("ERROR: Bad Magic Number");
      break;

    case PACKET_CHECKSUM_BAD:
      ROS_ERROR("ERROR: Bad checksum");
      break;

    default:
      ROS_ERROR("ERROR: UNKNOWN ERROR");
  }
}

void handleStatus(TPacket *packet) {
  ROS_INFO(" ------- ALEX STATUS REPORT ------- ");
  /*ROS_INFO(("Left Forward Ticks:\t\t" + std::to_string(packet->params[0])).c_str());
  ROS_INFO(("Right Forward Ticks:\t\t" + std::to_string(packet->params[1])).c_str());
  ROS_INFO(("Left Reverse Ticks:\t\t" + std::to_string(packet->params[2])).c_str());
  ROS_INFO(("Right Reverse Ticks:\t\t" + std::to_string(packet->params[3])).c_str());
  ROS_INFO(("Left Forward Ticks Turns:\t" + std::to_string(packet->params[4])).c_str());
  ROS_INFO(("Right Forward Ticks Turns:\t" + std::to_string(packet->params[5])).c_str());
  ROS_INFO(("Left Reverse Ticks Turns:\t" + std::to_string(packet->params[6])).c_str());
  ROS_INFO(("Right Reverse Ticks Turns:\t" + std::to_string(packet->params[7])).c_str());
  ROS_INFO(("Forward Distance:\t\t" + std::to_string(packet->params[8])).c_str());
  ROS_INFO(("Reverse Distance:\t\t" + std::to_string(packet->params[9])).c_str());
  ROS_INFO("---------------------------------------\n");*/
}

void handleResponse(TPacket *packet) {
  // The response code is stored in command
  switch(packet->command)	{
    case RESP_OK:
      ROS_INFO("Command OK");
      break;

    case RESP_STATUS:
      handleStatus(packet);
      break;

    default:
      ROS_INFO("Alex is confused.");
  }
}

void handleErrorResponse(TPacket *packet) {
  // The error code is returned in command
  switch(packet->command) {
    case RESP_BAD_PACKET:
      ROS_INFO("Arduino received bad magic number");
      break;

    case RESP_BAD_CHECKSUM:
      ROS_INFO("Arduino received bad checksum");
      break;

    case RESP_BAD_COMMAND:
      ROS_INFO("Arduino received bad command");
      break;

    case RESP_BAD_RESPONSE:
      ROS_INFO("Arduino received unexpected response");
      break;

    default:
      ROS_INFO("Arduino reports a weird error");
  }
}

void handleMessage(TPacket *packet) {
  std::string message(packet->data);
  //ROS_INFO("Message from Alex: " + message);
}

void handlePacket(TPacket *packet) {
  switch(packet->packetType) {
    case PACKET_TYPE_COMMAND:
      // Only we send command packets, so ignore
      break;

    case PACKET_TYPE_RESPONSE:
      handleResponse(packet);
      break;

    case PACKET_TYPE_ERROR:
      handleErrorResponse(packet);
      break;

    case PACKET_TYPE_MESSAGE:
      handleMessage(packet);
      break;
  }
}

void *receiveThread(void *p) {
  char buffer[PACKET_SIZE];
  int len;
  TPacket packet;
  TResult result;
  int counter=0;

  while (1) {
    len = serialRead(buffer);
    counter+=len;
    if(len > 0) {
      result = deserialize(buffer, len, &packet);
      if(result == PACKET_OK)	{
        counter=0;
        handlePacket(&packet);
      }	else {
        if (result != PACKET_INCOMPLETE)	{
          ROS_INFO("PACKET ERROR");
          handleError(result);
        }
      }
    }
  }
}

// Shouldn't be needed but keep just in case
/* void flushInput() {
   char c;
   while((c = getchar()) != '\n' && c != EOF);
   } */

int main(int argc, char **argv) {

  // Start ROS
  ros::init(argc, argv, "main_node");
  ros::NodeHandle main_node_handle;
  ros::ServiceServer cli_server = main_node_handle.advertiseService("cli_command", execute_cli_command);
  
  // Connect to the Arduino
  startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

  // Sleep for two seconds
  ROS_INFO("WAITING TWO SECONDS FOR ARDUINO TO REBOOT");
  sleep(2);
  ROS_INFO("DONE");

  // Spawn receiver thread
  pthread_t recv;
  pthread_create(&recv, NULL, receiveThread, NULL);

  // Send a hello packet
  TPacket helloPacket;
  helloPacket.packetType = PACKET_TYPE_HELLO;
  sendPacket(&helloPacket);

  ROS_INFO("Main Node Started");

  //ros::Subscriber cli_sub = n.subscribe("cli_keystrokes", 0, cli_callback);
  //ros::ServiceClient client = n.serviceClient<alex_main_pkg::camera>("camera_node");
  //alex_main_pkg::camera srv;

  ros::spin();
  return 0;
}
