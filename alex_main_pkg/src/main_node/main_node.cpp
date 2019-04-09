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
#include "../../../arduino/packet.h"
#include "serial.h"
#include "serialize.h"
//#include "constants.h"
#include "../../../arduino/constants.h"

#define PORT_NAME			"/dev/ttyACM0" //TODO verify
#define BAUD_RATE			B57600

//TODO make the Arduino recognise the safety switch command, and report its
//status with telemetry packets

int exitFlag = 0;
sem_t _xmitSema;

/**
 * Function to check and parse command entered by user.
 *
 * @param[in] input The command as entered by the user.
 * @param[out] msg The cli_message to send to the main node.
 * @return False if invalid, true if valid (and msg will be updated
 * accordingly).
 */
bool parseCommand (const std::string &input, char &command, uint32_t &distance,
uint32_t &speed) {
  char validCmds[] = {'P', 'W', 'A', 'S', 'D', 'X', 'G', 'C', 'Q', 'U', 'H',
  'J', 'K', 'L'};
  bool isValid = false;
  std::istringstream detoken(input);
  std::string temp;
  detoken >> temp;
  if (temp.size() != 1) return false;
  command = toupper(temp[0]); //extract first character and capitalise
  for (auto &c : validCmds) {
    if (command == c) {
      isValid = true;
      break;
    }
  }
  if (!isValid) return false;
  if (command == 'P' || command == 'X' || command == 'G' || command == 'C' ||
    command == 'Q' || command == 'U') {
    distance = 0;
    speed = 0;
    return true;
  } else {
    detoken >> distance >> speed;
    if (detoken.fail()) return false;
    return true;
  }
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
  ROS_INFO("Left Forwsard Ticks:\t\t%lu", (unsigned long)packet->params[0]);
  ROS_INFO("Right Forward Ticks:\t\t%lu", (unsigned long)packet->params[1]);
  ROS_INFO("Left Reverse Ticks:\t\t%lu", (unsigned long)packet->params[2]);
  ROS_INFO("Right Reverse Ticks:\t\t%lu", (unsigned long)packet->params[3]);
  ROS_INFO("Left Forward Ticks Turns:\t\t%lu", (unsigned long)packet->params[4]);
  ROS_INFO("Right Forward Ticks Turns:\t\t%lu", (unsigned long)packet->params[5]);
  ROS_INFO("Left Reverse Ticks Turns:\t\t%lu", (unsigned long)packet->params[6]);
  ROS_INFO("Right Reverse Ticks Turns:\t\t%lu", (unsigned long)packet->params[7]);
  ROS_INFO("Forward Distance:\t\t%lu", (unsigned long)packet->params[8]);
  ROS_INFO("Reverse Distance:\t\t%lu", (unsigned long)packet->params[9]);
  ROS_INFO("---------------------------------------\n");
}

void handleResponse(TPacket *packet) {
  // The response code is stored in command
  switch(packet->command)	{
    case RESP_OK:
      ROS_INFO("Command OK");
      break;

    case RESP_SAFETY_ON:
      ROS_INFO("Safety enabled");
      break;
    
    case RESP_SAFETY_OFF:
      ROS_INFO("Safety disabled");
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
  //std::string message(packet->data);
  ROS_INFO("Message from Alex: %s", (packet->data));
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
void sendPacket(TPacket *packet) {
  char buffer[PACKET_SIZE];
  int len = serialize(buffer, packet, sizeof(TPacket));
  serialWrite(buffer, len);
}

// Shouldn't be needed but keep just in case
/* void flushInput() {
   char c;
   while((c = getchar()) != '\n' && c != EOF);
   } */

int main(int argc, char **argv) {

  // Start ROS
  ros::init(argc, argv, "main_node");
  
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

  ROS_INFO("MAIN NODE STARTED");

  ros::start();
  ros::Rate loop_rate(10);

  //pull input from user
  std::string input;
  uint32_t distance, speed;
  char command;
  while(ros::ok()) {
    std::getline(std::cin, input);
    //std::string original_input = input;
    if (!parseCommand(input, command, distance, speed)) {
      ROS_ERROR("Invalid command.");
    } else {
      TPacket commandPacket;
      bool valid = true;
      commandPacket.packetType = PACKET_TYPE_COMMAND;
      commandPacket.params[0] = distance, commandPacket.params[1] = speed;

      if (command == 'W') { // Forward movement
        ROS_INFO("Moving Forward");
        commandPacket.command = COMMAND_FORWARD;
        // Add errors here as well as need be.

      } else if (command == 'S') { // Backward movement
        ROS_INFO("Moving Back");
        commandPacket.command = COMMAND_REVERSE;
        // Add errors here as well as need be.

      } else if (command == 'A') { // Left turn
        ROS_INFO("Turning Left");
        commandPacket.command = COMMAND_TURN_LEFT;
        // Add errors here as well as need be.

      } else if (command == 'D') { // Right turn
        ROS_INFO("Turning Right");
        commandPacket.command = COMMAND_TURN_RIGHT;
        // Add errors here as well as need be.

      } else if (command == 'X') { // Halt
        ROS_INFO("Stopping");
        commandPacket.command = COMMAND_STOP;
        // Add errors here as well as need be.

      } else if (command == 'P') { // Take photo
        ros::NodeHandle camera_command_handle;
        ros::ServiceClient client = camera_command_handle.serviceClient<alex_main_pkg::camera>("take_photo");
        alex_main_pkg::camera msg;
        ROS_INFO("Starting Colour Detection");

        msg.request.input = "start detection";

        if (client.call(msg)) {
          if (msg.response.output == "Detection failed") {
            ROS_INFO("Unknown detection error");
          } else {
            ROS_INFO("%s", msg.response.output.c_str());
          }
        } else {
          ROS_ERROR("Failed to contact camera_node");
        }

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
        ROS_INFO("Stopping");
        //commandPacket.command = COMMAND_STOP;
        //TODO Any shutdown procedures for ROS? What packet to send to Arduino
        //to shut it down?

      } else if (command == 'U') {
        ROS_INFO("Toggling safety");
        commandPacket.command = COMMAND_SAFETY;

      } else if (command == 'K') { // Forward movement
        ROS_INFO("Forcing forward move");
        commandPacket.command = COMMAND_FORCE_FORWARD;
        // Add errors here as well as need be.

      } else if (command == 'J') { // Backward movement
        ROS_INFO("Forcing reverse move");
        commandPacket.command = COMMAND_FORCE_REVERSE;
        // Add errors here as well as need be.

      } else if (command == 'H') { // Left turn
        ROS_INFO("Forcing left turn");
        commandPacket.command = COMMAND_FORCE_LEFT;
        // Add errors here as well as need be.

      } else if (command == 'L') { // Right turn
        ROS_INFO("Forcing right turn");
        commandPacket.command = COMMAND_FORCE_RIGHT;
        // Add errors here as well as need be.
      
      } else { // Error or unknown command
        valid = false;
        ROS_INFO("Unknown command entered");
      }

      if (valid) sendPacket(&commandPacket);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
