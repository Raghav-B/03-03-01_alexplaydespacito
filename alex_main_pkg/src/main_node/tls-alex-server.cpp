#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alex_main_pkg/camera.h"
#include "alex_main_pkg/cli_messages.h"

#include "make_tls_server.h"
#include "tls_common_lib.h"
#include "netconstants.h"
#include <stdint.h>
#include "../../../arduino/Alex/packet.h"
#include "../../../arduino/Alex/constants.h"
#include "serial.h"
#include "serialize.h"

/* TODO: Set PORT_NAME to the port name of your Arduino */
#define PORT_NAME			"/dev/ttyACM0"
/* END TODO */

#define BAUD_RATE			B57600

// TLS Port Number
#define SERVER_PORT			5000
#define KEY_FNAME "alex.key"
#define CERT_FNAME "alex.crt"
#define CA_CERT_FNAME "signing.pem"
#define CLIENT_NAME "laptop.play.despacito"

// Our network buffer consists of 1 byte of packet type, and 128 bytes of data
#define BUF_LEN				129

// This variable shows whether a network connection is active
// We will also use this variable to prevent the server from serving
// more than one connection, to keep connection management simple.

static volatile int networkActive;

// This variable is used by sendNetworkData to send back responses
// to the TLS connection.  It is sent by handleNetworkData

static void *tls_conn = NULL;

/*

   Alex Serial Routines to the Arduino

 */

// Prototype for sendNetworkData
void sendNetworkData(const char *, int);

// To send message from other nodes to operator
void sendMessage(const std::string message) {
  char data[129];
  if (message.size() > 127) { //account for null byte
    printf("MESSAGE TOO LONG!\n");
    return;
  }
  printf("SENDING MESSAGE TO OPERATOR: %s\n", message.c_str());
  data[0] = NET_MESSAGE_PACKET;
  memcpy(&data[1], message.c_str(), message.size() + 1);
  sendNetworkData(data, sizeof(data));
}

void handleErrorResponse(TPacket *packet) {
  printf("UART ERROR: %d\n", packet->command);
  char buffer[2];
  buffer[0] = NET_ERROR_PACKET;
  buffer[1] = packet->command;
  sendNetworkData(buffer, sizeof(buffer));
}

void handleMessage(TPacket *packet) {
  char data[33];
  printf("UART MESSAGE PACKET: %s\n", packet->data);
  data[0] = NET_MESSAGE_PACKET;
  memcpy(&data[1], packet->data, sizeof(packet->data));
  sendNetworkData(data, sizeof(data));
}

void handleStatus(TPacket *packet) {
  char data[65];
  printf("UART STATUS PACKET\n");
  data[0] = NET_STATUS_PACKET;
  memcpy(&data[1], packet->params, sizeof(packet->params));
  sendNetworkData(data, sizeof(data));
}

void handleResponse(TPacket *packet) {
  // The response code is stored in command
  switch(packet->command) {
    case RESP_OK:
      char resp[2];
      printf("Command OK\n");
      resp[0] = NET_ERROR_PACKET;
      resp[1] = RESP_OK;
      sendNetworkData(resp, sizeof(resp));
      break;

    case RESP_SAFETY_ON:
    case RESP_SAFETY_OFF:
      char resp[2];
      printf("Safety toggled\n");
      resp[0] = NET_ERROR_PACKET;
      resp[1] = packet->command;
      sendNetworkData(resp, sizeof(resp));
      break;

    case RESP_STATUS:
      handleStatus(packet);
      break;

    default:
      printf("Boo\n");
  }
}


void handleUARTPacket(TPacket *packet) {
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

void uartSendPacket(TPacket *packet) {
  char buffer[PACKET_SIZE];
  int len = serialize(buffer, packet, sizeof(TPacket));
  serialWrite(buffer, len);
}

void handleError(TResult error) {
  switch(error) {
    case PACKET_BAD:
      printf("ERROR: Bad Magic Number\n");
      break;

    case PACKET_CHECKSUM_BAD:
      printf("ERROR: Bad checksum\n");
      break;

    default:
      printf("ERROR: UNKNOWN ERROR\n");
  }
}

void *uartReceiveThread(void *p)
{
  char buffer[PACKET_SIZE];
  int len;
  TPacket packet;
  TResult result;
  int counter=0;

  while(1) {
    len = serialRead(buffer);
    counter+=len;
    if(len > 0) {
      result = deserialize(buffer, len, &packet);
      if(result == PACKET_OK) {
        counter=0;
        handleUARTPacket(&packet);
      } else if (result != PACKET_INCOMPLETE) {
        printf("PACKET ERROR\n");
        handleError(result);
      } // result
    } // len > 0
  } // while
}

/*

   Alex Network Routines

 */

void sendNetworkData(const char *data, int len) {
  // Send only if network is active
  if(networkActive) {
    // Use this to store the number of bytes actually written to the TLS connection.
    int c;
    printf("WRITING TO CLIENT\n");
    if (tls_conn != NULL) {
      c = sslWrite(tls_conn, data, len);
    }
    // Network is still active if we can write more then 0 bytes.
    networkActive = (c > 0);
  }
}

void handleCommand(void *conn, const char *buffer) {
  // The first byte contains the command
  char cmd = buffer[1];
  uint32_t cmdParam[2];
  TPacket commandPacket;
  bool valid = true;

  // Copy over the parameters and write them to the packet
  memcpy(cmdParam, &buffer[2], sizeof(cmdParam));
  commandPacket.packetType = PACKET_TYPE_COMMAND;
  commandPacket.params[0] = cmdParam[0];
  commandPacket.params[1] = cmdParam[1];
  printf("COMMAND RECEIVED: %c %lu %lu\n", cmd, (unsigned long) cmdParam[0], (unsigned long) cmdParam[1]);

  switch(cmd) {
    case 'W':
      sendMessage("Moving forward");
      commandPacket.command = COMMAND_FORWARD;
      break;

    case 'A':
      sendMessage("Turning left");
      commandPacket.command = COMMAND_TURN_LEFT;
      break;

    case 'S':
      sendMessage("Moving back");
      commandPacket.command = COMMAND_REVERSE;
      break;

    case 'D':
      sendMessage("Turning right");
      commandPacket.command = COMMAND_TURN_RIGHT;
      break;

    case 'X':
      sendMessage("Halting");
      commandPacket.command = COMMAND_STOP;
      break;

    case 'C':
      sendMessage("Clearing telemetry counters");
      commandPacket.command = COMMAND_CLEAR_STATS;
      break;

    case 'G':
      sendMessage("Fetching telemetry counters");
      commandPacket.command = COMMAND_GET_STATS;
      break;

    case 'U':
      sendMessage("Toggling safety");
      commandPacket.command = COMMAND_SAFETY;
      break;

    case 'K':
      sendMessage("Forcing forward move");
      commandPacket.command = COMMAND_FORCE_FORWARD;
      break;

    case 'J':
      sendMessage("Forcing reverse move");
      commandPacket.command = COMMAND_FORCE_REVERSE;
      break;

    case 'H':
      sendMessage("Forcing left turn");
      commandPacket.command = COMMAND_FORCE_LEFT;
      break;

    case 'L':
      sendMessage("Forcing right turn");
      commandPacket.command = COMMAND_FORCE_RIGHT;
      break;

    case 'P':
      ros::NodeHandle camera_command_handle;
      ros::ServiceClient client = camera_command_handle.serviceClient<alex_main_pkg::camera>("take_photo");
      alex_main_pkg::camera msg;
      sendMessage("Starting colour detection");
      msg.request.input = "start detection";

      if (client.call(msg)) {
        if (msg.response.output == "Detection failed") {
          sendMessage("Unknown detection error");
        } else {
          sendMessage(msg.response.output);
        }
      } else {
        sendMessage("Failed to contact camera_node");
      }
      break;

    default:
      valid = false;
      sendMessage("Bad command");
  }
  if (valid) uartSendPacket(&commandPacket);
}

void handleNetworkData(void *conn, const char *buffer, int len) {
  /* Note: A problem with our design is that we actually get data to be written
     to the SSL network from the serial port. I.e. we send a command to the Arduino,
     get back a status, then write to the TLS connection.  So we do a hack:
     we assume that whatever we get back from the Arduino is meant for the most
     recent client, so we just simply store conn, which contains the TLS
     connection, in a global variable called tls_conn */

  tls_conn = conn; // This is used by sendNetworkData
  if (buffer[0] == NET_COMMAND_PACKET) handleCommand(conn, buffer);
}

void *worker(void *conn) {
  int len;
  char buffer[BUF_LEN];
  while(networkActive) {
    len = sslRead(tls_conn, buffer, sizeof(buffer));
    // As long as we are getting data, network is active
    networkActive=(len > 0);
    if (len > 0) handleNetworkData(conn, buffer, len);
    else if(len < 0) perror("ERROR READING NETWORK: ");
  }
  // Reset tls_conn to NULL.
  tls_conn = NULL;
  EXIT_THREAD(conn);
}


void sendHello() {
  // Send a hello packet
  TPacket helloPacket;
  helloPacket.packetType = PACKET_TYPE_HELLO;
  uartSendPacket(&helloPacket);
}

int main(int argc, char **argv) {
  // Start the uartReceiveThread. The network thread is started by
  // createServer

  pthread_t serThread;
  printf("\nALEX REMOTE SUBSYSTEM\n\n");
  printf("Opening Serial Port\n");
  // Open the serial port
  startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
  printf("DONE. Waiting 3 seconds for Arduino to reboot\n");
  sleep(3);
  printf("DONE. Starting Serial Listener\n");
  pthread_create(&serThread, NULL, uartReceiveThread, NULL);
  
  printf("DONE. Starting Alex Server\n");
  networkActive = 1;
  createServer(KEY_FNAME, CERT_FNAME, SERVER_PORT, &worker, CA_CERT_FNAME,
      CLIENT_NAME, 1);
  printf("Server up. Starting ROS\n");
  ros::init(argc, argv, "main_node");
  ros::start();
  ros::Rate loop_rate(10);
  printf("DONE. Sending HELLO to Arduino\n");
  sendHello();
  printf("DONE.\n");

  // Loop while the server is active
  while (server_is_running() && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
