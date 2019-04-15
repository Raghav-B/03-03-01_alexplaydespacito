// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "../arduino/Alex/constants.h"
/* TODO: #define filenames for the client private key, certificatea,
   CA filename, etc. that you need to create a client */

#define CA_CERT_FNAME "signing.pem"
#define CLIENT_CERT_FNAME "laptop.crt"
#define CLIENT_KEY_FNAME "laptop.key"
#define SERVER_NAME_ON_CERT "alex.play.despacito"

/* END TODO */

// Tells us that the network is running.
static volatile int networkActive=0;

/**
 * Function to check and parse command entered by user.
 *
 * @param[in] input The command as entered by the user.
 * @param[out] command The uppercase letter identifying the command.
 * @param[out] distance The distance to move through or angle to turn through.
 * @param[out] speed The speed at which to turn or move.
 * @return False if invalid, true if valid 
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
  } else {
    detoken >> distance;
    if (detoken.fail()) {
      if (command == 'W' || command == 'S') distance = 10;
      else if (command == 'J' || command == 'K' || command == 'H' || command ==
          'L') distance = 500;
      else distance = 15;
      detoken.clear();
    }
    detoken >> speed;
    if (detoken.fail()) {
      if (command == 'W' || command == 'S' || command == 'J' || command == 'K') speed = 75;
      else speed = 100;
    }
  }
  return true;
}

void handleError(const char *buffer)
{
  switch(buffer[1])
  {
    case RESP_OK:
      printf("Command / Status OK\n");
      break;

    case RESP_SAFETY_ON:
      printf("SAFETY ENABLED\n");
      break;

    case RESP_SAFETY_OFF:
      printf("SAFETY DISABLED\n");
      break;

    case RESP_BAD_PACKET:
      printf("BAD MAGIC NUMBER FROM ARDUINO\n");
      break;

    case RESP_BAD_CHECKSUM:
      printf("BAD CHECKSUM FROM ARDUINO\n");
      break;

    case RESP_BAD_COMMAND:
      printf("PI SENT BAD COMMAND TO ARDUINO\n");
      break;

    case RESP_BAD_RESPONSE:
      printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
      break;

    default:
      printf("PI IS CONFUSED!\n");
  }
}

void handleStatus(const char *buffer)
{
  int32_t data[16];
  memcpy(data, &buffer[1], sizeof(data));

  printf("\n ------- ALEX STATUS REPORT ------- \n\n");
  printf("Left Forward Ticks:\t\t%d\n", data[0]);
  printf("Right Forward Ticks:\t\t%d\n", data[1]);
  printf("Left Reverse Ticks:\t\t%d\n", data[2]);
  printf("Right Reverse Ticks:\t\t%d\n", data[3]);
  printf("Left Forward Ticks Turns:\t%d\n", data[4]);
  printf("Right Forward Ticks Turns:\t%d\n", data[5]);
  printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
  printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
  printf("Forward Distance:\t\t%d\n", data[8]);
  printf("Reverse Distance:\t\t%d\n", data[9]);
  printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
  printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
  // We don't do anything because we issue commands
  // but we don't get them. Put this here
  // for future expansion
}

void handleNetwork(const char *buffer, int len)
{
  // The first byte is the packet type
  int type = buffer[0];

  switch(type)
  {
    case NET_ERROR_PACKET:
      handleError(buffer);
      break;

    case NET_STATUS_PACKET:
      handleStatus(buffer);
      break;

    case NET_MESSAGE_PACKET:
      handleMessage(buffer);
      break;

    case NET_COMMAND_PACKET:
      handleCommand(buffer);
      break;
  }
}

void sendData(void *conn, const char *buffer, int len) {
  int c;
  printf("\nSENDING %d BYTES OF DATA\n\n", len);
  if (networkActive) {
    c = sslWrite(conn, buffer, len);
    networkActive = (c > 0);
  }
}

void *readerThread(void *conn) {
  char buffer[128];
  int len;

  while (networkActive)	{
    len = sslRead(conn, buffer, 128);
    printf("read %d bytes from server.\n", len);
    networkActive = (len > 0);
    if (networkActive) handleNetwork(buffer, len);
  }
  printf("Exiting network listener thread\n");
  /* TODO: Stop the client loop and call EXIT_THREAD */
  EXIT_THREAD(conn);

  /* END TODO */
}

void *writerThread(void *conn) {
  int quit=0;
  while(!quit) {
    char ch;
    char buffer[10];
    int32_t params[2];

    buffer[0] = NET_COMMAND_PACKET;
    switch(ch)
    {
      case 'f':
      case 'F':
      case 'b':
      case 'B':
      case 'l':
      case 'L':
      case 'r':
      case 'R':
        getParams(params);
        buffer[1] = ch;
        memcpy(&buffer[2], params, sizeof(params));
        sendData(conn, buffer, sizeof(buffer));
        break;
      case 's':
      case 'S':
      case 'c':
      case 'C':
      case 'g':
      case 'G':
        params[0]=0;
        params[1]=0;
        memcpy(&buffer[2], params, sizeof(params));
        buffer[1] = ch;
        sendData(conn, buffer, sizeof(buffer));
        break;
      case 'q':
      case 'Q':
        quit=1;
        break;
      default:
        printf("BAD COMMAND\n");
    }
  }

  printf("Exiting keyboard thread\n");
  EXIT_THREAD(conn);
}

void connectToServer(const char *serverName, int portNum) {
  createClient(serverName, portNum, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT,
      1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);
}

int main(int ac, char **av) {
  if(ac != 3)  {
    fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
    exit(-1);
  }
  networkActive = 1;
  connectToServer(av[1], atoi(av[2]));
  while (networkActive); //keep running while the network is up
  printf("\nMAIN exiting\n\n");
}
