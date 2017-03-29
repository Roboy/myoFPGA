#pragma once
// std
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string>
#include <ifaddrs.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>

#define MAXBUFLENGTH 1024

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

using namespace std;

bool convertByte2Text(uint32_t inet, char *inet_str);
bool convertText2Byte(char *inet_str, uint32_t &inet);

typedef struct
{
    uint8_t control_mode[16];
    int32_t outputPosMax[16]; /*!< maximum control output in the positive direction in counts, max 4000*/
    int32_t outputNegMax[16]; /*!< maximum control output in the negative direction in counts, max -4000*/
    int32_t spPosMax[16];/*<!Positive limit for the set point.*/
    int32_t spNegMax[16];/*<!Negative limit for the set point.*/
    uint16_t Kp[16];/*!<Gain of the proportional component*/
    uint16_t Ki[16];/*!<Gain of the integral component*/
    uint16_t Kd[16];/*!<Gain of the differential component*/
    uint16_t forwardGain[16]; /*!<Gain of  the feed-forward term*/
    uint16_t deadBand[16];/*!<Optional deadband threshold for the control response*/
    int16_t IntegralPosMax[16]; /*!<Integral positive component maximum*/
    int16_t IntegralNegMax[16]; /*!<Integral negative component maximum*/
    float radPerEncoderCount[16] = {2 * 3.14159265359f / (2000.0f * 53.0f)};
}control_Parameters_t;

enum CONTROL{
    POSITION,
    VELOCITY,
    DISPLACEMENT
};

class UDPSocket{
public:
    /**
     * Creates a socket on the given server_IP and server_port and sets up the "connection" with the client.
     * Because it is UDP, there is no handshake, the socket just sends and listens to packages from the client_IP
     * and client_port
     * @param server_IP the server socket IP
     * @param server_port the server socket port
     * @param client_IP the client to send and receive UDP packets to/from
     * @param client_port the client port
     * @param exclusive receive exclusively packages from client
     */
    UDPSocket(const char* server_IP, int server_port, const char* client_IP, int client_port, bool exclusive=true);
    /**
     * Tries to guess the users IP and sets up the socket on Port 8000 and "connects" to client_IP on client_port
     * @param client_IP the client to send and receive UDP packets to/from
     * @param client_port the client port
     * @param exclusive receive exclusively packages from client
     */
    UDPSocket(const char* client_IP, int client_port, bool exclusive=true);


    UDPSocket(const char* client_IP, int client_port, int server_port, bool exclusive=true);
    /**
     * Creates a broadcast socket on port
     * @param port
     */
    UDPSocket(int port, bool broadcaster);
    ~UDPSocket();

    /**
     * Receive motor config values
     * @param config the controller parameters
     * @return success
     */
    bool receiveMotorConfig(control_Parameters_t &config);

    bool sendMotorConfig(control_Parameters_t *config);

    pair<uint32_t,string> myIP;
private:
    /**
     * Sets the UDP package receive and send timeout
     * @param usecs time in microseconds
     * @return success
     */
    bool setTimeOut(int usecs);

    /**
     * Tries to guess your IP
     * @param ip your IP
     * @param success (fails if I can't find a valid IP for wifi or ethernet adapter)
     */
    bool whatsMyIP(string &IP);

    bool initialized = false;

    /**
    * receive from anyone
    * @return success
    */
    bool receiveUDP();
    /**
    * receive from client
    * @return success
    */
    bool receiveUDPFromClient();
    /**
     * send to client
     * @return success
     */
    bool sendUDPToClient();
    /**
     * broadcast
     * @return success
     */
    bool broadcastUDP();
private:
    int sockfd; //* socket
    struct sockaddr_in server_addr; /* server's addr */
    struct sockaddr_in broadcast_addr; /* server's addr */
    struct sockaddr_in client_addr; /* client addr */
    socklen_t client_addr_len, server_addr_len, broadcast_addr_len; /* byte size of addresses */
    ssize_t numbytes; /* message byte size */
    struct addrinfo *servinfo;
    char buf[MAXBUFLENGTH];
    bool exclusive;
    int timeout = 10000;
};