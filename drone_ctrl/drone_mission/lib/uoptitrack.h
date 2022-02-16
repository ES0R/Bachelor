/**
 * Builds on top of NatNet example, as per the statement below 
 * Modified by DTU jca@elektro.dtu.dk 2021 for use in ASTA Optitrack system
 * 
 * Receive data from Optitrack Motive (v1.5-2.1)
 * by directly reading NatNet (2.5-3.0) UDP stream;
 * works under Ubuntu 14.04 and 16.04 with Python 3 or C++11.
 * This is a direct translation of the official Packet Client example from
 * [NatNet SDK](http://optitrack.com/downloads/developer-tools.html#natnet-sdk),
 * that originally works only on Windows.
 * 
 */


//=============================================================================
// Copyright � 2014 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================

#ifndef UOPTITRACK_H_INCLUDED
#define UOPTITRACK_H_INCLUDED


#include <cinttypes>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <climits>

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include "urun.h"
#include "ulogfile.h"

#define MAX_PACKETSIZE              100000    // actual packet size is dynamic
#define MAX_NAMELENGTH              256

// bool setup(int argc, char *argv[]);
void DecodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID);
bool TimecodeStringify(unsigned int inTimecode,
                       unsigned int inTimecodeSubframe,
                       char *Buffer,
                       size_t BufferSize);



class URigid;
class ULogFile;

class UOptitrack : private URun
{
public:
  UOptitrack();
  ~UOptitrack();
  
public:
  //   void printStatus() override;
  static const int MAX_RIGID_NAMES = 40;
  static const int MAX_RIGID_NAME_SIZE = 32;
  char rigidNames[MAX_RIGID_NAMES][MAX_RIGID_NAME_SIZE] = {'\0'};
  
protected:
  
  #define NAT_CONNECT                 0
  #define NAT_SERVERINFO              1
  #define NAT_REQUEST                 2
  #define NAT_RESPONSE                3
  #define NAT_REQUEST_MODELDEF        4
  #define NAT_MODELDEF                5
  #define NAT_REQUEST_FRAMEOFDATA     6
  #define NAT_FRAMEOFDATA             7
  #define NAT_MESSAGESTRING           8
  #define NAT_UNRECOGNIZED_REQUEST    100
  
  #define MULTICAST_ADDRESS       "239.255.42.99"
  #define PORT_COMMAND            1510      // NatNet Command channel
  #define PORT_DATA               1511      // NatNet Data channel
  
  // Sockets
  in_addr ServerAddress;
  sockaddr_in HostAddr;
  
  typedef struct {
    char szName[MAX_NAMELENGTH];            // sending app's name
    uint8_t Version[4];                     // [major.minor.build.revision]
    uint8_t NatNetVersion[4];               // [major.minor.build.revision]
  } sSender;
  
  typedef struct sSender_Server {
    sSender Common;
    // host's high resolution clock frequency (ticks per second)
    uint64_t HighResClockFrequency;
    uint16_t DataPort;
    bool IsMulticast;
    uint8_t MulticastGroupAddress[4];
  } sSender_Server;
  
  typedef struct {
    uint16_t iMessage;                      // message ID (e.g. NAT_FRAMEOFDATA)
    uint16_t nDataBytes;                    // Num bytes in payload
    union {
      uint8_t cData[MAX_PACKETSIZE];
      char szData[MAX_PACKETSIZE];
      uint32_t lData[MAX_PACKETSIZE / sizeof(uint32_t)];
      float fData[MAX_PACKETSIZE / sizeof(float)];
      sSender Sender;
      sSender_Server SenderServer;
    } Data;                                 // Payload incoming from NatNet Server
  } sPacket;
  
  bool connected = false;
  
  int MessageID = 0;
  int nBytes = 0;
  int frameNumber = 0;
  int nMarkerSets = 0;
  // positions
  static const int MAX_MARKERS = 30;
  URigid * markers[MAX_MARKERS] = {nullptr};
  int markersCnt = 0;
  // common timestamp
  unsigned int timecode = 0;   // not used
  unsigned int timecodeSub = 0;// not used
  static const int MTL = 128;
  char szTimecode[MTL] = "";
  double timestamp;
  // more timing
  uint64_t cameraMidExposureTimestamp = 0; // not used
  uint64_t cameraDataReceivedTimestamp = 0; // not used
  uint64_t transmitTimestamp = 0; // from server

  // Command mode global variables
  int gCommandResponse = 0;
  int gCommandResponseSize = 0;
  unsigned char gCommandResponseString[PATH_MAX];
  
  
  virtual URigid * findMarker(int id)
  { return nullptr;};
  virtual void frameUpdate(double time, int rigidCnt)
  { };
  /**
   * setup of NatNet connection */
  bool natNetSetup(int argc, char * argv[]); 
  /**
   * NatNet unpack function */
  void DecodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID);
  bool IPAddress_StringToAddr(char *szNameOrAddress,
                              struct in_addr *Address);
  int CreateCommandSocket(in_addr_t IP_Address, unsigned short uPort);
//   void * CommandListenThread(void *dummy);
  void run() override;
  // logfile for connection setup and errors
  ULogFile setupLog;
public:
//  void *DataListenThread(void *dummy);
  void dataListen();
  
  
public:
  /**
   * New dataframe received, to handle 
   * \param pData is pointer to binary NatNet data to unpack
   * */
  void unpack(char * pData);
  /**
   * set start parameters */
  void setup(const char * argv0);
private:
  
  /**
   * copy paste function from 
   * https://stackoverflow.com/questions/212528/get-the-ip-address-of-the-machine
   * to find IP string for this machine - returns the LAST IP, assuming it is for the wifi
   * \param ipString is string to return result,
   * \param ipStringLen is maximum length of the provided string */
  void findIP(char * ipString, int ipStringLen);
  
};

#endif
