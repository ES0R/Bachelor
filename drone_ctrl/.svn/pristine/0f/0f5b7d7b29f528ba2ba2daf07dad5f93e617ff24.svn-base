/***************************************************************************
*   Copyright (C) 2016 by DTU (Christian Andersen)                        *
*   jca@elektro.dtu.dk                                                    *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Lesser General Public License as        *
*   published by the Free Software Foundation; either version 2 of the    *
*   License, or (at your option) any later version.                       *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU Lesser General Public License for more details.                   *
*                                                                         *
*   You should have received a copy of the GNU Lesser General Public      *
*   License along with this program; if not, write to the                 *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#ifndef UREGBOT_H
#define UREGBOT_H

#include <sys/time.h>
#include <mutex>
#include <fstream>

//#include "urun.h"
#include "utime.h"
//#include "ulogfile.h"
//#include <tcpCase.h>
#include <udata.h>

// class UHandler;


using namespace std;

/**
 * The robot class handles the 
 * port to the REGBOT part of the robot,
 * REGBOT handles the most real time issues
 * and this class is the interface to that. */
class UTeensy : public UData
{ // REGBOT interface
public:
  /// Is port opened successfully
  bool connectionOpen;
  
private:
  // serial port handle
  int usbport;
//   std::fstream usbio;
  // serial port (USB)
  const char * usbdevice = "/dev/ttyACM0";
  // simulator hostname
// //   const char * simHost;
//   // simulator port
//   int simPort = 0;
  // mutex to ensure commands to regbot is not mixed
  mutex sendMtx;
//   mutex logMtx;
  mutex eventUpdate;
  // receive buffer
  static const int MAX_RX_CNT = 200;
  char rx[MAX_RX_CNT + 1];
  // number of characters in rx buffer
  int rxCnt;
  int rxChars = 0; /// total number of received chars
  // command and data handler
//   UHandler * handler;
  //
  UTime lastTxTime;
  int messageInCnt = 0;
  int messageErrCnt = 0;
  int messageOutCnt = 0;
  // socket to simulator
//   tcpCase socket;
  
public:
  /** constructor */
  UTeensy(UDataBase * database, int id, const char device[]);
  /** destructor */
  ~UTeensy();
  /**
   * Set device */
  void setDevice(char * usbDev, int simport, char * simhost)
  {
    usbdevice = usbDev;
//     simHost = simhost;
//     simPort = simport;
  }
  /**
   * send a string to the serial port 
   * But wait no longer than timeout - the un-send part is skipped 
   * \param key is message ID,
   * \param params is message parameters
   * \param timeoutMs is number of ms to wait at maximum */
  void send(const char * key, const char * params, int timeoutMs = 10);
  /**
   * send a string to the serial port 
   * But wait no longer than timeout - the unsend part is skipped 
   * \param cmd is c_string to send,
   * \param timeoutMs is number of ms to wait at maximum */
  void send(const char * cmd, int timeoutMs = 10);
  /**
   * runs the receive thread 
   * This run() function is called in a thread after a start() call.
   * This function will not return until the thread is stopped. */
  void run();
  /**
   * open log with communication with robot */
  void openLog() override;
  /**
   * Print status of interface */
  void printStatus(int client) override;
  /**
   * send help */
  void sendHelp(const int client) override;
  /**
   * decode message - potentialy - to Teensy */
  bool decode(const char * message, int source, const char * key, bool isCheckedOK) override;
  
  void tick() override;
  
private:
  /**
   * Open the connection.
   * \returns true if successful */
//   bool openToRegbot();
  bool openUsb();
  /**
   * save traffic data to log */
  void saveDataToLog(const char * message, bool incoming);
  /**
   * decode messages from REGBOT */
  void decodeMsgFromTeensy(char * msg);
  /**
   * Update connection state */
  void updateConnectionState();
  /**
   * calculate idle time in connection to Teensy 
   * mostly debug */
  int connectErrCnt = 0;
  int readIdleLoops = 0;
  int again = 0;
//   UTime idleTimer;
  UTime idlePeriod;
  UTime aliveTimer;
//   float idleTime = 0.0;
  float idlePct;
};

#endif
