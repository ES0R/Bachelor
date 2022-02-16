/***************************************************************************
*   Copyright (C) 2016-2020 by DTU (Christian Andersen)                        *
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

#ifndef UBRIDGE_H
#define UBRIDGE_H

#include <sys/time.h>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include "tcpCase.h"
#include "udata.h"

class UDataBase;

/**
 * The bridge class handles the 
 * port to the hardware (Teensy) through a bridge,
 *  */
class UBridge : public UData, public tcpCase
{ // socket interface to source data and actuators
private:
  // mutex to ensure commands to regbot are not mixed
  std::mutex sendMtx;
//   std::mutex logMtx;
  // data decode link
  UDataBase * db;
  // load
  float bridgeLoad = 0;
  int msgCnt1sec = 0;
  // host name/IF and port number as 'host:port'
  static const int MAX_NAME_LENGTH = 1000;
  static const int MAX_HOST_LENGTH = 1100;
  char host[MAX_HOST_LENGTH];
  // communication messages count
  int msgInCnt = 0;
  int msgOutCnt = 0;
  
public:
  /** constructor
   * \param server is a string with either IP address or hostname.
   * connects to port 24001
   */
  UBridge(UDataBase * database, const char * server, int id);
  /** destructor */
  ~UBridge();
  /**
   * send a string to the serial port */
  bool send(const char * cmd);
  /**
   * send on-line help text to this client */
  void sendHelp(int client) override;
  /**
   * receive thread */
  void run();
  /**
   * Print status for bridge and all data elements */
  void printStatus(int client) override;
  /**
   * Open logfile */
  void openLog() override;
  /**
   * save traffic to log - if log is open */
  void saveDataToLog(const char * message, bool incoming);  
  /**
   * decode message that may be outgoing specifically for this interface */
  bool decode(const char * msg, int sender, const char * key, bool isCheckedOK) override;
  
private:
  /**
   * decode messages from external connection */
  void decode(char * msg);
  /**
   * open a connection to a bridge 
   * \param is a string with IP or hostname, and possibly ':' and port number
   * */
  void connectToBridge(const char * host);
  /**
   * time (for debug print) */
  UTime t;
};

#endif
