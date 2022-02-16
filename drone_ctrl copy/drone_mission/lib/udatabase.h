/***************************************************************************
 *   Copyright (C) 2021-2021 by DTU (Christian Andersen)                        *
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

#ifndef UDATABASE_H
#define UDATABASE_H

//#include <ubridge.h>
#include "urun.h"
#include "utime.h"
#include "upublish.h"

class UState;
class UPose;
class UAccGyro;
class UJoy;
class UBridge;
class UData;
class UOptiData;
class UTeensy;
class UServerPort;
class UCamera;
class ArUcoVals;
class URelay;
class UUnhandled;
class UGps;
class UMission;
class UGamepad;

/**
 * Database with access to all data elements */
class UDataBase : public URun
{
public:
  static const int MAX_DATA_ELEMENTS = 100;
  UData * data[MAX_DATA_ELEMENTS] = {nullptr};
  int dataCnt = 0;
  /// data elements
  UGamepad * gamepad = nullptr;
  UState * state = nullptr;
  UBridge * bridge = nullptr;
  UPose * pose = nullptr;
  UAccGyro * imu = nullptr;
  UJoy * joy = nullptr;
  UOptiData * opti = nullptr;
  UTeensy * teensy = nullptr;
  UServerPort * clients = nullptr;
  UCamera * camera = nullptr;
  ArUcoVals * arucos = nullptr;
  URelay * relay = nullptr;
  UUnhandled * unh = nullptr;
  UGps * gps = nullptr;
  UMission * mission;
  //
  // initialization variables
  static const int MIL = 500;
  char iniFileName[MIL];
  char bridgeIp[MIL];
  char servicePort[MIL];
  char teensyDevice[MIL];
  char gnssDevice[MIL];
  const char * argv0;
  bool asDaemon = false;
  //
  //
  enum UDataSource {Teensy = 1, Bridge = 2, Cam = 3, ArUco = 4};
  /**
   * constructor 
   * creates most data structures
   */
  UDataBase();
  /**
   * destructor */
  ~UDataBase();
  /**
   * set communication channels */
  void setup();
  /**
   * stop and close all */
  void stopAll();
  /**
   * request data to data modules - flag indicates the source */
  void requestData(bool toTeensy);
  /**
   * print status to console */
  void printStatus(int client);
  /**
   * send help */
  bool sendHelp(int client, const char * key);
  /**
   * decode an incoming message
   * \param message is the data element
   * \param source  is the source of the element - any reply will be send to this client
   * */
  bool decode(const char * message, int source, bool msgIsOK);
  /**
   * send reply to a client 
   * mostly reply to a data subscription 
   * The function will ask all possible client connections to find
   * the client with this clientID
   * \returns true if send */
  bool sendReply(const char * message, int toClient);
  /**
   * send message to datasource 
   * \param message is the line of text to send.
   * \param clientID is the destination ID.
   * \returns true if send */
  bool sendToDataSource(const char * message, int clientID);
  /**
   * called when a data source connection is just opened 
   * \param toTeensy is true if using serial port (USB), else to a bridge (false) */
  void dataSourceOpened(bool toTeensy);
  /**
   * open logfiles for selected interfaces */
  void openAllLogs();
  /**
   * close all logfiles */
  void closeAllLogs();
  /**
   * Run a periodic tic, e.g. for servicing published data elements */
  void run() override;
  /**
   * set key from this message
   * \param pm pointer to start of string with the key. The pointer is advanced to after key on return (if found).
   * \param key is where to return the key
   * \param keyLen is the length of the key buffer.
   * \returns true if a key is found and pointer 'pm' is advanced. */
  bool getKeyString(const char ** pm, char * key, int keyLen = UPublish::MAX_KEY_LENGTH);
  /**
   * utility to get string pointer to next parameter - (for ini-file)
   * \param p1 is a string starting at a label,
   * \returns pointer to first next parameter after some blank or '=' char(s) */
  static const char * skipToNext(const char * p1);
  /**
   * read command line 
   * \returns true if asked for command line help */
  bool readCommandLineParameters(int argc, char ** argv, bool * asDaemon);
  
  
private:
  /**
   * implement commands in ini-file 
   * \param ignoreLogfile ignore any command starting with 'log' */
  void readCommandsInIniFile(bool ignoreLogfile);
  /** last tick time */
  UTime tickTime;
  /** tick interval in seconds */
  float tickInterval = 0.01;
};

#endif
