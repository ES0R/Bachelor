
/***************************************************************************
 *   Copyright (C) 2019-2021 by DTU (Christian Andersen)                        *
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


#ifndef USTATE_H_INCLUDED
#define USTATE_H_INCLUDED

// #include <ubridge.h>
#include <udata.h>


class UState : public UData
{
public:
  // time
  float regbotTime = 0.0;
  timeval lastHbtTime;
  timeval lastFakeHbtTime;
  bool lastHbtOnTime = true;
  timeval bootTime;
  // statics
  int robotId = 0;
  int robotFWversion = 0;  // Firmware version (from SVN)
  int robotSWversion = 0;  // hardware configuration 1..6
  static const int MAX_NAME_LENGTH = 32;
  char robotname[MAX_NAME_LENGTH] = {'\0'};  
  // state
  enum ArmState {Init, Disarmed, Armed, Fail};
  ArmState armState = Init;
  enum FlightState {OnGround, Starting, InFlight, Landing};
  FlightState flightState = OnGround;
  float batteryVoltage = 0.0;
  
  // methods
  /** 
   * constructor */
  UState(UDataBase * data);
  
  float getTime();
  
  void subscribeFromHW(bool fromTeensy) override;
  bool isHeartbeatOK();  
  /**
   * Print status for bridge and all data elements */
  void printStatus(int client) override;
  /**
   * Open logfile */
  void openLog();
  /**
   * save status to log - if log is open */
  void saveDataToLog();  
  /**
   * decode an incomming message */
  bool decode(const char * message, int source, const char * key, bool isCheckedOK) override;
  /**
   * check if hbt is too slow for clients. */
  void tick() override;
  
  void sendHelp(const int client) override;
  
private:
  void decodeId(const char * msg);
  void decodeHbt(const char * msg);
//   void decodeMission(const char * msg);
  float measureCPUtemp();
};

#endif
