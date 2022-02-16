
/***************************************************************************
 *   Copyright (C) 2019-2020 by DTU (Christian Andersen)                        *
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

#ifndef UOPTIDATA_H_INCLUDED
#define UOPTIDATA_H_INCLUDED


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

#include <uoptitrack.h>
#include "udata.h"
#include "udatabase.h"

/**
 * Class for one rigid body detected by OPtitrack system 
 * The base orientation is in quatonions */
class URigid : public UPublish
{
public:
  URigid(ULogFile * log, UData * data)
  : UPublish(data, "rigid")
  {
    lf = log;
    parent = data;
  }
  /// optitrack ID - no optitrack name is available
  int ID;
  char name[UOptitrack::MAX_RIGID_NAME_SIZE];
  float pos[3];
  float rotq[4];
  float euler[3];
  bool eulerFrame = 0;
  float posErr;
  /// is tracked by optitrack
  bool valid = false;
  /// update time - when updated on this computer
//   UTime t;
  /// is set true on update
//   bool fresh = false;
  /// logfile pointer
  ULogFile * lf;
  UData * parent;
  /**
   * print values to console */
  void printRigid(int client);
  /**
   * Convert to euler angled  
   * from wikipedia https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles */
  void toEuler();
  /**
   * save update to log - if open */
  void saveToLog();
  /**
   * send this data to a client */
  bool sendToClient(int client, double frametime) override;
  /**
   * set to empty data */
  void clear();

public:
  /**
   * set true during unpack, but frametime etc is missing */
  bool fresh; 
  double frametime;
  int frameCnt;
  int totalRigidCnt = 0;
};

// //////////////////////////////////////////////////
// //////////////////////////////////////////////////
// //////////////////////////////////////////////////

/**
 * Class for data access and logging for the optitrack interface 
 */
class UOptiData : public UData, private UOptitrack
{
public:
  UOptiData(UDataBase * database, int id);
  ~UOptiData();
  
protected:
  
  void printStatus(int client) override;
  /**
   * searches the rigid ISs received so far,
   * \param id is the id to look for.
   * \returns a pointer to the mrigid body data, if found, else a nullptr.
   * */
  URigid * findMarker(int id) override;
  /**
   * Test if data is fresh for a rigid body ID
   * \param id is the ID of the marker.
   * \returns pointer to the rigid by, if it is fresh, else a nullptr (0 or false)
   * The user must reset the fresh-flag after use. 
   * */
//   URigid * isUpdated(int id);
  /**
   * send on-line help */
  void sendHelp(const int client) override;
  /**
   * subscribe to optitrack data element */
  bool subscribeFromThis(const char * message, const char * key, int client) override;
  
  void tick() override;
  
  void updateOptiData();
  
  void openLog() override;
  
  void frameUpdate(double time, int rigidCnt) override;
  
//   bool logOpenClose(const char * params, const char * key) override;
  
public:
  /**
   * set start parameters for Optitrack connection */
  void setup(const char * argv0);
  
private:
  // if no real markers exist
  URigid * dummyMarker;
  bool openAllLogfiles = false;
  // frame data
  double frameTime = 0;
  int frameCnt = 0;
  int   rigidCount = 0;
};


#endif
