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



#include <sys/time.h>
#include <cstdlib>

#include "umission.h"
#include "utime.h"


UMission::UMission(UDataBase * data)
: UData(data)
{
  db = data;
  addKey("mission");
//   bridge = db->bridge;
}


UMission::~UMission()
{
  printf("Mission class destructor\n");
}


void UMission::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  db->sendReply("# ------- Mission ----------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# active = %d, finished = %d\n", active, finished);
  db->sendReply(s, client);
  snprintf(s, MSL, "# mission part=%d, in state=%d\n", mission, missionState);
  db->sendReply(s, client);
  printDataStatus(client);
}

////////////////////////////////////////////////////////////

void UMission::openLog()
{
  UData::createLogfile("log_mission");
  UTime t;
  t.now();
  if (logfile != NULL)
  {
    const int MSL = 50;
    char s[MSL];
    fprintf(logMission, "%% Mission log started at %s\n", t.getDateTimeAsString(s));
    fprintf(logMission, "%% 1  Time [sec]\n");
    fprintf(logMission, "%% 2  mission number.\n");
    fprintf(logMission, "%% 3  mission state.\n");
  }
  else
    printf("#UMission:: Failed to open image logfile\n");
}

void UMission::tick()
{
  bool newState = true;
  switch (missionState)
  {
    case 0:
      missionState++;
      break;
    case 1:
      missionState++;
      break;
    case 2:
      missionState++;
      break;
    default:
      newState = false;
      break;
  }
  if (newState)
    saveDataToLog();
}


void UMission::saveDataToLog()
{
  if (logfile != NULL)
  {
    timeval t;
    gettimeofday(&t, NULL);
    fprintf(logfile, "%ld.%03ld %d %d\n", t.tv_sec, t.tv_usec / 1000, 
            mission,
            missionState
    );
    fflush(logfile);
  }
}

