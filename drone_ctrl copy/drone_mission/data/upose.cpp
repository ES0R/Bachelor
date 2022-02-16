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

#include <math.h>
#include <udatabase.h>
#include <cstring>
#include "upose.h"


UPose::UPose(UDataBase * data)
: UData(data)
{
  // openLog();
  // add published keywords
  addKey("pos");
}

bool UPose::decode(const char * msg, int source, const char * key, bool isCheckedOK)
{ //   snprintf(s, MSL, "pos %5.3f %5.3f %5.3f %.2f %.3f\n", 
  // imu->roll, imu->pitch, imu->yaw, height, heightVelocity);
  bool used = true;
  if (strcmp(key, "pos") == 0)
  {
    const char * p1 = &msg[3];
    const char * p2;
    roll = strtof(p1, (char**)&p1);
    pitch = strtof(p1, (char**)&p1);
    yaw = strtof(p1, (char**)&p1);
    height = strtof(p1, (char**)&p1);
    heightVel = strtof(p1, (char**)&p2);
    bool isOK = p1 != p2;
    if (not isCheckedOK)
    {
      printf("# failed     q-check (%s)\n", msg);
    }
    if (not isOK)
    {
      printf("# failed pose decode (%s)\n", msg);
    }
//     else
//     {
//       printf("# ok     pose decode (%s)\n", msg);
//     }
    if (logfile != NULL and isOK)
    {
      UTime t;
      t.now();
      fprintf(logfile, "%ld.%03ld %.5f %.5f %.5f %.2f %.2f\n", t.getSec(), t.getMilisec(), roll, pitch, yaw, height, heightVel);
    }
    if (isOK)
      updated(msg, source, key);
  }
  else
    used = false;
  return used;
}

void UPose::subscribeFromHW(bool fromTeensy)
{
  if (fromTeensy)
  { // request pose streaming
    db->sendToDataSource("robot sub pose 20\n", UDataBase::Teensy); // Pose
  }
  else
  { // request pose data from bridge
    db->sendToDataSource("pos subscribe 1\n", UDataBase::Bridge); // Pose
  }
}

void UPose::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  db->sendReply("# ------- Robot pose ----------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# Roll %f [deg], pitch %f, yaw %f\n", 
         roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
  db->sendReply(s, client);
  snprintf(s, MSL, "# height %fm, height vel %f m/s\n", 
         height, heightVel);
  db->sendReply(s, client);
  //printf("# data age %.3fs\n", getTimeSinceUpdate());
  printDataStatus(client);
}

void UPose::openLog()
{ // open pose log
  UData::createLogfile("log_odometry");
  if (logfile != NULL)
  {
    fprintf(logfile, "%% robobot mission pose logfile\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 roll [radians]\n");
    fprintf(logfile, "%% 3 pitch\n");
    fprintf(logfile, "%% 4 Yaw\n");
    fprintf(logfile, "%% 5 height [m]\n");
    fprintf(logfile, "%% 6 height velocity\n");
  }
}

void UPose::sendHelp(const int client)
{
  db->sendReply("# ------- Pose ----------\r\n", client);
  sendKeyList(client);
  
  // db->sendReply("# keywords: pos     Holds: roll, pitch, yaw and height\r\n", client);
}


