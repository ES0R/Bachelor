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
#include "urelay.h"


URelay::URelay(UDataBase * data)
: UData(data)
{
  // add published keywords
  addKey("#");
}

bool URelay::decode(const char * msg, int source, const char * key, bool isCheckedOK)
{ //   snprintf(s, MSL, "pos %5.3f %5.3f %5.3f %.2f %.3f\n", 
  // imu->roll, imu->pitch, imu->yaw, height, heightVelocity);
  bool used = true;
  if (msg[0] == '#' or msg[0] == '%' or isdigit(msg[0]))
  {
//     if (logfile != NULL)
//     {
//       UTime t;
//       t.now();
//       fprintf(logfile, "%ld.%03ld %s", t.getSec(), t.getMilisec(), msg);
//     }
    updated(msg, source, key);
    if (rawCnt > 0)
      raw[0]->saveToLog(logfile);
    // debug
    printf("# URelay::%s", msg);
  }
  else
    used = false;
  return used;
}

void URelay::subscribeFromHW(bool fromTeensy)
{
  if (not fromTeensy)
  { // request pose data from bridge
    db->sendToDataSource("# subscribe 1\n", UDataBase::Bridge); // Pose
  }
}

void URelay::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  db->sendReply("# ------- # (log and comments) ----------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# received %d msgs\n", updateCnt);
  db->sendReply(s, client);
  printDataStatus(client);
}

void URelay::openLog()
{ // open pose log
  UData::createLogfile("log_hash");
  if (logfile != NULL)
  {
    fprintf(logfile, "%% robobot mission pose logfile\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 message\n");
  }
}

void URelay::sendHelp(const int client)
{
  db->sendReply("# ------- # and bulk ----------\r\n", client);
  db->sendReply("# keys: '#', '%' and digit as first character\r\n", client);
}
