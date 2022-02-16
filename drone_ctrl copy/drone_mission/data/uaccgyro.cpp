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
#include <udata.h>
#include <string.h>
#include "uaccgyro.h"
#include <udatabase.h>


UAccGyro::UAccGyro(UDataBase * datasource)
: UData(datasource)
{
//   openLog();
  // add keys the decode function will handle
  // or is created locally (published keywords)
  addKey("acc");
  addKey("gyro");
  addKey("mag");
}

bool UAccGyro::decode(const char* msg, int source, const char * key, bool isCheckedOK)
{ 
  bool used = true;
  const char * p1, *p2;
  if (strcmp(key, "acc") == 0)
  {
    p1 = &msg[3];
    acc[0] = strtof(p1, (char**)&p1);
    acc[1] = strtof(p1, (char**)&p1);
    acc[2] = strtof(p1, (char**)&p1);
    accW[0] = strtof(p1, (char**)&p1);
    accW[1] = strtof(p1, (char**)&p1);
    accW[2] = strtof(p1, (char**)&p2);
  }
  else if (strcmp(key, "gyro") == 0)
  {
    p1 = &msg[4];
    gyro[0] = strtof(p1, (char**)&p1);
    gyro[1] = strtof(p1, (char**)&p1);
    gyro[2] = strtof(p1, (char**)&p2);
  }
  else if (strcmp(key, "mag") == 0)
  {
    p1 = &msg[3];
    mag[0] = strtof(p1, (char**)&p1);
    mag[1] = strtof(p1, (char**)&p1);
    mag[2] = strtof(p1, (char**)&p2);
  }
  else 
    used = false;
  if (used)
  { // some communication 
    bool isOK = p1 != p2 and isCheckedOK;
    if (isOK)
    {
      updated(msg, source, key);
      if (logfile != NULL)
      {
        fprintf(logfile, "%ld.%03ld %d %.3f %.3f %.3f %.3f %.3f %.3f %g %g %g\n", dataTime.tv_sec, dataTime.tv_usec / 1000, dataID, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
      }
    }
  }
  return used;
}

void UAccGyro::subscribeFromHW(bool fromTeensy)
{ 
  if (fromTeensy)
  { // ask regbot to generate acc messages
    // if rate < 4 (< 10ms) then mag will not be send
    db->sendToDataSource("robot sub imu 8\n", UDataBase::Teensy);
  }
  else
  {  // ask bridge to forward all acc messages
    db->sendToDataSource("acc subscribe 2\n", UDataBase::Bridge); 
    // ask bridge to forward all gyro messages
    db->sendToDataSource("gyro subscribe 2\n", UDataBase::Bridge);
    // ask bridge to forward all magnetometer messages
    db->sendToDataSource("mag subscribe 2\n", UDataBase::Bridge);
  }
}

void UAccGyro::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  db->sendReply("# ------- Accelerometer and gyro ----------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# accelerometer (x, y, z) = (%6.3f, %6.3f, %6.3f) m/s^2\n", acc[0], acc[1], acc[2]);
  db->sendReply(s, client);
  snprintf(s, MSL, "# gyro          (x, y, z) = (%6.3f, %6.3f, %6.3f) deg/s\n", gyro[0], gyro[1], gyro[2]);
  db->sendReply(s, client);
  snprintf(s, MSL, "# magnetometer  (x, y, z) = (%6.3f, %6.3f, %6.3f) uT?\n", mag[0], mag[1], mag[2]);
  db->sendReply(s, client);
  snprintf(s, MSL, "# data age %.3fs\n", getTimeSinceUpdate());
  db->sendReply(s, client);
  printDataStatus(client);
}

float UAccGyro::turnrate()
{
  return sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
}

void UAccGyro::openLog()
{ // open pose log
  if (not logIsOpen())
  {
    UData::createLogfile("log_accgyro");
    if (logfile != NULL)
    {
      fprintf(logfile, "%% robobot mission Accelerometer, gyro and magnetometer log\n");
      fprintf(logfile, "%% 1 Timestamp in seconds\n");
      fprintf(logfile, "%% 2-4 accelerometer x,y,z [m/s^2]\n");
      fprintf(logfile, "%% 5-7 gyro x,y,z [deg/s]\n");
      fprintf(logfile, "%% 8-10 magnetometer x,y,z [uT?]\n");
    }
  }
}

void UAccGyro::sendHelp(int client)
{
  db->sendReply("# ------- IMU ----------\r\n", client);
  sendKeyList(client);
  // db->sendReply("# keywords: acc, gyro, mag     Holds: accelerometer [m/s^2], gyro[deg/s] and magnetometer [uT]\r\n", client);
}
