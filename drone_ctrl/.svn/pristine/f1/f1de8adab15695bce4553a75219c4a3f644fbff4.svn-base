 /***************************************************************************
 *
 *   sending of subscribed data
 * 
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "usubscribe.h"
#include "upropshield.h"
#include "uheight.h"
#include "ustate.h"

void USubscribe::tick(int mainLoop, bool gettingLog)
{
  if (true) // not gettingLog)
  {
    if (sendPose > 0)
    {
      if (mainLoop % sendPose == sendPose - 1)
      {
        hgt->sendPose();
      }
    }
    if (sendHb > 0)
    {
      if (mainLoop % sendHb == sendHb - 1)
      {
        state->sendState();
      }
    }
    switch (sendImu)
    {
      case 0: break;
      case 1:
        imu->sendAccSI();
        imu->sendGyroSI();
        imu->sendMagSI();
        break;
      case 2:
        if (mainLoop % sendImu == 0)
        {
          imu->sendGyroSI();
          imu->sendAccSI();
        }
        else if (mainLoop % sendImu == 1)
          imu->sendMagSI();
        break;
      default:
        if (mainLoop % sendImu == 0)
          imu->sendGyroSI();
        else if (mainLoop % sendImu == 1)
          imu->sendAccSI();
        else if (mainLoop % sendImu == 2)
          imu->sendMagSI();
        break;
    }
  }
}

void USubscribe::sendHelp()
{
  const int MRL = 250;
  char reply[MRL];
  command->usb_send_str("# ----- data subscribe -----\r\n");
  snprintf(reply, MRL, "#   sub xxx N    Send/stream xxx data every N, xxx=[pose, state]; N=0 stop, N=1,2,.. = every N sample.\r\n");
  command->usb_send_str(reply);
}


bool USubscribe::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "sub ", 3) == 0)
  { // decode subscription commands
    // pt only pose and state are supported
    const int MSL = 100;
    char s[MSL];
    const char * p1 = &buf[3];
    const char * p2;
    int n;
    while (isspace(*p1)) p1++;
    p2 = strchr(p1, ' ');
    if (p2 != NULL)
      n = strtol(p2, NULL, 10);
    else
      n = 1;
    if (strncmp(p1, "pose", 4) == 0)
    {
      sendPose = n;
      // debug
      snprintf(s, MSL, "# subscribe to pose interval = %d, from %s (p2=%s)\n", n, buf, p2);
      command->usb_send_str(s);
      // debug end
    }
    else if (strncmp(p1, "state", 5) == 0)
    {
      sendHb = n;
      // debug
      snprintf(s, MSL, "# subscribe to hartbeat interval = %d, from %s (p2=%s)\n", n, buf, p2);
      command->usb_send_str(s);
      // debug end
    }
    else if (strncmp(p1, "imu", 3) == 0)
    {
      sendImu = n;
      // debug
      snprintf(s, MSL, "# subscribe to IMU data (acc, gyro and magnetometer) interval = %d, from %s (p2=%s)\n", n, buf, p2);
      command->usb_send_str(s);
      // debug end
    }
    else
    {
      snprintf(s, MSL, "# subscribe subject '%s' unknown, n=%d\n", p1, n);
      command->usb_send_str(s);
    }
  }
  else
    used = false;
  return used;
}
