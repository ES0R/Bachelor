 /***************************************************************************
 * 
 * The main control functions are in control.cpp file
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

#include "control.h"
#include "ulaser.h"
#include "command.h"
#include "eeconfig.h"

ULaser::ULaser()
{
  rx[0] = '\0';
  rxCnt = 0;
}

void ULaser::setup()
{ // initialize
  Serial5.begin(19200, SERIAL_8N1);
  Serial5.print("C\n");
  Serial5.flush();
  timeHeightLast = 0;
  laserQuality = 9999;
}


void ULaser::sendStatus()
{
  const char MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "lash %d %.3f %.3f %d %.3f %.3f %.3f %d\n", laserOK, laserHeight, laserVelocity, laserQuality, getSampleTimeinMs()/1000.0, heightQLimit, heightOffset, laserValid);
  command->usb_send_str(s);
}

void ULaser::sendHelp()
{
  command->usb_send_str("# ----- Laser height sensor -----\r\n");
  command->usb_send_str( "#   lasl            Get latest string from sensor (as # comment)\r\n");
  command->usb_send_str( "#   laser ofs minQ  Set laser-height zero height, and quality limit (50 ... 2000) low is high q\r\n");
  command->usb_send_str( "#   lass            Get sensor status (returns old status and request a new)\r\n");
  command->usb_send_str( "#                   returns: 'lash OK height velocity quality Ts offset valid')\r\n");
  command->usb_send_str( "#   laso            Open laser (start laser height)\r\n");
  command->usb_send_str( "#   lasc            Close laser (saves power (<=1W))\r\n");
}

bool ULaser::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "lass ", 4) == 0)
  { // report laser-height height and height velocity
    Serial5.print("S\n");
    sendStatus();
  }
  else if (strncmp(buf, "laser ", 5) == 0)
  { // report laser-height height and height velocity
    const char * p1 = &buf[5];
    heightOffset = strtof(p1, (char **)&p1);
    heightQLimit = strtof(p1, (char **)&p1);
  }
  else if (strncmp(buf, "lasl ", 4) == 0)
  { // report the last received line from laser
    const char MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# laser: '%s'\n", rx2);
    command->usb_send_str(s);
  }
  else if (strncmp(buf, "laso ", 4) == 0)
  { // report laser-height height and height velocity
    Serial5.print("O\n");
  }
  else if (strncmp(buf, "lasc ", 4) == 0)
  { // report laser-height height and height velocity
    Serial5.print("C\n");
  }
  else
    used = false;
  return used;
}

void ULaser::readHeight()
{
  if (Serial5.available())
  {
    bool isOK = true;
    rx[rxCnt] = Serial5.read();
    if (rx[rxCnt] == '\r')
    { // a measurement is received
      // replane new-line with string terminator
      rx[rxCnt] = '\0';
      if (rxCnt > 3 and isalnum(rx[0]))
      { // get time since last measurement
        sampleTime = hbTimerCnt - timeHeightLast;
        timeHeightLast = hbTimerCnt;
        int stMs = getSampleTimeinMs();
        if (stMs < 2100 and stMs > 200)
        { // data should be valid (sample time between 0.3 to 2.0 sec - with a margin)
          // decode height
          char * p1 = &rx[0];
          float hm = strtof(p1, &p1);
          if (*p1++ != 'm')
            isOK = false;
          if (*p1++ != ',')
            isOK = false;
          int hmq = strtol(p1, nullptr, 10);
          if (isOK)
          { // height change in m
            float dh = hm - heightRawLast;
            heightRawLast = hm;
            laserHeight = heightRawLast - heightOffset;
            // height velocity in m/s
            laserVelocity = dh / float(stMs) * 1000;
            laserQuality = hmq;
          }
          // debug
          memcpy(rx2, rx, rxCnt + 1);
          // debug end
        }
      }
      else if (rx[0] == ',')
      { // status or OK
        if (rx[5] == 'C')
        { // status get voltage and temperature
          laserTemp = strtof(rx, nullptr);
          laserVoltage = strtof(&rx[7], nullptr);
        }
        if (rx[2] == 'K')
        { // an OK reply - 
          laserOK = true;
        }
      }
      else if (rx[0] == 'E')
      { // error - send to client
        const int MSL = 30;
        char s[MSL];
        snprintf(s, MSL, "# Laser error: '%s'\n", rx);
        command->usb_send_str(s);
      }
      laserValid = isOK;
      rxCnt = 0;
    }
    else if (rx[rxCnt] >= ' ')
    { // we got non-control character other than '\r'
        rxCnt++;
      if (rxCnt >= MRx)
      { // message overload
        rxCnt = 0;
      }
      rx[rxCnt] = '\0';
    }
  }
  else
  { // detect if no laser is available
    int32_t dt = hbTimerCnt - timeHeightLast;
    // laser is taken as valid if received data in the last 0.5 seconds
    laserValid = dt < int(0.5 / SAMPLETIME) and dt > 0;
    if (not laserValid)
    {
      laserVelocity = 0;
      laserHeight = 0;
      sampleTime = 0;
    }
  }
}

void ULaser::eePromLoad()
{
  heightOffset = eeConfig->readFloat();
    heightQLimit = eeConfig->readFloat();
}

void ULaser::eePromSave()
{
  eeConfig->pushFloat(heightOffset);
  eeConfig->pushFloat(heightQLimit);
}




