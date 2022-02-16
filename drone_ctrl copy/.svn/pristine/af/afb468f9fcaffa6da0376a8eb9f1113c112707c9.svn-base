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
#include "ultrasound.h"
#include "command.h"
#include "eeconfig.h"

UUltrasound::UUltrasound()
{
  rx[0] = '\0';
  rxCnt = 0;
}

void UUltrasound::setup()
{ // initialize
  Serial4.begin(9600, SERIAL_8N1_RXINV);
  Serial4.flush();
  timeHeight = 0;
}


void UUltrasound::sendStatus()
{
  const char MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "uhgt %.3f %.3f %.3f %.3f %.3f %d\n", sonarHeight, sonarVelocity, getSampleTimeinMs()/1000.0, heightLimit, heightOffset, sonarValid);
  command->usb_send_str(s);
}

void UUltrasound::sendHelp()
{
  command->usb_send_str("# ----- Ultrasound height -----\r\n");
  command->usb_send_str( "#   uht          Get ultrasound height [m] (height velocity [m/s] and sample time [s])\r\n");
  command->usb_send_str( "#   sonar max ofs   Set ultrasound max height to use and zero height offset\r\n");
}

bool UUltrasound::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "uht ", 3) == 0)
  { // report ultrasound height and height velocity
    sendStatus();
  }
  else if (strncmp(buf, "sonar ", 5) == 0)
  { // report ultrasound height and height velocity
    const char * p1 = &buf[5];
    heightLimit = strtof(p1, (char **)&p1);
    heightOffset = strtof(p1, (char **)&p1);
  }
  else
    used = false;
  return used;
}

void UUltrasound::readHeight()
{
  if (Serial4.available())
  {
    bool isOK = false;
    rx[rxCnt] = Serial4.read();
    if (rx[rxCnt] == '\r')
    { // a measurement is received
      rx[rxCnt] = '\0';
      if (hasGotR and rxCnt > 1 and rxCnt < 8)
      {
        sampleTime = hbTimerCnt - timeHeight;
        timeHeight = hbTimerCnt;
        int stMs = getSampleTimeinMs();
        if (stMs < 500 and stMs > 0)
        {
          // decode height
          char * p1 = &rx[1];
          int hi = strtol(p1, NULL, 10);
          // this if from MaxBotix MB1040 that deliver data in inches
          // minimum distance is 6 inches (15cm) 
          // ~ also the distance from sensor to ground, so convert to height above ground
          // but better to leave it to the user.
          float hm = hi * 0.0254;
          // height change in m
          float dh = hm - heightRaw;
          heightRaw = hm;
          sonarHeight = heightRaw - heightOffset;
          // debug
          memcpy(rx2, rx, rxCnt + 1);
          // debug end
          // height velocity in m/s
          if (fabs(dh) < 20)
            sonarVelocity = dh / float(stMs) * 1000.0;
          else
            sonarVelocity = 0;
          isOK = true;
        }
      }
      sonarValid = isOK;
      hasGotR = false;
      rxCnt = 0;
    }
    else if (rx[rxCnt] >= 0)
    { // we got a character other than '\r'
      if (rxCnt == 0 and rx[0] == 'R')
      { // R is start of value
        hasGotR = true;
      }
      if (hasGotR)
      { // ready for next character
        rxCnt++;
      }
      if (rxCnt >= MRx)
      { // message overload
        rxCnt = 0;
      }
      rx[rxCnt] = '\0';
    }
  }
  else
  { // detect if no sonar is available
    int32_t dt = hbTimerCnt - timeHeight;
    // sonar is taken as valid if received data in the last 0.5 seconds
    sonarValid = dt < int(0.5 / SAMPLETIME) and dt > 0;
    if (not sonarValid)
    {
      sonarVelocity = 0;
      sonarHeight = 0;
      sampleTime = 0;
    }
  }
}

void UUltrasound::eePromLoad()
{
  heightOffset = eeConfig->readFloat();
  heightLimit = eeConfig->readFloat();
}

void UUltrasound::eePromSave()
{
  eeConfig->pushFloat(heightOffset);
  eeConfig->pushFloat(heightLimit);
}




