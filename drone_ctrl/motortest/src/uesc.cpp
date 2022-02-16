/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a small 72MHz ARM processor MK20DX256,
 *   intended for 31300 Linear control
 *   has an IMU and a dual motor controller with current feedback.
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
 ***************************************************************************//*
  This file contains all the functions used for calculating
  the frequency, real Power, Vrms and Irms.
*/
#include <stdlib.h>
#include "main.h"
#include "uesc.h"
#include "eeconfig.h"
#include "pins.h"



UEsc::UEsc()
{
//   initEsc();
}

void UEsc::initEsc()
{
  // resolution set by motor controller
  //analogWriteRes(10); /// resolution (10 bit)
  // frequency is common for a motor-pin too - on HW version 3
  pinMode(PIN_ESC_01, OUTPUT);
  pinMode(PIN_ESC_02, OUTPUT);
  pinMode(PIN_ESC_03, OUTPUT);
  pinMode(PIN_ESC_04, OUTPUT);
  pinMode(PIN_ESC_05, OUTPUT);
  pinMode(PIN_ESC_06, OUTPUT);
  // analogWriteFrequency(PIN_ESC_02, PWMfrq); /// frequency (Hz)
  analogWrite(PIN_ESC_01, 0);
  analogWrite(PIN_ESC_02, 0);
  analogWrite(PIN_ESC_03, 0);
  analogWrite(PIN_ESC_04, 0);
  analogWrite(PIN_ESC_05, 0);
  analogWrite(PIN_ESC_06, 0);
  setPWMfrq(PWMfrq);
  // used by DAConverter - but may influence motor controller
  // disable esc (analog value = 0)
  for (int i = 0; i < MAX_ESC_CNT; i++)
  {
    escEnabled[i] = false;
    escValue[i] = 0;
    escVel[i] = 0;
    escRef[i] = 0;
    escEnaRef[i] = false;
  }
}


void UEsc::releaseEscs()
{
//   usb_send_str("# releasing escs\r\n");
  for (int i = 0; i < MAX_ESC_CNT; i++)
  {
      setEscPWM(i, 0, false, 1);
  }
}



void UEsc::sendEscStatus()
{ // return esc status
  const int MSL = 100;
  char s[MSL];
  // changed to svs rather than svo, the bridge do not handle same name 
  // both to and from robot - gets relayed back to robot (create overhead)
  snprintf(s, MSL, "esd %d %d %d %d  %d %d %d  %d %d %d  %d %d %d "
                   " %d %d %d  %d %d %d\r\n", 
           PWMfrq,
           escEnabled[0], int(escValue[0]/100), escVel[0],
           escEnabled[1], int(escValue[1]/100), escVel[1],
           escEnabled[2], int(escValue[2]/100), escVel[2],
           escEnabled[3], int(escValue[3]/100), escVel[3],
           escEnabled[4], int(escValue[4]/100), escVel[4],
           escEnabled[5], int(escValue[5]/100), escVel[5]
  );
  usb_send_str(s);
}

void UEsc::setPWMfrq(const char* line)
{
  // pwmfrq 400\n
  const char * p1 = line;
  int frq = strtol(p1, (char**)&p1, 10);
  if (p1 != line)
  {
    if (frq >=25 and frq < 500)
    {
      PWMfrq = frq;
      msPulse = (max_pwm * PWMfrq) / 1000;
      /// center position (1.5ms), no 1ms
      midt = msPulse; // (msPulse * 3) / 2;
      analogWriteFrequency(PIN_ESC_02, PWMfrq); /// frequency (Hz)
      //
      const int MSL = 100;
      char s[MSL];
      snprintf(s, MSL, "# setting PWM frq to %d Hz\n", PWMfrq);
      usb_send_str(s);
    }
  }
}

void UEsc::setPWMfrq(int frq)
{
  // pwmfrq 400\n
  if (frq >=25 and frq < 500)
  {
    PWMfrq = frq;
    msPulse = (max_pwm * PWMfrq) / 1000;
    /// center position (1.5ms), no 1ms
    midt = msPulse; // (msPulse * 3) / 2;
    analogWriteFrequency(PIN_ESC_02, PWMfrq); /// frequency (Hz)
  }
}

void UEsc::setOneEsc(const char* line)
{
    const char * p1 = line;
    int8_t idx = strtol(p1, (char**)&p1, 10);
    if (idx >= 0 and idx < MAX_ESC_CNT)
    {
      int us = strtol(p1, (char**)&p1, 10);
      int vel = strtol(p1, (char**)&p1, 10);
      bool enable;
      enable = us >= -512 and us <= 1536;
      setEscPWM(idx, us, enable, vel); 
      const int MSL = 64;
      char s[MSL];
      snprintf(s, MSL, "#setting ESC %d to %dus vel=%d (use esi for status)\r\n", idx, us, vel);
      usb_send_str(s);
    }
    else
      usb_send_str("# unknown esc: %s\r\n", line);
}


void UEsc::escTick()
{ // speed limit on esc
  const int escPin[MAX_ESC_CNT] = {PIN_ESC_01, PIN_ESC_02, PIN_ESC_03, PIN_ESC_04, PIN_ESC_05, PIN_ESC_06};
  for (int i = 0; i < MAX_ESC_CNT; i++)
  {
    if (escEnabled[i] != escEnaRef[i])
    { 
      if (not escEnaRef[i])
      { // closing
        if (not digitalReadFast(escPin[i]))
        { // output is zero, time to disable PWM for esc (konstant 0)
          analogWrite(escPin[i], 0);
          escEnabled[i] = false;
        }
      }
      else
        escEnabled[i] = true;
    }
    //
    // set esc position - if enabled
    if (escEnabled[i])
    { // often, this is a fast esc
      // velocity
      //     0 = Fastest (esc decide)
      //     1 = 1/100 value per tick
      //     2 = 2/100 values per tick
      //     ...
      //     999 = 9.99 values per tick
      int dw = escRef[i] * 100 - escValue[i];
      if (abs(dw) > escVel[i] and escVel[i] > 0)
      {
        if (dw > 0)
          dw = escVel[i];
        else
          dw = -escVel[i];
      }
      if (dw != 0)
      { // implement new value
        escValue[i] += dw;
        // midt v= 2040, min=1240, max= 2840
        // valid for HiTec HS7235-MH in high angle mode
        int v = ((escValue[i]/100) * msPulse) / 1024 + midt;
        escVal[i] = float(v) / float(msPulse);
//         const int MSL = 64;
//         char s[MSL];
//         snprintf(s, MSL, "#set esc %d pin %d to v=%d (dw=%d, ref=%d val=%ld)\r\n",i, escPin[i], v, dw, escRef[i], escValue[i]);
//         usb_send_str(s);
        analogWrite(escPin[i], v);
      }
    }
  }
}

///////////////////////////////////////////////////////

void UEsc::eePromSave()
{
  uint8_t flag = 0;
  // flags
//   if (esc1isSteering)
//     flag +=  1 << 0;
  eeConfig.pushByte(flag);
}

void UEsc::eePromLoad()
{
    uint8_t flag;
    flag = eeConfig.readByte();
    // enabeled
    if (flag & 0x01)
      usb_send_str("#loading ESC data from ee\r\n");
//     esc1isSteering = (flag & (1 << 0)) != 0;
}


