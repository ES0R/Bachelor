/***************************************************************************
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
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
#include "ustate.h"
#include "logger.h"



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
  if (esc_installed > 4)
  {
    pinMode(PIN_ESC_05, OUTPUT);
    pinMode(PIN_ESC_06, OUTPUT);
  }
  if (esc_installed > 6)
  {
    pinMode(PIN_ESC_07, OUTPUT);
    pinMode(PIN_ESC_08, OUTPUT);
  }
  // set value to all motors
  // analogWriteFrequency(PIN_ESC_02, PWMfrq); /// frequency (Hz)
  analogWrite(PIN_ESC_01, 0);
  analogWrite(PIN_ESC_02, 0);
  analogWrite(PIN_ESC_03, 0);
  analogWrite(PIN_ESC_04, 0);
  if (esc_installed > 4)
  {
    analogWrite(PIN_ESC_05, 0);
    analogWrite(PIN_ESC_06, 0);
  }
  if (esc_installed > 6)
  {
    analogWrite(PIN_ESC_07, 0);
    analogWrite(PIN_ESC_08, 0);
  }
  setPWMfrq(PWMfrq);
  // used by DAConverter - but may influence motor controller
  // disable esc (analog value = 0)
  for (int i = 0; i < MAX_ESC_CNT; i++)
  {
    escValue[i] = 0;
    escIncrementVel[i] = 0; // rate limit (0=no limit)
    escRef[i] = 0;
    escRefMan[i] = 0;
    escManual[i] = false;
    escIncrementInterval[i] = 0;
    escIncrementIntervalCnt[i] = 0;
  }
}


void UEsc::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  command->usb_send_str("# ---- ESC ----\n\r");
  snprintf(reply, MRL, "#   esc N M P V T  Set esc N=1..%d, M:manual, P:(position):0..1024, V (velocity): 0=full, else values/per T, T in ms\r\n"
  "#                (status: N=1..3 [Manual %d,%d,%d, P= %d, %d, %d, V= %d, %d, %d)\r\n", 
           MAX_ESC_CNT,
           escManual[0], escManual[1], escManual[2],
           int(escValue[0]), int(escValue[1]), int(escValue[2]),
             escIncrementVel[0], escIncrementVel[2], escIncrementVel[2] 
  );
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   esi          Get esc status (esd [enabled value velocity]x8 )\r\n");
  command->usb_send_str(reply);
}


bool UEsc::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "esi", 3) == 0)
  {
    sendEscStatus();
  }
  else if (strncmp(buf, "esc ", 4) == 0)
  {
    setOneEsc(&buf[4]);
  }
  else
    used = false;
  return used;
}


void UEsc::sendEscStatus()
{ // return esc status
  const int MSL = 100;
  char s[MSL];
  // changed to svs rather than svo, the bridge do not handle same name 
  // both to and from robot - gets relayed back to robot (create overhead)
  snprintf(s, MSL, "esd %d %d %d %d  %d %d %d  %d %d %d  %d %d %d "
                         " %d %d %d  %d %d %d  %d %d %d  %d %d %d\r\n", 
           PWMfrq,
           escManual[0], int(escValue[0]), escIncrementVel[0],
           escManual[1], int(escValue[1]), escIncrementVel[1],
           escManual[2], int(escValue[2]), escIncrementVel[2],
           escManual[3], int(escValue[3]), escIncrementVel[3],
           escManual[4], int(escValue[4]), escIncrementVel[4],
           escManual[5], int(escValue[5]), escIncrementVel[5],
           escManual[6], int(escValue[6]), escIncrementVel[6],
           escManual[7], int(escValue[7]), escIncrementVel[7]
  );
  command->usb_send_str(s);
}

void UEsc::setPWMfrq(const char* line)
{
  // pwmfrq 400\n
  const char * p1 = line;
  int frq = strtol(p1, (char**)&p1, 10);
  if (p1 != line)
    // valid number
    setPWMfrq(frq);
}

void UEsc::setPWMfrq(int frq)
{
  // pwmfrq 400\n
  if (frq >=25 and frq < 500)
  {
    PWMfrq = frq;
    msPulseCount = (max_pwm * PWMfrq) / 1000; // ~1600 at 400 Hz
    /// center position (1.5ms (~2400)), or 1ms (~1600)
    pulseCountOffset = msPulseCount; 
    analogWriteFrequency(PIN_ESC_01, PWMfrq); /// frequency (Hz)
    analogWriteFrequency(PIN_ESC_02, PWMfrq); /// frequency (Hz)
    analogWriteFrequency(PIN_ESC_03, PWMfrq); /// frequency (Hz)
    analogWriteFrequency(PIN_ESC_04, PWMfrq); /// frequency (Hz)
    analogWriteFrequency(PIN_ESC_05, PWMfrq); /// frequency (Hz)
    analogWriteFrequency(PIN_ESC_06, PWMfrq); /// frequency (Hz)
    analogWriteFrequency(PIN_ESC_07, PWMfrq); /// frequency (Hz)
    analogWriteFrequency(PIN_ESC_08, PWMfrq); /// frequency (Hz)
    // debug
    const int MSL = 130;
    char s[MSL];
    snprintf(s, MSL, "# setting PWM frq to %d Hz => 1ms = %d count\n", PWMfrq, msPulseCount);
    command->usb_send_str(s);
  }
}

void UEsc::setOneEsc(const char* line)
{
  // command 
  // esc N e us
  // esc is ESC number (0..7)
  // e  0= disable (no PWM)
  // us is PWM value in us
  const char * p1 = line;
  int8_t idx = strtol(p1, (char**)&p1, 10);
  if (idx >= 0 and idx < MAX_ESC_CNT)
  {
    bool enable = strtol(p1, (char**)&p1, 10);
    // PWM - from 0.5ms (-512) to 2.4 ms
    int us = strtol(p1, (char**)&p1, 10);
    if (us < -512)
      us = -512;
    else if (us > 1470)
      us = 1470;
    int16_t vel = strtol(p1, (char**)&p1, 10);
    int16_t interval = strtol(p1, (char**)&p1, 10);
    setEscPWM(idx, us, enable, vel, interval); 
    const int MSL = 64;
    char s[MSL];
    snprintf(s, MSL, "#setting ESC %d to %dus vel=%d in=%d (use esi for status)\r\n", idx, us, vel, interval);
    command->usb_send_str(s);
    if (interval > 0)
      logger->restartLog();
  }
  else
    command->usb_send_str("# unknown esc: %s\r\n", line);
}


void UEsc::escTick()
{ // speed limit on esc
  const int escPin[MAX_ESC_CNT] = {PIN_ESC_01, PIN_ESC_02, PIN_ESC_03, PIN_ESC_04, PIN_ESC_05, PIN_ESC_06, PIN_ESC_07, PIN_ESC_08};
  escLimiting = false;
  float A[8] = {1,1,1,1,1,0.61,1,1};
  for (int i = 0; i < MAX_ESC_CNT; i++)
  { // implement PWM to actuators
    if ((state->getState() >= state->Armed) or escManual[i])
    { // check if value has changed
      int dw;
      if (escManual[i])
        dw = escRefMan[i] - escValue[i];
      else
        dw = escRef[i] - escValue[i];
      // *** implement rate limit
      // PWM velocity (range is 0..1024) - esc value in ~us (1-2ms):
      //     0 = Fastest (ESC decide)
      //     1 = value (out of 1000) per tick
      //     2 = values per tick
      // *** escInterval is how often should velocity be stepped
      if (escIncrementVel[i] > 0)
      {
        if (escIncrementIntervalCnt[i] >= escIncrementInterval[i])
        { // time to increment
          if (abs(dw) > escIncrementVel[i])
          {
            if (dw > 0)
              dw = escIncrementVel[i];
            else
              dw = -escIncrementVel[i];
          }
          escIncrementIntervalCnt[i] = 0;
        }
        else
        { // not time
          dw = 0;
          escIncrementIntervalCnt[i]++;
        }
      }
      // implement new value
      escValue[i] += dw;
    }
    else
    { // set actuators to idle value
      escValue[i] = idleValue[i];
    }
    /// limiting flag set, but actual limiting is left to the ESC or servo
    /// @todo escLimiting value not used pt.
    escLimiting |= (escValue[i] < 0) or (escValue[i] > 1024);
    // implement
    // PWM counter value (12 bit and 400Hz)
    // input 0..1024 gives 1ms to 2ms pulse
    // slightly more and less is allowed
    int v = (escValue[i] * msPulseCount) / 1024 + pulseCountOffset;

    analogWrite(escPin[i], v);// * A[i]);
    //       // debug
//       if (hbTimerCnt % 100 == 0 and i == 2)
//       {
//         const int MSL = 200;
//         char s[MSL];
//         snprintf(s, MSL,"# escTick: (sample %ld) m3: ref=%d, dw = %d, v=%d\n", hbTimerCnt, escRef[i], dw, v);
//         command->usb_send_str(s);
//       }
//       // debug end
  }
}

///////////////////////////////////////////////////////

void UEsc::eePromSave()
{
  // save desired PWM FRQ
  eeConfig->pushWord(PWMfrq);
  eeConfig->pushByte(esc_installed);
  eeConfig->pushWord(escLimitMax);
  for (int i = 0; i < MAX_ESC_CNT; i++)
    eeConfig->pushWord(idleValue[i]);
}

void UEsc::eePromLoad()
{
    PWMfrq = eeConfig->readWord();
    esc_installed = eeConfig->readByte();
    escLimitMax = eeConfig->readWord();
    for (int i = 0; i < MAX_ESC_CNT; i++)
      idleValue[i] = eeConfig->readWord();
    // implement FRQ - stops all motors first.
    initEsc();
}

void UEsc::stopAllMotors()
{ // NB! this will stop motors in Disarmed state only
  for (int i = 0; i < MAX_ESC_CNT; i++)
  { // disable motor next time pulse to ESC is 0
    escRef[i] = 0; // from mixer
    escValue[i] = 0; // actual value
    escRefMan[i] = 0; // manual value
    escManual[i] = false; // set to auto
    escIncrementVel[i] = 0;
    escIncrementInterval[i] = 0;
    escIncrementIntervalCnt[i] = 0;
  }
}


void UEsc::setEscPWM(int esc, int16_t pwm, bool enable, int16_t vel, int16_t interval_ms)
{
  if (esc >= 0 and esc < MAX_ESC_CNT)
  {
    escRefMan[esc] = pwm; // -512 to 1530 (0.5ms to 2.45ms)
    escManual[esc] = enable;
    escIncrementVel[esc] = vel;
    if (vel > 0)
    { // relevant if increment is != 0 only
      if (interval_ms <= 1)
        escIncrementInterval[esc] = 1;
      else
        escIncrementInterval[esc] = roundf(interval_ms / float(CONTROL_PERIOD_10us) / 100.0);
    }
  }
}
