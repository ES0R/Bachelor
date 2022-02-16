 /***************************************************************************
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

#ifndef USBUS_H
#define USBUS_H

#include <stdint.h>
#include <HardwareSerial.h>
#include "main.h"
#include "command.h"

#include "SBUS.h"


class USbus 
{
protected:
  uint16_t channel[16];
  uint16_t offset[16];
  float scale[16];
  bool rcLost = true; // RC is not detected, or lost
  bool lostFrame = true;
  int lostFrames = 0;
  int frameCnt = 0;
  uint32_t lastFrameTime = ARM_DWT_CYCCNT;
  float sampleTime = 20; // ms
  float noFrameTime = 0;
  
public:
  void setup()
  {
    rcbus->begin();
  }
  /// service SBUS seriel channel Serial1
  void tick(uint32_t tickCnt);
  /**
   * send help */
  void sendHelp();
  /**
   * decode command for this unit */
  bool decode(const char * buf);
  /// send RC status to USB 
  void sendStatus();
  /**
   * send scale and offset */
  void sendScaleOffset();
  /**
   * Check if control sticks are in an arm-able state */
//   bool canArm();
  bool isRcLost()
  {
    return rcLost;
  };
  /**
   * should we disarm */
//   bool disarmcheck();
  /**
   * is drone armable
   * Checks active RC and arm-switch (SC) only */
  bool armRequestCheck();
  /**
   * auto mode switch.
   * Auto means roll,pitch and yaw is allowed from USB
   * \returns true if auto switch is on. */
  bool autoCheck();
  
  /**
   * start datalog */
  bool logswitch();
  /**
   * switch to channel 
   * \param sw is {a,b,c,d,e,f,g,h}
   * \return channel or -1 if not valid */
  int switchToChannel(char sw);
  
  
  /**
   * get RC connection strength */
  int getRSSI()
  {
    return channel[7];
  };
  /**
   * get fail count (lost frames since last good frame from SBus) */
  int getLostFrameCount()
  {
    return lostFrames;
  };
  /**
   * save scale and offset */
  void eePromSave();
  /**
   * load scale and offset */
  void eePromLoad();
  
protected:
  void stickToRef();
  
protected:
  SBUS * rcbus = new SBUS(Serial1);
  int lastLogSwitch = 0;
};


#endif
