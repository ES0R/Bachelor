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

#ifndef ULASER_H
#define ULASER_H

#include <stdint.h>
#include "main.h"

class ULaser
{
public:
  /**
   * constructor */
  ULaser();
  /**
   * Setup */
  void setup();
  /**
   * Decode mixer info command */
  void sendStatus();
  /**
   * sendHelp */
  void sendHelp();
  /**
   * Decode commands from USB connection 
   * \param buf is the command line string buffer
   * \returns true is command is used. */
  bool decode(const char * buf);
  /**
   * Read height - if any */
  void readHeight();
  /**
   * get time in miliseconds */
  inline int getTimeinMs() 
  {
    return (timeHeightLast * CONTROL_PERIOD_10us)/100;
  }
  /**
   * Get sample time for ultrasound updates */
  inline int getSampleTimeinMs() 
  {
    return (sampleTime * CONTROL_PERIOD_10us)/100;
  }
  /**
   * save steering configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  
public:
  bool laserOK = false; // reply on open is OK
  bool laserValid = false;
  float laserHeight; /// in m
  float laserVelocity; /// in m/s
  int laserQuality; /// 4 digits low value is best quality
  float laserTemp = 0; // temperature in degrees C
  float laserVoltage  = 0; // laser supply voltage (2.0-3.0V)
  int timeHeightLast;
  int sampleTime = 1;
  float heightQLimit = 6.0; // usable height limit (0=not used)
  float heightOffset = 6 * 0.0254; // 6"
private:
  static const int MRx = 25;
  char rx[MRx];
  int rxCnt = 0;
  char rx2[MRx + 1];
  char rxErr[MRx];
  float heightRawLast = 0;
};

#endif
