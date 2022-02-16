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

#ifndef UHEIGHT_H
#define UHEIGHT_H

#include <stdint.h>
#include "main.h"
#include "controlbase.h"

class UHeight
{
public:
  /**
   * constructor */
  UHeight();
  /**
   * Setup */
  void setup();
  /**
   * Decode mixer info command */
  void sendStatus();
  /**
   * get low pass filter tau */
  void sendFilterStatus();
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
  void combineHeight(uint32_t mainLoop);
  
  void sendPose();
  /**
   * save steering configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();

private:
  /**
   * Altitude complementary filter with pressure altitude and accelerometer
   * from:
   * "Altitude data fusion utilising differential measurement and complementary filter" 
   * by Sheng Wei, Gu Dan and Hu Chen
   * calculates also height velocity
   * height velocity is further low-pass filtered.
   */
  void altComplFiltTick(uint32_t mainLoop);
  

public:
  float height; /// in m
  float heightAcc; /// height acceleration from rotated acc vector
  float heightVelocity; /// in m/s - merged with sonar and filtered
  float altitudef = 0; // filtered version of complementary filter
private:
  //float altitudeLast; // pressure altitude
  // velocity in altitude
//   float z_vel;
  // filtered velocity in altitude - out of complementary filter
  float z_vel_fil = 0;
  float lowPassTau = 0.1;
  
  float mergeFactor = 0.9; // trust ultrasonic factor (if below limit)
//   float sonarTrustLimitMax = 6.0; // m
  float filterTau = 0.3; // second
  int dvCount = 0;
  
  // Second order compl filt 
  UTransferFunctionI int1;
  UTransferFunctionI int2;
  // Lowpass filter for height velocity
  UTransferFunctionPZ lowPass;
  //
  int txcnt = 0;
};

#endif
