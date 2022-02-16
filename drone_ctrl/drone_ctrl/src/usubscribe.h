 /***************************************************************************
 * 
 *   Copyright (C) 2020 by DTU                             *
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

#ifndef U_SUBSCRIBE_H
#define U_SUBSCRIBE_H

#include <stdint.h>
#include "main.h"

/**
 * Class for continuous stream of data to automation controller
 * - sends stream og sensor and status data through USB connection
 * */
class USubscribe
{
public:
  /**
   * Decode subscription command
   * \param buf is the command line
   * \returns true is the command is known */
  bool decode(const char * buf);
  /**
   * help text for relevant commands */
  void sendHelp();  
  /**
   * Tick is called after each control loop
   */
  void tick(int mainLoop, bool gettingLog);
  /**
   * setup of subscription 
   * */
  void setup() 
  { // nothing needed
  }
private:
  /**
   * Data subscriprion flag(s)
   * value = 0 => no data
   * value = 1 => highest priority (send every cycle)
   * value = 2 => send every other cycle
   * value 3..n => send every nth cycle
   * */
  int sendPose = 0;
  int sendHb = 0;
  int sendImu = 0;
};


#endif
