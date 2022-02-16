/***************************************************************************
 *   Copyright (C) 2019 by DTU (Christian Andersen)                        *
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

#ifndef UJOY_H
#define UJOY_H

#include <ubridge.h>
#include <udata.h>

#define BUTTON_GREEN    0
#define BUTTON_RED      1
#define BUTTON_BLUE     2
#define BUTTON_YELLOW   3
#define BUTTON_LB       4
#define BUTTON_LR       5
#define BUTTON_BACK 6
#define BUTTON_START 7
#define BUTTON_MANUAL 8
// axes 0 is left hand left-right axis
// axes 1 is left hand up-down axis
#define AXIS_SERVO_2   1
// axes 2 is left front speeder
// axes 3 is right hand left-right
#define AXIS_TURN      3
// axes 4 is right hand up-down
#define AXIS_VEL       4
// axes 5 is right front speeder
// axes 6 id digital left-write
// axes 7 is digital up-down


class UJoy  : public UData
{
public:
  int axes[8] = {0,0,0,0,0,0,0};
  // axis 0 is left hand left-right axis
  // axis 1 is left hand up-down axis (servo)
  // axis 2 is left front speeder
  // axis 3 is right hand left-right (turn)
  // axis 4 is right hand up-down (velocity)
  // axis 5 is right front speeder
  // axis 6 id digital left-write
  // axis 7 is digital up-down 
  bool button[11] = {0,0,0,0,0,0,0,0,0,0,0};
  // first 4 is [0]=green, [1]=red, [2]=blue, [3]=yellow
  // [4] = LB, [5] = RB, [6]=back, [7]=start, 
  // [8]=logitech logo, [9]=left knob, [10]=right knob
  bool manual = false;
  /**
   * \param data is pointer to parent of all elements */
  UJoy(UDataBase * data);
  /** open logfile and write initial comment */
  void openLog() override;
  /// decode gamepad message from bridge
  bool decode(const char * msg, int source, const char * key, bool isCheckedOK) override;
  /// subscribe to joystick data
  void subscribeFromHW(bool fromTeensy) override;
  /**
   * Print status for bridge and all data elements */
  void printStatus(int client) override;
  /**
   * send help */
  void sendHelp(const int client) override;
};



#endif
