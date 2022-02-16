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

#ifndef UGAMEPAD_H
#define UGAMEPAD_H

#include <ubridge.h>
#include <udata.h>


/**
 * Class to manage interface to joystick - actually a gamepad.
 * Is tested with Logitech F710, but should work with most js-devices.
 * */
class UGamepad  : public UData
{
public:
  /**
   * \param data is pointer to parent of all elements */
  UGamepad(UDataBase * data);
  /** open logfile and write initial comment */
  void openLog() override;
  /// decode gamepad message from bridge
  //bool decode(const char * msg, int source, const char * key, bool isCheckedOK) override;
  /**
   * Update temp message for subscription */
  void updateData();
  
  /**
   * Print status for bridge and all data elements */
  void printStatus(int client) override;
  /**
   * send help */
  void sendHelp(const int client) override;
  /**
   * update tick */
//   void tick() override;
  /**
   * update thread */
  void run() override;
private:
  /**
   * Open joustick device,
   * \returns false if device not found */
  bool initJoy();
  /**
   * Get fresh data from joystick 
   * \return false if device disappeared of received other than event data */
  bool getNewJsData();
  
private:
  bool connected = false;
  // device
  const char * joyDevice = "/dev/input/js0";
  int jDev = 0;  ///File descriptor
  
public:
  //Structure to hold joystick values
  struct jVal {
    bool button[16];
    int axes[16];
  };
  struct jVal joyValues;
  int number_of_axes = 8; 
  int number_of_buttons = 11;
  int updateCnt = 0;  
};



#endif
