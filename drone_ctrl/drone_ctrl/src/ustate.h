 /***************************************************************************
 * 
 *  Drone state control, 
 *  e.g. Disarmed, Armed
 *  Drone name and ID
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

#ifndef USTATE_H
#define USTATE_H

#include <stdint.h>
#include "main.h"
#include "usbus.h"
#include "uesc.h"

class UState
{
public:
  /**
   * constructor */
  UState();
  /**
   * Setup */
  void setup();
//   { // nothing to do
//     time = 0.0;
//     lasthbTime = hb10us;
//     deviceID = 1;
//     leds->stateSet(armState);
//   }
  /**
   * send help */
  void sendHelp();
  /**
   * decode command for this unit */
  bool decode(const char * buf);  
  /**
   * Checks for state change
   * to ESC values */
  void tick();
  /**
   * armcheck using rc switches */
  void checkArmState();
  /**
   * set fail state */
  void setFailed();
  /**
   * send state (heartbeat) */
  void sendState();
  /**
   * are we in bypass mode (ie. running without remote control) */
  inline bool isBypass()
  {
    return bypassRC;
  }
  /**
   * is control from RC or USB 
   * \returns true, if control is from USB (auto from raspberry or manual from GUI) */
  inline bool isUsbControl()
  {
    return usbCtrl;
  }
  /**
   * Is drone armed */
  inline bool isArmed()
  {
    return armState == Armed;
  }
  /**
   * Is drone armed */
  inline bool isLanding(float * trustFactor)
  {
    bool is = false;
    //const float landTime = 2.0; // seconds
    if (flightState == Landing)
    {
      if (trustFactor != nullptr)
        *trustFactor = (landTime - float(landingCounter) * SAMPLETIME) / landTime;
      is = true;
    }
    return is;
  }
  /**
   * Get state as integer (see coding below: 0=init, 1=Disarmed, ...) */
  inline int getState()
  {
    return armState;
  }
  /**
   * load from flash */
  void eePromLoad();
  /**
   * save to flash */
  void eePromSave();
private:
  /**
   * Set to armed state, if conditions are fulfilled */
  void setArmed(bool value);
  
public:
  /// system state
  enum ArmState {Init, Disarmed, Armed, Fail};
  /// flight state
  enum FlightState {OnGround, Starting, InFlight, Landing};
  /// Estimated flight state
  /// OnGround (0), Starting (1), InFlight (2), Landing (3)
  FlightState flightState = OnGround;
  /// time in seconds (since power on)
  float time;
  /// device ID
  int deviceID;
  /// and name
  static const int MAX_NAME_LENGTH = 32;
  char droneName[MAX_NAME_LENGTH] = "noname";
  
private:
  uint32_t lasthbTime;
  bool bypassRC = false; /// no RC safety if true
  bool usbCtrl = false;  /// control signals from USB connection if true
  /// main Arm state
  /// Init (0), Disarmed (1), Armed (2), Fail (3)
  ArmState armState = Init;
  /// counters
  int landingCounter = 0;
  int startingCounter = 0;
  const float landTime = 2.0;
  int msgCnt = 0;
  /// request arm
  bool tryArm = false;
  /// emergency switch
  bool stopNow = false;
};

#endif
