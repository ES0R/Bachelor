 /***************************************************************************
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk
 *
 * Main PID control (all)
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

#ifndef REGBOT_CONTROL_H
#define REGBOT_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "controlbase.h"
 

class UControl
{
public:
  /**
   * constructor */
  UControl();
  /**
   * setup of controllers */
  void setup()
  {
    setRegulatorInOut();
    resetControl();
  }
  bool decode(const char * buf);
  /**
   * decode command line
   * \param buf is the command line control values
   * first is controller name and possibly additional parameters.
   * \returns true if valid keywords are found. */
  bool decodeCtrl(const char * buf);
  /**
   * decode command line
   * \param buf if the command line new setpoints or nothing to get status
   * \returns true if valid keywords are found. */
  bool decodeRef(const char * buf);
  /** get limits for trust, roll, pitch and yaw velocity
   * */
  bool decodeLimit(const char * buf);
  /**
   * send help on control parameters
   * ctrl is keyword for control commands */
  void sendHelp();
  /**
   * send status about regulators 
   * \param line is a line received from client, that may be 
   * setting of a controller.
   * \returns true if line is used */  
  bool setRegulator(const char * line);
  /**
   * Send control status for one controller */
  bool sendStatusControl(const char * line);
  /**
   * save controller configuration to EE Prom */
  void eePromSaveCtrl();
  /**
   * load controller configuration from EE Prom
   * same order as when saved
   */
  void eePromLoadCtrl();
  /**
   * Reset all controllers, i.e. set all passed values to zero. */
  void resetControl();
  /**
   * This function is called every time it is time to do a new control calculation
   * */
  void controlTick(void); 
  /**
   * is control ref ready to arm */
//   bool canArm();
  /// get ref height
  inline float getRefHeight() { return refHeight;};
  /// get ref roll
  inline float getRefRoll() { return refRoll;};
  /// get ref pitch
  inline float getRefPitchRate() { return refPitchRate;};
  /// get ref pitch
  inline float getRefPitch() { return refPitch;};
  /// get ref yaw
  inline float getRefYaw() { return refYaw;};
  /// set control ref (setpoint)
  void setRef(float height, float roll, float pitch, float yaw);
  

public:  
//   UControlBase * ctrlVelHeight;
  UControlBase * ctrlVelRoll;
  UControlBase * ctrlVelPitch;
  UControlBase * ctrlVelYaw;
  UControlBase * ctrlVelHeight;
  UControlBase * ctrlRoll;
  UControlBase * ctrlPitch;
//   UControlBase * ctrlYaw;
  //
//   float refVelHeight = 0;  
  float refRollRate   = 0;
  float refPitchRate = 0;  
  float refYawRate = 0;
private:  
  float refHeight = 0;
  float refRoll = 0;
  float refPitch = 0;
  float refYaw = 0;
  float refHeightMin = 0, refHeightMax = 1000; // esc value (us)
  float refRollMin = -5, refRollMax = 5;
  float refPitchMin = -5, refPitchMax = 5; // angle degrees
  float refYawMin = -5, refYawMax = 5; // yaw velocity deg/s
  
protected:
  /**
   * Initialize input and output of controllers */
  void setRegulatorInOut();
private:
  //
  int tick;
  /// mission end time (theEnd got true)
  float endTime;

public:
  bool controlActive;
};

#endif
