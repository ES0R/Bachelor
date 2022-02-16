/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * Motor controller functions - controlling Pololu1213 dual motor controller
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
 
#ifndef ESC_ON_REGBOT_H
#define ESC_ON_REGBOT_H

#include <stdint.h>
#include "main.h"
#include "command.h"
#include "pins.h"

class UEsc
{
public:
  /** 
   * constructor (niit) */
  UEsc();
  static const int MAX_ESC_CNT = 6;
  /** else disabled (no pulse) */
  bool escEnabled[MAX_ESC_CNT];
  /** last commanded value */
  int32_t escValue[MAX_ESC_CNT];
  int16_t escVel[MAX_ESC_CNT];
  int16_t escRef[MAX_ESC_CNT];
  bool    escEnaRef[MAX_ESC_CNT];
  /** PWM frequency (400 Hz is probably maximum) */
  int PWMfrq = 400; // Hz - 3ms update rate
  /// is esc 1 steered front wheel
  bool esc1isSteering;
  /**
  * set PWM port of frekvens */
  void initEsc();
  float escVal[MAX_ESC_CNT];
  /**
  * Set esc 1 PWM value
  * \param pwm allowed input is +/- 512, where
  * 0 is center (1.5ms)
  *  \param enable if false, then PWM pulse is stopped (esc disables),
  * but port is still an output port
  * \param vel is max velocity for esc 0=no limit 1 is slow 1 value per second 999 is 999 values per second.
  * */  
  inline void setEscPWM(int esc, int16_t pwm, bool enable, int16_t vel)
  {
    if (esc >= 0 and esc < MAX_ESC_CNT)
    {
      escRef[esc] = pwm;
      escEnaRef[esc] = enable;
      escVel[esc] = vel;
    }
  }
  /** 
   * \param pin allowed pin is 0,1. 
   * \param value input true is 1
   * \param enable if false, then port in set as input
   * */
  void setEscPin(int8_t pin, int16_t value, bool enable);
  /**
   * set any esc or IO pin/value */
  void setEsc(int8_t idx, int16_t value, bool enable, int8_t vel);
  /**
   * stop PWM to escs and set esc pins to input */
  void releaseEscs();
  /**
   * send esc status to client */
  void sendEscStatus();
  /**
   * set esc values from string */
  void setEscConfig(const char * line);
  /**
   * set one esc or pin (mostly for debug) */
  void setOneEsc(const char * line);
  /**
   * set one esc or pin (mostly for debug) */
  void setPWMfrq(const char * line);
  void setPWMfrq(int frq);
  /**
   * 5ms update of esc */
  void escTick();
  
  /**
   * save steering configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  
  
private:
  // 1ms = frq/12bit
  static const int max_pwm = 4096;
  /// pwm value to give 1ms
  int msPulse;
  /// center position (1.5ms), now 1ms for one way only
  int midt;
};

#endif // MOTOR_CONTROLLER_H
