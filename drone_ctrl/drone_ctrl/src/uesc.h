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
  static const int MAX_ESC_CNT = 8;
  // actual installed ESCs
  int8_t esc_installed = 6;
  /** manual override - direct control of ESC (from GUI)
   *  (else in auto mode) */
  bool escManual[MAX_ESC_CNT];
  /**
   * idle value 
   * value to use if disarmed */
  int idleValue[MAX_ESC_CNT];
  /** Desired ESC value (target for rate limit)
   *  from mixer (auto mode)
   * */
  int16_t escRef[MAX_ESC_CNT];
  /** last commanded value (after rate limit) */
  int32_t escValue[MAX_ESC_CNT];
  /** rate limit 0=no limit, else this value change per second (approx)
   * */
  int16_t escIncrementVel[MAX_ESC_CNT];
  /** Interval between increments in sample ticks
   * */
  int16_t escIncrementInterval[MAX_ESC_CNT];
  uint16_t escIncrementIntervalCnt[MAX_ESC_CNT];
  /** manual value, used when escRefMan is true */
  int16_t escRefMan[MAX_ESC_CNT];
  /**
   * Is set true, if any motor is limiting output */
  bool escLimiting = false;
  /**
   * Limit ESC power input (PWM input) */
  int16_t escLimitMax = 1023;
//   /** Is ESC in manual overide mode */
//   bool    escEnaManual[MAX_ESC_CNT];
  /** PWM frequency (400 Hz is probably maximum) */
  int PWMfrq = 400; // Hz - 400Hz => 2.5ms update rate
  /**
  * set PWM port of frekvens */
  void initEsc();
  /**
   * send command help */
  void sendHelp();
  /**
   * decode commands */
  bool decode(const char * buf);
//   float escVal[MAX_ESC_CNT] = {0};
  /**
  * Set esc PWM value directly to a manual value
  * \param pwm allowed input is 0..1024, where
  * 0 is 1ms and 1024 is 2ms
  * \param enable if false, then PWM pulse is stopped (esc disables),
  * but port is still an output port
  * \param vel is max velocity for esc 0=no limit 1 is slow (about) 1 value per second 999 is 999 values per second.
  * */  
  void setEscPWM(int esc, int16_t pwm, bool enable, int16_t vel, int16_t interval_ms);
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
//   void releaseEscs();
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
   * 2.5ms update of esc */
  void escTick();
  /**
   * save configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  /**
   * emergency stop */
  void stopAllMotors();
  
  
private:
  // 1ms = frq/12bit
  static const int max_pwm = 4095; // 12 bit
  /// pwm value to give 1ms
  int msPulseCount;
  /// pwm at center position 1.5ms for signed or 1ms for one way only (trust)
  int pulseCountOffset;
};

#endif // MOTOR_CONTROLLER_H
