/***************************************************************************
 *   Copyright (C) 2021 by DTU                                             *
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

#ifndef REGBOT_MAIN_H
#define REGBOT_MAIN_H

/// Minor revision must be no bigger than 9 
/// (not to overflow 16 bit integer when added to svn version number*10 
///  - revision number is in command.cpp)
#define REV_MINOR 1

#include <string.h>
#include <stdlib.h>
#include <core_pins.h>
#include <usb_serial.h>
#include <HardwareSerial.h>

/*
 * giver problemer med compiler gcc 7.5 og 9.3 - gives:
 * libraries/ADC/settings_defines.h:445:5: error: 
 * body of constexpr function:
 * 'constexpr uint32_t ADC_settings::get_CFG_VERY_LOW_SPEED(uint32_t)' not a return-statement
 * An issue with C++17 that is supported from GCC version 10.0
 * */

#include <ADC.h>
#include "pins.h"

// control period is time between control calls
// and is in units of 10us, ie 125 is 1.25ms or 800Hz
#define CONTROL_PERIOD_10us 250
// sample time in seconds
#define SAMPLETIME  (0.00001 * CONTROL_PERIOD_10us)
//
extern volatile uint32_t hb10us;     /// heartbeat timer count (10 us)
extern volatile uint32_t hbTimerCnt; /// counts control period (1ms or 2.5ms) (not assumed to overflow)
extern int usbWriteFullCnt;
extern unsigned int cycleTimerInUs; /// heartbeat timer interrupt every (us)
extern int adcFullCycles; /// count ADC cycles in a control period
extern int adcFullCyclesMax; // allow 4 measurements per control period (or more if no IMU)


inline void setDebugLed(uint8_t value) {
	digitalWriteFast(PIN_LED_DEBUG, value);
}

class UEsc;
class USensor;
class UPropShield;
class UControl;
class UMixer;
class ULogger;
class USbus;
class USubscribe;
class UState;
class ULEDs;
class UUltrasound;
class UHeight;
class EEConfig;
class UCommand;
class ULaser;


extern UEsc * esc;
extern USensor * sensor;
extern UPropShield * imu;
extern UControl * control;
extern UMixer * mixer;
extern ULogger * logger;
extern USbus * rc;
extern USubscribe * subscribe;
extern UState * state;
extern ULEDs * leds;
extern UUltrasound * uhgt;
extern ULaser * uhlas;
extern UHeight * hgt;
extern EEConfig * eeConfig;
extern UCommand * command;


#endif
