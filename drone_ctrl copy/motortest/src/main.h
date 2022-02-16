/***************************************************************************
 *   Copyright (C) 2019 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a teensy 3.1 72MHz ARM processor MK20DX256 - or any higher,  *
 *   intended for 31300/1 Linear control 1
 *   has an IMU and a dual motor controller with current feedback.
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

//#define REV_MAIN 3
//#define REV "$Rev: 982 $" - moved to command.cpp (use getRevisionNumber())
/// Minor revision must be no bigger than 9 (not to overflow 16 bit integer when added to svn version number*10)
#define REV_MINOR 0

// debug
//#define REGBOT_HW4
// debug end

#include <string.h>
#include <stdlib.h>
#include <core_pins.h>
#include <usb_serial.h>
#include <HardwareSerial.h>
#include <ADC.h>
#include "pins.h"

// control period is time between control calls
// and is in units of 10us, ie 125 is 1.25ms or 800Hz
#define CONTROL_PERIOD_10us 250
#define SAMPLETIME  (0.00001 * CONTROL_PERIOD_10us)

extern int16_t robotId; // robot number [1..999]
/** hw version 1 no line sensor nor wifi
 * hw 2 no power control on board, no wifi, 
 * hw 3 build in power control and wifi, and servo header - big motor controller
 * hw 4 same as 3, but small motor controller
 * hw 5 same as 2, but with sattelite power and wifi boards 
 * hw 6 is for version 4.0 (teensy 3.5) with integrated motor controller
 * */
extern uint8_t robotHWversion; 

extern volatile uint32_t hb10us;     /// heartbeat timer count (10 us)
extern volatile uint32_t hbTimerCnt; /// in ms (not assumed to overflow)
extern int usbWriteFullCnt;
// Is communication with IMU possible (if not a power cycle is needed)
extern bool imuAvailable;
/// battery halt is when battery voltage is too low ,
/// mission is stopped and 12V power is cut off.
/// if on USB power, then the processor continues.
extern bool batteryHalt;
extern uint32_t mainLoop;
extern const int EEPROM_SIZE;
extern unsigned int cycleTimerInUs; /// heartbeat timer interrupt every (us)


inline void setDebugLed(uint8_t value) {
	digitalWriteFast(PIN_LED_DEBUG, value);
}

class UEsc;
class USensor;
extern UEsc * esc;
extern USensor * sensor;


#endif
