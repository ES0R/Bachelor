/***************************************************************************
 *   Copyright (C) 2017 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a small 72MHz ARM processor MK20DX256,
 *   intended for 31300 Linear control
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

#ifndef COMMAND_H
#define COMMAND_H

#include <string.h>
#include <stdlib.h>
#include <usb_serial.h>
#include <core_pins.h>
#include <HardwareSerial.h>

extern int8_t localEcho;
extern bool button; // start switch 
// extern bool sendStatusWhileRunning;
extern uint32_t controlUsedTime[3]; /// control time in CPU units (0=sensor, 1=full cycle)
/// battery voltage measurement
extern uint16_t batVoltInt;
extern bool pinModeLed;
extern int pushInterval;
extern int pushTimeLast;
extern bool silentUSB;
extern float missionTime;



uint16_t getRevisionNumber();
void handleIncoming(uint32_t mainLoop);
/**
 * send one statusmessage every statusinterval, takes about 40 samples (status lines) to send all.
 * Every other status is measurements (current, gyro, etc).
 * This is for the Python GUI to get a semi-live status on screen.
 * For wifi the fastest sample rate is about 250 ms, i.e. 10 seconds for a full update,
 * on USB a sample rate of 50ms is OK */
//void sendStatus();
/**
 * Send message "hbt time"
 * where time is decimal seconds since last time reset (boot or mission start).
 * Is send to both USB and (one) wifi connection */
void sendHartBeat();
/**
 * Send (calibrated) accelerometer message (3D) */
void sendAcc();
/**
 * Send calibrated gyro message (3D) */
void sendGyro();

/**
 * Send motor current message x,y,h */
void sendMotorCurrent();


/**
 * Send string to USB channel - to whoom is controlled by global requestingClient,
 * requestingClient = -2, then send to all (USB and first wifi client)
 * requestingClient = -1, then USB only
 * requestingClient = 0..4, then to that wifi client only.
 * \param str is string to send
 * \param blocking set to true is this message needs to get through (e.g. log), 
 * else the whole message will be dropped if connection is busy.
 * \param toUSB do not send to USB connection if false (default true)
 * \param toWifi send to first alive if true (default) else send to none 
 * \returns true if send to all requested connections, false if not send to all 
 * active wifi connections (wifi busy or more to serve).
 * then data is lost (i.e. USB disconnected). */
bool usb_send_str(const char * str, bool blocking = false); //, bool toUSB = true, bool toWifi = true);

/**
 * Parse commands from the USB connection and implement those commands.
 * \param buf is string to send
 * \param serChannel is request source, 1- is USB, [0..4] is wifi client.
 * The function is served by the main loop, when time allows. */
void parse_and_execute_command(char *buf, int8_t serChannel);


#endif
