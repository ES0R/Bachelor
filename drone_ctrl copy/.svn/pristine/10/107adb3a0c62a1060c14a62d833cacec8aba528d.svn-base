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

#ifndef COMMAND_H
#define COMMAND_H

#include <string.h>
#include <stdlib.h>
#include <usb_serial.h>
#include <core_pins.h>
#include <HardwareSerial.h>
#include <mutex>


class UCommand
{
public:
  UCommand()
  {
  }
  /**
  * Get revision number from SVN annotation */
  uint16_t getRevisionNumber();
  /**
  * Service the USB connection
  * and send any lines to decoding and execution */
  void handleIncoming();
  /**
  * Send string to USB channel
  * \param str is string to send
  * \param blocking set to true is this message needs to get through (e.g. log), 
  * else the whole message will be dropped if connection is busy.
  * \returns true if send to all requested connections, false if not send to all */
  bool usb_send_str(const char * str, bool blocking = true);
  /**
   * Send comment with time and this info.
   * \param info to send - placed after teensy time in seconds, like '# 123.456 info [msg]'
   * \param msg is an optional related message 
   * \returns true if send. */
  bool sendInfoAsCommentWithTime(const char * info, const char * msg);
  /**
  * Parse commands from the USB connection and implement those commands.
  * \param buf is string to send
  * The function is served by the main loop, when time allows. */
  void parse_and_execute_command(char *buf);
  /**
   * is USB active, i.e. got message at least every 4 seconds */
  bool usbActive()
  {
    return not silentUSB;
  };
  /**
   * send to USB cvhannel 
   * \param str is string to send
   * \param n is number of bytes to send
   * \param blocking if false, then send if space only, else don't return until send
   */
  inline bool usb_send_block(const char * str, int n, bool blocking)
  {
    return client_send_str(str, n, blocking);
  }
  
private:
  /**
   * handle next character from USB */
  void receivedCharFromUSB(uint8_t n);
  /**
   * Send version string to client */
  void sendStatusVersion();
  /**
   * send to USB cvhannel 
   * \param str is string to send
   * \param n is number of bytes to send
   * \param blocking if false, then send if space only, else don't return until send
   */
  bool client_send_str(const char * str, int n, bool blocking);
  //
private:
  int8_t localEcho;
  bool silentUSB;
  //
  bool silenceUSBauto = true; // manuel inhibit of USB silent timeout
  // should teensy echo commands to usb
  bool justSendPrompt = false;
  /**
   * usb command buffer space */
  static const int RX_BUF_SIZE = 200;
  char usbRxBuf[RX_BUF_SIZE];
  int usbRxBufCnt = 0;
  uint32_t rxStartHb = 0;
  bool usbRxBufOverflow = false;
  uint32_t usbTimeoutGotData = 0;
};

#endif
