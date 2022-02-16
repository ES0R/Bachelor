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
 

#define REV_ID "$Id: command.cpp 1342 2021-09-23 12:12:55Z jcan $" 

#include <malloc.h>
#include <ADC.h>
#include "IntervalTimer.h"
#include "main.h"
#include "eeconfig.h"
#include "command.h"
#include "uesc.h"
#include "sensor.h"
#include "upropshield.h"
#include "logger.h"
#include "usbus.h"
#include "control.h"
#include "usubscribe.h"
#include "ustate.h"
#include "mixer.h"
#include "uled.h"
#include "ultrasound.h"
#include "uheight.h"
#include "ulaser.h"


/**
 * Get SVN revision number */
uint16_t UCommand::getRevisionNumber()
{
  const char * p1 = strstr(REV_ID, ".cpp");
  return strtol(&p1[4], NULL, 10) * 10 + REV_MINOR;
}

////////////////////////////////////////////

/**
 * Got a new character from USB channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
void UCommand::receivedCharFromUSB(uint8_t n)
{ // got another character from usb host (command)
  if (usbRxBufCnt == 0)
    // got first char in new message
    rxStartHb = hbTimerCnt;
  if (n >= ' ')
  {
    usbRxBuf[usbRxBufCnt] = n;
    if (usbRxBufCnt < RX_BUF_SIZE - 1)
      usbRxBufCnt++;
    else
    {
      usbRxBufOverflow = true;
      usbRxBufCnt = 0;
      usbRxBuf[usbRxBufCnt] = '\0';
    }
  }
  //
  if (localEcho == 1 and not silentUSB)
    // echo characters back to terminal
    usb_serial_putchar(n);
  if (n == '\n' or n=='\r')
  { // zero terminate
    if (usbRxBufOverflow)
    {
      usbRxBufOverflow = false;
      usb_send_str("# USB rx-buffer overflow\n");
    }
    else
    {
      if (usbRxBufCnt > 0)
      {
        if (logger->logUSB and logger->isLogging())
        { // add string with newline to USB log
          usbRxBuf[usbRxBufCnt] = '\n'; // to make log readable
          logger->addUSBLogEntry(usbRxBuf, usbRxBufCnt + 1, rxStartHb, -1);
        }
        usbRxBuf[usbRxBufCnt] = '\0';
        parse_and_execute_command(usbRxBuf);
      }
      if (localEcho == 1)
      {
        usb_send_str("\r\n>>");
        justSendPrompt = true;
      }
    }
    // flush remaining input
    usbRxBufCnt = 0;
  }
  else if (usbRxBufCnt >= RX_BUF_SIZE - 1)
  { // garbage in buffer, just discard
    usbRxBuf[usbRxBufCnt] = 0;
    const char * msg = "** Discarded (missing \\n)\n";
    usb_send_str(msg);
    usbRxBufCnt = 0;
  }
}



void UCommand::handleIncoming()
{
  int n = 0, m;
  // get number of available chars in USB buffer
  m = usb_serial_available();
  if (m > 20)
    // limit to no more than 20 chars in one 1ms cycle
    m = 20;
  // next 4 lines, if Teensy should stop communication if no alive from USB
  else if (m == 0 and (not silentUSB) and silenceUSBauto and hbTimerCnt - usbTimeoutGotData > 4000)
  { // no data for a long time - stop sending data to USB
    usbTimeoutGotData = hbTimerCnt;
    if (localEcho == 0)
    {
      usb_send_str("# No activity, stopping USB communication (i=2 to continue)\r\n");
    }
    silentUSB = true;
  }
  // 
  if (m > 0)
  { // get characters
    for (int i = 0; i < m; i++)
    { // get pending characters
      n = usb_serial_getchar();
      if (n < 0)
        break;
      if (n >= '\n' and n < 0x80)
      { // there is data from USB, so it is active
        if (silentUSB and (n == '\n' or n == '\r'))
        { // open for data
          usb_send_str("# USB activated\r\n");
        }
        silentUSB = false;
        usbTimeoutGotData = hbTimerCnt;
        // command arriving from USB
        //usb_send_str("#got a char from USB\r\n");
        receivedCharFromUSB(n) ;
        break;
      }
    }
  }
}

//////////////////////////////////////////


void UCommand::sendStatusVersion()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "version %.1f\r\n", (float)getRevisionNumber() / 10.0);
  usb_send_str(reply);
}


// parse a user command and execute it, or print an error message
//
void UCommand::parse_and_execute_command(char * buf)
{ // command may be preceded by 'robot' or 'teensy'
  if (strncmp(buf, "robot ", 6) == 0)
  {
    buf += 6; // move pointer 7 characters forward to discard
    usb_send_str("# discarding the 'robot' part\n");
    while (*buf == ' ')
      buf++;
  }
  else if (strncmp(buf, "teensy ", 7) == 0)
  {
    buf += 7; // move pointer 7 characters forward to discard
    while (*buf == ' ')
      buf++;
    usb_send_str("# discarding the 'teensy' part\n");
  }
  // check if commands are handled by someone (most urgent first)
  if (control->decode(buf)) {} // a control issue - decoded there, handles ref, limit, mix
  else if (rc->decode(buf)) {} // a RC issue - decoded there, handled: rci, rcos
  else if (state->decode(buf)) {} // handled: arm, stop, bypass
  else if (mixer->decode(buf)) {}     // handles mix
  else if (subscribe->decode(buf)) {} // handles 'sub'
  else if (leds->decode(buf)) {} // handles 'led and leds'
  else if (uhgt->decode(buf)) {} // handles 'uht' (sonar)
  else if (uhlas->decode(buf)) {} // handles lass, lasl and laser (laser height)
  else if (hgt->decode(buf)) {} // handles 'hgt' (combined height), altfilt
  else if (sensor->decode(buf)) {} // handles sensor, temp, batt, seq, amps, time, adctick
  else if (imu->decode(buf)) {} // handles usemag, alt, test, offsetcal, imu, imuzero, imucal, imuraw
  else if (eeConfig->decode(buf)) {} // handles eew and eer
  else if (esc->decode(buf)) {} // decodes esc and esi
  // commands handled here, e.g. help
  else if (strncmp(buf, "help", 4) == 0)
  {
    const int MRL = 320;
    char reply[MRL];
    snprintf(reply, MRL, "# ------ Commands available for %s ------- \r\n", state->droneName);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   i=V          Interactive: V=0: GUI info (require activity), V=1: local echo, V=2:no timeout (i=%d)\r\n", localEcho);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   silent=1     Should USB be silent, if no communication (1=auto silent)\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   version      %s.\r\n", REV_ID);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   help         This help text.\r\n");
    usb_send_str(reply);
    // more specific help
    imu->sendHelp();
    sensor->sendHelp();
    logger->sendHelp();
    control->sendHelp();
    mixer->sendHelp();
    rc->sendHelp();
    state->sendHelp();
    subscribe->sendHelp();
    leds->sendHelp();
    uhgt->sendHelp();
    uhlas->sendHelp();
    hgt->sendHelp();
    eeConfig->sendHelp();
    //
  }
  else if (buf[1] == '=')
  { // one character assignment command
    switch (buf[0])
    {
      case 'i':  // interactive mode -1=no info, 0=normal GUI, 1=telnet session (local echo of all)
        localEcho = strtol(&buf[2], NULL, 10);
        break;
      default: 
        sendInfoAsCommentWithTime(" ** unknown one character command!", buf);
        break;
    }
  }
  else if (strncmp(buf, "silent", 6) == 0)
  {
    char * p2 = strchr(&buf[4],'=');
    int v = 1;
    if (p2 != NULL)
    {
      p2++;
      v = strtol(p2, NULL, 10);
    }
    silenceUSBauto = v != 0;
    silentUSB = false;
  }
  else if (strncmp(buf, "log ", 4) == 0)
  { // log and sub-commands
    logger->decode(&buf[4]);
  }
  else if (strncmp(buf, "version ", 7) == 0)
  {
    const int MSL = 120;
    char s[MSL];
    snprintf(s, MSL, "ver %s\r\n", REV_ID);
    usb_send_str(s);
  }
  else
    sendInfoAsCommentWithTime("unhandled", buf);
}

/////////////////////////////////////////////////

// bool client_send_str(const char * str, bool blocking);

bool UCommand::usb_send_str(const char* str, bool blocking)
{
  bool send;
  if (localEcho == 1 and justSendPrompt)
  {
    client_send_str("\n\r", 2, blocking);
    justSendPrompt = false;
  }
  int n = strlen(str);
  send = client_send_str(str, n, blocking);
  return send;
}

bool UCommand::sendInfoAsCommentWithTime(const char* info, const char * msg)
{
  const int MSL = 400;
  char s[MSL];
  bool isOK = false;
  snprintf(s, MSL, "# %.3f %s: %s", float(hb10us) * SAMPLETIME, info, msg);
  isOK = usb_send_str(s, false);
  return isOK;
}


//////////////////////////////////////////////////

bool UCommand::client_send_str(const char * str, int m, bool blocking) //, bool toUSB, bool toWifi)
{
  //int n = strlen(str);
  bool okSend = true;
  if (true)
  { // 
    // generate q-code first
    int sum = 0;
    const char * p1 = str;
    for (int i = 0; i < m; i++)
      sum += *p1++;
    const int MQL = 4;
    char q[MQL];
    snprintf(q, MQL, ";%02d", (sum % 99) + 1);
    uint32_t pt = hbTimerCnt;
    usb_serial_write(q, 3);
    usb_serial_write(str, m);
    // usb_serial_flush_output();
    if (logger->isLogging() and logger->logUSB)
    {
      logger->addUSBLogEntry(q, 3, pt, true);
      logger->addUSBLogEntry(str, m, pt, false);
    }
  }
  else if (false)
  {
    // cutting all messages to max 64 bytes
    int n = m + 3; // 3 for q-mark
    if (n > 64)
      n = 64;
    const int MBL = 65;
    char b[MBL];
    // fill in the rest
    strncpy(&b[3], str, MBL-3);
    b[64] = '\0';
    b[63] = '\n';
    char c = b[3];
    const char * p1 = &b[3];
    // send q-code first
    int sum = 0;
    for (int i = 0; i < n - 3; i++)
      sum += *p1++;
//     const int MQL = 4;
//     char q[MQL];
    snprintf(b, 4, ";%02d", (sum % 99) + 1);
    b[3] = c;
    uint32_t pt = hbTimerCnt;
    // debug check
//     for (int i = 0; i < n; i++)
//     {
//       if (b[i] < '\n' or b[i] > 127)
//       {
//         const int MSL = 100;
//         char s[MSL];
//         snprintf(s, MSL, "# send %x as char %d\n", b[i], i);
//         client_send_str(s,strlen(s), false);
//       }
//     }
    // debug end
    usb_serial_write(b, n);
//     usb_serial_write(str, n);
//     usb_serial_putchar('\n');
    //     usb_serial_flush_output();
    if (logger->isLogging() and logger->logUSB)
    {
//       logger->addUSBLogEntry(q, 3, pt, 1);
      logger->addUSBLogEntry(b, n, pt, true);
    }
  }
  else
  {
    // a human client, so send all to USB and other clients
    int n = m;
    if (not silentUSB or localEcho > 0)
    { // ensure all is send (like log data)
      // or nothing if measurement data (blocking = false)
      uint32_t pt = hbTimerCnt;
      int m = usb_serial_write_buffer_free();
      if (m < n + 3 and not blocking)
      { // just skip this message
        const int MSL = 20;
        char s[MSL];
        if (n < 60)
          snprintf(s, MSL, "aa:%d<%d\n", m,n);
        else
          snprintf(s, MSL, "bb:%d<%d\n", m,n);
        int f = strlen(s);
        usb_serial_write(s, f);
        if (n < 60)
          // short messages are just dropped
          okSend = false;
      }
      if (okSend)
      { // should be send
        const char * p1 = str, *p2 = "~\n";
        // send q-code first
        int sum = 0;
        for (int i = 0; i < n; i++)
          sum += *p1++;
        const int MQL = 4;
        char q[MQL];
        snprintf(q, MQL, ";%02d", (sum % 99) + 1);
        usb_serial_write(q, 3);
        //
        int k = n;
        p1 = str;
        uint32_t t0 = hbTimerCnt;
        while (k > 0)
        { // not all send yet
          m = usb_serial_write_buffer_free();
          if (m > 0)
          {
            if (m > k)
              m = k;
            pt = hbTimerCnt;
            usb_serial_write(p1, m);
            usb_serial_flush_output();
            k = k - m;
            if (k <= 0)
              break; // all send
//             else
//             { // debug
//               okSend = false;
//               break;
//             } // debug end
            p1 += m;
          }
          if (hbTimerCnt - t0 > 10) // each is a sample time (2.5ms)
          { // wait no more 
            okSend = false;
            // send a give up (abort) code
            usb_serial_write(p2, 2);
            break;
          }
        }
        if (logger->isLogging() and logger->logUSB)
        { // save all parts to log
          logger->addUSBLogEntry(q, 3, pt, 1);
          logger->addUSBLogEntry(str, n-k, pt, false);
          if (not okSend)
            // add truncation char and line-feed
            logger->addUSBLogEntry(p2, 2, pt, false);
        }
      }
    }
  }
  return okSend;
}

////////////////////////////////////////////////////////////////

