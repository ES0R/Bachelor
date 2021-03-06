/***************************************************************************
 *   Copyright (C) 2020 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a teensy 3.2 and teensy 3.5
 *   intended for 31300 and 31301 Linear control 1 at DTU
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
 

#define REV_ID "$Id: command.cpp 1153 2020-10-03 06:44:18Z jcan $" 

#include <malloc.h>
#include <ADC.h>
#include "IntervalTimer.h"
#include "main.h"
#include "eeconfig.h"
#include "command.h"
#include "uesc.h"
#include "sensor.h"

float missionTime = 0.0; // system time in seconds
uint32_t timeAtMissionStart = 0;
const char * lfcr = "\n\r";
// bool pushToUSB = true;
bool logToUSB = false;
bool silentUSB = false;
bool silenceUSBauto = true; // manuel inhibit of USB silent timeout
// should teensy echo commands to usb
int8_t localEcho = 0;
bool   justSendPrompt = false;
int32_t motorCurrentMLowPass[2];
//
// int pushInterval = 0;
// int pushTimeLast = 0;
// int pushStatus = 0;
// line sensor full or reduced power (output is high power)
bool pinModeLed = OUTPUT;
//
// mission stop and start
bool button;
bool missionStart = false;
bool missionStop = false;
// bool sendStatusWhileRunning = false;
/**
 * usb command buffer space */
const int RX_BUF_SIZE = 200;
char usbRxBuf[RX_BUF_SIZE];
int usbRxBufCnt = 0;
bool usbRxBufOverflow = false;
bool moreMissionLines = false;
uint32_t usbTimeoutGotData = 0;

int * m1;
/**
 * write to I2C sensor (IMU) */
int writeSensor(uint8_t i2caddr, int reg, int data);
/**
 * send state as text to USB (direct readable) */
void stateToUsb();
void sendStatusSensor(int8_t type);
//void sendStatusLogging();
void sendStatusVersion();
/// eeprom functions
void eePromSaveStatus(uint8_t * configBuffer);
void eePromLoadStatus(const uint8_t * configBuffer);
///
///
/** Who is requesting data
 * -2 = none (push), -1=USB, 0..4 = wifi client */
int8_t requestingClient = -2;
// 
/// serial 3 variables
/// debug copy everyting to USB (to be used with putty)
bool echoSerial3ToUsb = false;
/// size of serial3 command buffer
const int MAX_SER3_BUFFER_CNT = 100;
/// buffer for commands from serial 3
char ser3buff[MAX_SER3_BUFFER_CNT];
/// number of valid characters in serial 3 buffer
int16_t ser3buffCnt = 0;

/**
 * Get SVN revision number */
uint16_t getRevisionNumber()
{
  const char * p1 = strstr(REV_ID, ".cpp");
  return strtol(&p1[4], NULL, 10) * 10 + REV_MINOR;
}

////////////////////////////////////////////

/**
 * Test all I2C adresses and print reply. */
// void testAddr(void)
// {
//   int ak;
//   const int MRL = 100;
//   char reply[MRL];
//   for (int i= 0; i < 0x7f; i++)
//   {
//     Wire.beginTransmission(i);
//     Wire.write(0);
//     ak = Wire.endTransmission(I2C_STOP,1000);
//     snprintf(reply, MRL, "addr test addr %d (%x) gave %d", i, i, ak);
//     usb_send_str(reply);
//   }
// }

// ////////////////////////////////////////

/**
 * Got a new character from USB channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
void receivedCharFromUSB(uint8_t n)
{ // got another character from usb host (command)
  if (n >= ' ')
  {
    usbRxBuf[usbRxBufCnt] = n;
    if (usbRxBufCnt < RX_BUF_SIZE - 1)
      usbRxBuf[++usbRxBufCnt] = '\0';
    else
    {
      usbRxBufOverflow = true;
      usbRxBufCnt = 0;
      usbRxBuf[usbRxBufCnt] = '\0';
    }
  }
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
        usbRxBuf[usbRxBufCnt] = '\0';
        parse_and_execute_command(usbRxBuf, -1);
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



void handleIncoming(uint32_t mainLoop)
{
  int n = 0, m;
  // get number of available chars in USB buffer
  m = usb_serial_available();
  if (m > 20)
    // limit to no more than 20 chars in one 1ms cycle
    m = 20;
  else if (m == 0 and (not silentUSB) and silenceUSBauto and hbTimerCnt - usbTimeoutGotData > 4000)
  { // no data for a long time - stop sending data to USB
//     usb_send_str("# USB silent - no activity\r\n");
    usbTimeoutGotData = hbTimerCnt;
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


void sendHartBeat()
{
  const int MRL = 35;
  char reply[MRL];
  snprintf(reply, MRL,  "hbt %.1f us\r\n",  float(sensor->controlUsedTime[4]) / sensor->c_us);
  usb_send_str(reply, false);
}

// void sendMotorCurrent()
// {
//   const int MRL = 64;
//   char reply[MRL];
//   snprintf(reply, MRL,"mca %g %g\r\n", motorCurrentA[0], motorCurrentA[1]);
//   usb_send_str(reply);
// }

void sendTimingInfo()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL,"vti %g\r\n", SAMPLETIME);
  usb_send_str(reply);
}


void sendStatusVersion()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "version %.1f\r\n", (float)getRevisionNumber() / 10.0);
  usb_send_str(reply);
//   snprintf(reply, MRL, "# version %d (%s)\r\n", getRevisionNumber(), REV_ID);
//   usb_send_str(reply);
  
}


// parse a user command and execute it, or print an error message
//
void parse_and_execute_command(char *buf, int8_t serChannel)
{ // first character is the port letter
//   const int MSL = 100;
//   char s[MSL];
//  char * p2;
  bool commandHandled = false;
  //
  if (strncmp(buf, "stop", 4) == 0)
  {
    missionStop = true;
//     remoteControlVel = 0;
//     remoteControlVelDif = 0;
    logToUSB = false;
    commandHandled = true;
  }
  else if (strncmp(buf, "halt", 4) == 0)
  { // stop all 12V power (or turn on if 12 V power is off (and switch is on))
    if (buf[4] >= ' ')
      batteryHalt = strtol(&buf[5], NULL, 10);
    else
      batteryHalt = true;
    commandHandled = true;
  }
  else if (logToUSB)
    // while sending log to client, no other cammands are valid
    return;
  // the rest is
  // commands with a simple reply, so
  // requesting client is needed
  requestingClient = serChannel;
  if (not commandHandled)
  { // all other commands
    if (buf[1] == '=')
    { // one character assignment command
      switch (buf[0])
      {
        case 'i':  // interactive mode -1=no info, 0=normal GUI, 1=telnet session (local echo of all)
          localEcho = strtol(&buf[2], NULL, 10);
          break;
        default: 
          usb_send_str("#** unknown one character command!\r\n");
          break;
      }
    }
    else if (strncmp(&buf[0], "rc=", 3) == 0)
    { // remote control 
      // not implemented yet
    }
    else if (strncmp(buf, "help", 4) == 0)
    {
      const int MRL = 320;
      char reply[MRL];
      snprintf(reply, MRL, "# RegBot commands (" REV_ID ")\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   i=1         Interactive: 0: GUI info, 1: use local echo of all commands, -1:no info  (i=%d)\r\n", localEcho);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   eeW          Get configuration as string\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   eer          Read configuration from EE-Prom\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   start        start mission now\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   stop         terminate mission now\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   alive        Alive command: 'hbt <ctrl cycle time>'");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   esc N P V  Set esc N=1..%d P (position):0..1024, V (velocity): 0=full, else values/per s\r\n"
                           "#                (status: N=1..3 [enabled %d,%d,%d, P= %d, %d, %d, V= %d, %d, %d)\r\n", 
               esc->MAX_ESC_CNT,
               esc->escEnabled[0], esc->escEnabled[1], esc->escEnabled[2],
               int(esc->escValue[0]/100), int(esc->escValue[1]/100), int(esc->escValue[2]/100),
               esc->escVel[0], esc->escVel[2], esc->escVel[2] 
      );
      usb_send_str(reply);
      snprintf(reply, MRL, "#   pwmfrq frq   Set PWM frequency (50..450 Hz)\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   esi          Get esc status (esd [enabled value velocity]x6 )\r\n");
      usb_send_str(reply);
//       snprintf(reply, MRL, "#   esd s1 p1 v1 s2 p2 v2 s3 p3 v3 s4 p4 v4 s5 p5 v5  Set esc status sx=enable px=position (-1000..1000), vx velocity 0 (full) 1..10 (slower)\r\n");
//       usb_send_str(reply);
      snprintf(reply, MRL, "#   silent=1     Should USB be silent, if no communication (1=auto silent)\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   temp         send temperature info (tmp t1,t2,ad19,ad20).\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   time         send timing info (tim cycle, sense, ctrl, cycleuse, adc cycle, adc/cycle [us]).\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   batt         send battery voltage and current (bat volt,amp,ad17,ad18,ad6).\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   curDebug     send current AD and offset values.\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   sensor       send sensor data.\r\n");
      usb_send_str(reply);
//       snprintf(reply, MRL, "#   adctick      call adc tick (debug).\r\n");
//       usb_send_str(reply);
      snprintf(reply, MRL, "#   adc0         clear readings\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   log start M  start/clear log every m ms (>=0.1ms) (is started %d)\r\n", sensor->dataLog);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   log get      get log (idx=%d/%d)\r\n", dataIdx, MAX_DATA_CNT);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   version      %s.\r\n", REV_ID);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   help         This help text.\r\n");
      usb_send_str(reply);
    }
    else if (strncmp(buf, "eew", 3) == 0)
    { // initalize ee-prom
      eeConfig.setStringBuffer(NULL, true);
      // save to ee-prom
      usb_send_str("# saving to flash\r\n");
      eeConfig.eePromSaveStatus(false);
    }
    else if (strncmp(buf, "eeW", 3) == 0)
    { // save config to string buffer (and do not touch ee-prom)
      uint8_t buffer2k[2048];
      eeConfig.setStringBuffer(buffer2k, false);
      eeConfig.eePromSaveStatus(true);
      // send string-buffer to client
      eeConfig.stringConfigToUSB(NULL, 0);
      eeConfig.clearStringBuffer();
    }
    else if (strncmp(buf, "eer", 3) == 0)
    {  // load from flash to ccurrent configuration
      eeConfig.eePromLoadStatus(NULL);
    }
    else if (strncmp(buf, "start", 5) == 0)
    {
      missionStop = false;
      missionStart = true;
    }
    else if (strncmp(buf, "esi", 3) == 0)
    {
      esc->sendEscStatus();
    }
    else if (strncmp(buf, "esc ", 4) == 0)
    {
      esc->setOneEsc(&buf[4]);
    }
    else if (strncmp(buf, "pwmfrq ", 6) == 0)
    {
      esc->setPWMfrq(&buf[6]);
    }
    else if (strncmp(buf, "sensor ", 6) == 0)
    {
      sensor->sendSensorData();
    }
    else if (strncmp(buf, "temp ", 4) == 0)
    {
      sensor->sendTempData();
    }
    else if (strncmp(buf, "batt ", 3) == 0)
    {
      sensor->sendCurrentVolt();
    }
    else if (strncmp(buf, "seq ", 3) == 0)
    {
      sensor->startMotorLogSequence(buf);
    }
    // curDebug
    else if (strncmp(buf, "curDebug ", 3) == 0)
    {
      sensor->sendCurrentVals();
    }
    else if (strncmp(buf, "time", 4) == 0)
      sensor->sendTimingInfo();
    else if (strncmp(buf, "adctick ", 7) == 0)
    {
      sensor->adcTick();
    }
//     else if (strncmp(buf, "span ", 4) == 0)
//     {
//       if (strlen(buf) > 5)
//       {
//         int v = strtol(&buf[4],NULL, 0);
//         sensor->spanLimit = v;
//       }
//       else
//         usb_send_str("# missing parameter\r\n");
//     }
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
    { // log and more
      char * p1 = &buf[4];
      while (*p1 == ' ')
        p1++;
      if (strncmp(p1, "start", 4) == 0)
      {
        p1 += 5;
        float rate = strtof(p1, &p1);
        sensor->startLog(rate);
        const int MSL = 64;
        char s[MSL];
        snprintf(s, MSL, "#setting log interval to %g ms\r\n", rate);
        usb_send_str(s);
      }
      else if (strncmp(p1, "get", 3) == 0)
        sensor->getLog();
    }
    else if (strncmp(buf, "alive ", 5) == 0)
    {
//       usb_send_str("alive\n");
      sendHartBeat();
    }
    else if (strncmp(buf, "version ", 7) == 0)
    {
      const int MSL = 120;
      char s[MSL];
      snprintf(s, MSL, "ver %s\r\n", REV_ID);
      usb_send_str(s);
    }
  }
}

/////////////////////////////////////////////////

bool client_send_str(const char * str, bool blocking);

bool usb_send_str(const char* str, bool blocking)
{
  bool send;
  if (localEcho and justSendPrompt)
  {
    client_send_str("\n\r", blocking);
    justSendPrompt = false;
  }
  if (str[0] == '#')
  { // messages with a # is send to USB only (debug message)
    int8_t client = requestingClient;
    // do not use client as destination, use USB
    requestingClient = -1; // -1 is USB
    send = client_send_str(str, blocking);
    requestingClient = client;
  }
  else
    send = client_send_str(str, blocking);
  return send;
}

//////////////////////////////////////////////////

bool client_send_str(const char * str, bool blocking) //, bool toUSB, bool toWifi)
{
  int n = strlen(str);
  bool okSend = true;
  // a human client, so send all to USB and other clients
  if (not silentUSB)
  { // sending may give a timeout, and then the rest is not send!
    // this happends especially often on lenovo PCs, but they are especially slow in data transfer.
    if (not blocking)
      // surplus characters will be skipped
      usb_serial_write(str, n);
    else
    { // ensure all is send (like log data)
      int k = n, m;
      const char * p1 = str;
      uint32_t t0 = hbTimerCnt;
      while (k > 0 /*and usbWriteFullCnt < usbWriteFullLimit*/)
      {
        m = usb_serial_write_buffer_free();
        if (m > 0)
        {
          if (m > k)
            m = k;
          usb_serial_write(p1, m);
          k = k - m;
          if (k <= 0)
            break;
          p1 += m;
        }
        if (hbTimerCnt - t0 > 1500)
          // wait no more
          break;
      }
    }
  }
  return okSend;
}

////////////////////////////////////////////////////////////////

