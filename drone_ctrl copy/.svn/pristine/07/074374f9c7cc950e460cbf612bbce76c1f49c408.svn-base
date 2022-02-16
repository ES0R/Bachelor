/**
 * Interface with Teensy
 *  
 * ************************************************************************
*   Copyright (C) 2017 by DTU (Christian Andersen)                        *
*   jca@elektro.dtu.dk                                                    *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Lesser General Public License as        *
*   published by the Free Software Foundation; either version 2 of the    *
*   License, or (at your option) any later version.                       *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU Lesser General Public License for more details.                   *
*                                                                         *
*   You should have received a copy of the GNU Lesser General Public      *
*   License along with this program; if not, write to the                 *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#include <sys/time.h>
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>

#include "uteensy.h"
#include <udatabase.h>
#include "ustate.h"

using namespace std;

/** constructor */
UTeensy::UTeensy(UDataBase * database, int id, const char device[])
: UData(database)
{
    connectionOpen = false;
    sourceID = id;
    if (strlen(device) > 0)
      usbdevice = device;
//     openLog();
    addKey("teensy");
}
/** destructor */
UTeensy::~UTeensy()
{ // stop all activity before close
  // is done in run()
  closeLog();
}

void UTeensy::openLog()
{
  createLogfile("log_teensy");
  if (logfile != NULL)
  {
    logLock();
    fprintf(logfile, "%% log of Teensy interface\n");
    fprintf(logfile, "%% 1 Linux timestamp (seconds since 1 jan 1970)\n");
    fprintf(logfile, "%% 2 mission time (sec)\n");
    fprintf(logfile, "%% 3 direction flag -> (outgoung) or <- (incoming)\n");
    fprintf(logfile, "%% 4 string send or received\n");
    doFlush();
    logUnlock();
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "%s connected\n", usbdevice);
    saveDataToLog(s, false);
  }
  printf("# Teensy module opened the logfile\n");
}


void UTeensy::send(const char * key, const char * params, int msTimeout)
{ // construct string and send to robot
  const int MSL = 500;
  char s[MSL];
  int n = strlen(key) + strlen(params) + 5;
  if (n > MSL)
  {
    printf("UTeensy::send: msg to robot too long (%d>%d) truncated\r\n", n, MSL);
    n = MSL-1;
    s[n] = '\0';
  }
  if (strlen(params) == 0)
    snprintf(s, n, "%s\r\n", key);
  else
    snprintf(s, n, "%s %s\r\n", key, params);
  send(s, msTimeout);
}

/**
  * send a string to the serial port */
void UTeensy::send(const char * cmd, int msTimeout)
{ // this function may be called by more than one thread
  // so make sure that only one send at any one time
//   int t = 0;
  sendMtx.lock();
  int m = strlen(cmd);
  if (connectionOpen and m > 0)
  { // debug
//     if (cmd[0] == ';' or cmd[0] == '#')
//     { // should never be to Teensy
//       printf("# sending this to Teensy: %s", cmd);
//     }
//     else
    {
      // debug end
      // try send string to USB
  //     usbio.write(cmd, n);
  //     if (not usbio.good())
      int n = write(usbport, cmd, m);
      if (n <= 0)
      {
        connectionOpen = false;
        perror("# UTeensy::send failed:");
  //       printf("# send failed, closeing device\n");
        saveDataToLog("# send failed\n", false);
      }
      else
      {
  //       printf("# UTeensy::send n=%d as(%s)\n", n, cmd);
        messageOutCnt++;
        saveDataToLog(cmd, false);
      }
    }
  }
  // 
  sendMtx.unlock();
  lastTxTime.now();
  usleep(3300); // Short sleep helps communication be more reliable when many command lines are send consecutively.
}

/** subscription data update */
void UTeensy::updateConnectionState()
{
  snprintf(raw[0]->raw, UPublish::MAX_MESSAGE_LENGTH,"teensy %d %.1f %d %d %d\n", connectionOpen, idlePct*100, messageInCnt, messageOutCnt, messageErrCnt);
  raw[0]->updated();
}

/**
  * receive thread */
void UTeensy::run()
{ // read thread for REGBOT messages
  int n = 0;
  rxCnt = 0;
  readIdleLoops = 0;
//   int again = 0;
  const int MBL = 2000;
  char bb[MBL+1] = {""};
  bb[MBL] = '\0';
  int bbIdx = 0;  // end of currently received
//   int bbIdx0 = 0; // start of new message
  while (not th1stop)
  {
    if (not connectionOpen)
    { // close if error
      if (usbport > 0)
      {
        close(usbport);
        usbport = -1;
      }
      // wait a second (or 2)
      sleep(2);
      // then try to connect
      openUsb();
      if (connectionOpen)
      { // request base data
        db->dataSourceOpened(true);
        idlePeriod.now();
        readIdleLoops = 0;
      }
      // make status available
      updateConnectionState();
      n = 0;
    }
    else
    { // from USB
      if (false)
        // block
        n = read(usbport, &bb[bbIdx], MBL - bbIdx);
      else
      { // one char
        n = read(usbport, &rx[rxCnt], 1);
      }
      if (n < 0 and (errno == EAGAIN or errno == EWOULDBLOCK))
      { // no data
        n = 0;
        again++;
      }
      else if (n < 0)
      { // other error - close connection
        perror("Teensy:: port error");
        saveDataToLog("Port error - closeing\n", true);
        usleep(100000);
        sendMtx.lock();
        // don't close while sending
        close(usbport);
        sendMtx.unlock();
        connectionOpen = false;
      }
      else if (n == 0)
        printf("Teensy:: read got 0 chars, but would not block?\n");
    }
    if (n > 0)
    { // old one-char at a time
      if (n == 1)
      {
        rxCnt++;
        rxChars++;
        if (rx[rxCnt-1] == '\n')
        { // terminate string
          rx[rxCnt] = '\0';
          // handle new line of data
          saveDataToLog(rx, true);
          rxChars = 0;
          decodeMsgFromTeensy(rx);
          // reset receive buffer
          rxCnt = 0;
          n = 0;
          again = 0;
          // make status available
          updateConnectionState();
//           usleep(50);
        }
      }
      else
        printf("# Teensy rx data error - more than 1 char??\n");
        
    }
    else if (n == 0)
    {  // no data, so wait a bit
      sendMtx.unlock();
      // may send in this timeslot
      usleep(1000);
      // wait until (potential) send complete
      sendMtx.lock();
      readIdleLoops++;
    }
  } // while
  if (usbport != -1)
  {
    close(usbport);
    usbport = -1;
  }
  connectionOpen = false;
  printf("Teensy:: thread stopped and port closed\n");
}  

/**
  * Open the connection.
  * \returns true if successful */
bool UTeensy::openUsb()
{ // should use USB port
  //   printf("Regbot::openToRegbot opening\n"); 
  usbport = open(usbdevice, O_RDWR); // | O_NDELAY);// | O_NOCTTY); // );
  if (usbport == -1)
  {
    if (connectErrCnt < 5)
    {
      perror(usbdevice);
      perror("Teensy::openUsb open_port: Unable to open teensy connection");
    }
    connectErrCnt++;
  }
  else
  { // set connection to non-blocking
    if (true)
    {
      int flags;
      if (-1 == (flags = fcntl(usbport, F_GETFL, 0)))
        flags = 0;
      fcntl(usbport, F_SETFL, flags | O_NONBLOCK);    
  // #ifdef armv7l
      struct termios options;
      tcgetattr(usbport, &options);
      options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //<Set baud rate
      options.c_iflag = IGNPAR;
      options.c_oflag = 0;
      options.c_lflag = 0;
      tcsetattr(usbport, TCSANOW, &options);
  // #endif
      tcflush(usbport, TCIFLUSH);
    }
    connectErrCnt = 0;
  }
  connectionOpen = usbport != -1;
  if (connectionOpen)
    printf("# Teensy opened on %s\n", usbdevice);
  return connectionOpen;
}

/////////////////////////////////////////////////

/**
  * decode messages from REGBOT */
void UTeensy::decodeMsgFromTeensy(char * msg)
{
  bool msgIsOK = true;
  // skip"#  leading whitespace
  //printf("# UTeensy::decodeMsgFromTeensy:%s", msg);
  char * p1 = strrchr(msg,';');
  if (p1 == nullptr)
    p1 = msg;
  if (p1 > msg)
    printf("# skipped %ld chars from '%s'\n", p1-msg, msg);
  char * p0 = p1;
//   while (*p1 != ';' and *p1 > '\0')
//     p1++;
  if (*p1 == ';')
  { // there is a CRC check (q-check) of 3 chars
    int q = 0;
    int sum = 0;
    if (isdigit(p1[1]) and isdigit(p1[2]))
    { // assumed valid
      q = strtol(&p1[1], nullptr, 10);
      const char *p2 = &p1[3];
      while (*p2 != '\0')
        sum += *p2++;
      msgIsOK = ((sum % 99) +1) == q and q > 0;
    }
    else
      printf("# q-number fail\n");
    if (msgIsOK)
    {
      p1+=3;
      db->decode(p1, sourceID, msgIsOK);
    }
    else
    {
      messageErrCnt++;
      printf("# Teensy msg error sum=%d != ;%d: (%s)\n", sum%99+1, q, p0);
    }
//     printf("# Teensy msg error q=%d != ;%d\n", sum%99+1, q);
  }
  messageInCnt++;
}

//////////////////////////////////////////////////

void UTeensy::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  db->sendReply("# ------ Teensy interface ------\n", client);
  sendKeyList(client);
  snprintf(s, MSL,"# Connected %d to %s\n", connectionOpen, usbdevice);
  db->sendReply(s, client);
  snprintf(s, MSL,"# rx: %d, tx: %d (%d error messages) idle %.1f%%\n", messageInCnt, messageOutCnt, messageErrCnt, idlePct*100);
  db->sendReply(s, client);
  snprintf(s, MSL,"# logfile active %d\n", logfile != nullptr);
  db->sendReply(s, client);
}


void UTeensy::saveDataToLog(const char * message, bool incoming)
{
  if (logIsOpen())
  {
//     printf("# === Teensy log save (in)\n");
    timeval t;
    gettimeofday(&t, NULL);
    float dt = getTimeDiff(t, db->state->bootTime);
    const char * direction = "->";
    if (incoming)
      direction = "<-";
    logLock();
    fprintf(logfile, "%lu.%03ld %.3f (again=%d, rxChars=%d) %s: %s", t.tv_sec, t.tv_usec/1000, dt, again, rxChars, direction, message);
    doFlush();
    logUnlock();
//     printf("# === Teensy log save (out)\n");
  }
}


void UTeensy::sendHelp(const int client)
{
  db->sendReply("# ------- Teensy ----------\r\n", client);
  sendKeyList(client);
  //db->sendReply("keywords: %s\r\n", client);
}

bool UTeensy::decode(const char* message, int source, const char * key, bool isCheckedOK)
{ // returns false if no open Teensy connection
  // then it is send to bridge instead (if available)
  bool used = false;
  if (source != UDataBase::Teensy)
  { // not an incoming message from Teensy
    if (strcmp(key, "teensy") == 0 or strcmp(key, "robot") == 0)
    {
      const char * p1 = &message[6];
      while (isspace(*p1))
        p1++;
      send(p1, 20);
      used = true;
//       printf("# send to teensy '%s'\n", p1);
    }
//     else
//       printf("# not send to teensy '%s'\n", message);
  }
  return used;
}

void UTeensy::tick()
{
  float dt = aliveTimer.getTimePassed();
  if (dt > 0.5)
  { // request status
    send("alive\n");
    aliveTimer.now();
  }
  // idle calculation
  dt = idlePeriod.getTimePassed();
  if (dt > 3.0)
  { // update every 3 second
    idlePeriod.now();
//     if (readIdleLoops > 0)
//     { // get any extra time, if we are in idle
//       idleTime += idleTimer.getTimePassed();
//     }
    if (connectionOpen)
      idlePct = float(readIdleLoops)/(dt * 1000); // only if connected
    else
      idlePct = 1.0;
//     idleTime = 0.0;
    readIdleLoops = 0;
    if (logIsOpen())
    {
      UTime t;
      t.now();
      logLock();
      fprintf(logfile, "%lu.%03ld %.3f idle: %.2f%%\n", t.getSec(), t.getMilisec(), dt, idlePct * 100);
      doFlush();
      logUnlock();
    }    
  }
  // service status subscribers
  for (int i = 0; i < rawCnt; i++)
    raw[i]->tick();
}
