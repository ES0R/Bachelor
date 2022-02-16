/***************************************************************************
*   Copyright (C) 2016-2020 by DTU (Christian Andersen)                        *
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
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>

#include "ubridge.h"
#include "ustate.h"
#include "udatabase.h"
#include "ubridge.h"

using namespace std;

//////////////////////////////////////////////////////////////////

/** constructor */
UBridge::UBridge(UDataBase * database, const char * server, int id)
: UData(database)
{
  sourceID = id; /// id of commands from this source
  db = database;
  th1stop = false;
  th1 = NULL;
  char url[MAX_NAME_LENGTH];
  strncpy(url, server, MAX_NAME_LENGTH);
  bool none = strncmp(server, "none", 4) == 0;
  if (none)
    printf("Bridge connection not used\n");
  else
  {
    char * p1 = strchr(url,':');
    if (p1 == nullptr)
    {
      p1 = (char*)"24001";
    }
    else
    { // replace ':' with a string termination
      // and advance to next character
      *p1++ = '\0';
    }
    snprintf(host, MAX_HOST_LENGTH, "%s:%s", url, p1);
  //   printf("UBridge:: opening socket %s to bridge\n", host);
    bool isOK = createSocket(p1, url);
    if (isOK)
    {
      tryConnect();
      if (connected)
      { // start data source thread
        run();
        // tell system
        db->dataSourceOpened(false);
      }
    }
  }
  addKey("bridge");
}

/////////////////////////////////////////////////////////

/** destructor */
UBridge::~UBridge()
{ // stop all activity before close
  printf("Bridge destructor\n");
  stop();
  
  closeLog();
}

/////////////////////////////////////////////////////////

void UBridge::openLog()
{
  createLogfile("log_bridge");
  if (logfile != NULL)
  {
    logLock();
    fprintf(logfile, "%% log of bridge interface\n");
    fprintf(logfile, "%% received from bridge (incoming) (->)\n");
    fprintf(logfile, "%% 1 Linux timestamp (seconds since 1 jan 1970)\n");
    fprintf(logfile, "%% 2 mission time (sec)\n");
    fprintf(logfile, "%% 3 direction flag -> (outgoung) or <- (incoming)\n");
    fprintf(logfile, "%% 4 string send or received\n");
//     fprintf(logfile, "%lu.%03ld %6.3f->: %s %d\n", t.tv_sec, t.tv_usec/1000, dt, "Connected to bridge ", connected);
//     doFlush();
    logUnlock();
    // save hostname to log
    saveDataToLog(host, false);
  }
}

/////////////////////////////////////////////////////////

/**
  * send a string to connected port */
bool UBridge::send(const char * cmd)
{ // this function may be called by more than one thread
  // so make sure that only one send at any one time
//   printf("UBridge::send 0\n");
  sendMtx.lock();
  int n = 0;
  if (connected)
  { // send data
    n = sendData(cmd);
    saveDataToLog(cmd, false);
    msgOutCnt++;
  }
  // A short sleep helps communication be more reliable when many command lines are send consecutively.
  usleep(100);
  sendMtx.unlock();
  return n > 0;
}

/////////////////////////////////////////////////////////

/**
  * receive thread */
void UBridge::run()
{ // read thread for REGBOT messages
  int n = 0;
  int msgCnt = 0;
  int msgCntSec =0;
  timeval idleTime;
  gettimeofday(&idleTime, NULL);
  bool sockErr = false;
  UTime t, tsec;
  t.Now();
  tsec = t + 1;
  int loopSec = 0;
  int sleepUs = 1000;
  // get robot name
  //send("u4\n");
  int loop = 0;
  // receive buffer
  static const int MAX_RX_CNT = 500;
  // rx buffer
  char rx[MAX_RX_CNT];
  // number of characters in rx buffer
  int rxCnt = 0;
  while (not th1stop)
  {
    char c;
    n = readChar(&c, &sockErr);
    if (n == 1)
    { // not an error or hangup
      if (c >=' ' or c == '\n')
      { // got a valid character
        rx[rxCnt] = c;
        if (rx[rxCnt] == '\n')
        { // terminate string
          rx[rxCnt] = '\0';
          // printf("#### Received: (loop=%d, rxCnt=%d) '%s'\n",loop, rxCnt, rx);
          decode(rx);
          rxCnt = 0;
          msgCnt++;
          // clear buffer for old message
          memset(rx, '\0', MAX_RX_CNT);
          n = 0;
          gettimeofday(&idleTime, NULL);
        }
        else if (rxCnt < MAX_RX_CNT)
        { // prepare for next byte
          rxCnt++;
          continue;
        }
        else
        { // buffer overflow
          printf("UBridge::run: receiver overflow\n");
          rxCnt = 0;
        }
      }
    }
    else if (sockErr)
    {
      perror("UBridge:: port error");
      usleep(100000);
    }
    // go idle a bit
    usleep(sleepUs);
    loop++;
    t.now();
    if (t > tsec)
    {
      int loops = loop - loopSec;
      loopSec = loop;
      int sleptUs = loops * sleepUs;
      bridgeLoad = (1000000 - sleptUs) * 100.0 / 1000000.0;
      msgCnt1sec = msgCnt - msgCntSec;
      msgCntSec = msgCnt;
//       printf("# bridge load = %.1f %% (loops = %d, sleepUs=%d, msg cnt=%d/sec)\n", info->bridgeLoad, loops, sleepUs, info->msgCnt1sec);
      tsec += 1;
    }
  }
  printf("UBridge:: thread stopped\n");
}  

/////////////////////////////////////////////////////////

/**
  * decode messages just received from bridge */
void UBridge::decode(char * message)
{
  // skip whitespace
  while (*message <= ' ' and *message > '\0')
    message++;
  msgInCnt++;
  db->decode(message, sourceID, true);
  // communication log
  saveDataToLog(message, true);
}

//////////////////////////////////////////////////

void UBridge::sendHelp(int client)
{
    db->sendReply("# ------- Bridge ----------\r\n", client);
    db->sendReply("# bridge connect IP:port  Try connect to this IP and port\r\n", client);
    db->sendReply("# bridge message          Sends message to bridge, if connected\r\n", client);
}


void UBridge::printStatus(int client)
{
  const int MSL = 1200;
  char s[MSL];
  db->sendReply("# ------- Bridge ----------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# Connected=%d to %s\n", connected, host);
  db->sendReply(s, client);
  snprintf(s, MSL,"# logfile active=%d\n", logfile != NULL);
  db->sendReply(s, client);
  snprintf(s, MSL,"# load=%.0f %%, message rate %d/sec\n", bridgeLoad, msgCnt1sec);
  db->sendReply(s, client);
  snprintf(s, MSL,"# rx: %d, tx: %d\n", msgInCnt, msgOutCnt);
  db->sendReply(s, client);
  snprintf(s, MSL,"# logfile active %d\n", logfile != nullptr);
  db->sendReply(s, client);
}

void UBridge::saveDataToLog(const char * message, bool incoming)
{
  if (logIsOpen())
  {
    timeval t;
    gettimeofday(&t, NULL);
    float dt = getTimeDiff(t, db->state->bootTime);
    const char * direction = "->";
    if (incoming)
      direction = "<-";
    logLock();
    fprintf(logfile, "%lu.%03ld %.1f%%, %6.3f %s: %s\n", t.tv_sec, t.tv_usec/1000, bridgeLoad, dt, direction, message);
    doFlush();
    logUnlock();
  }
}

bool UBridge::decode(const char* msg, int sender, const char * key, bool isCheckedOK)
{
  bool used = true;
  if (strncmp(msg, "bridge", 6) == 0 and sender != UDataBase::Bridge)
  { // strip off the bridge keyword
    // if more bridges, then key can be repeated in message
    const char * p1 = &msg[6];
    while (isblank(*p1))
      p1++;
    if (*p1 != '\0')
    { // more data
      if (strncmp(p1, "connect ", 8) == 0)
        connectToBridge(&p1[8]);
      else
        send(p1);
    }
  }
  else
    used = false;
  return used;
}

void UBridge::connectToBridge(const char* host)
{
  const int MSL = 500;
  char s[MSL];
  strncpy(s, host, MSL);
  char * p2 = strchr(s, ':');
  if (p2 == nullptr)
  {
    
  }
}

