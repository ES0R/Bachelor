/***************************************************************************
 *   Copyright (C) 2016-2021 by DTU (Christian Andersen)                        *
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


#ifndef UPUBLISH_H_INCLUDED
#define UPUBLISH_H_INCLUDED

#include <sys/time.h>
#include <stdio.h>
#include <mutex>
#include <string.h>
#include "utime.h"

class UData;
class USubscription;
class UDataBase;

class UPublish
{
public:
  static const int MAX_KEY_LENGTH = 10;       // max length of key
  static const int MAX_MESSAGE_LENGTH = 2000; // max length of message
  
  UPublish(UData * data, const char * keyword)
  {
    strncpy(key, keyword, MAX_KEY_LENGTH);
    parent = data;
    snprintf(raw, MAX_MESSAGE_LENGTH,"%s (no data yet)\n", key);
  }
  /// parent data
  UData * parent;
  // raw copy of received data
  char raw[MAX_MESSAGE_LENGTH] = {0}; // message
  char key[MAX_KEY_LENGTH] = {0}; // key
  // update time
  UTime dataTime; // keys
  /// subscribers to this data
  static const int MAX_SUBSCRIBERS = 10;
  USubscription * subs[MAX_SUBSCRIBERS];
  int subsCnt = 0;
  /**
   * update subscription actions */
  void tick();
  /** save this raw message */
  void save4publish(const char * message);
  /**
   * save raw data to log with timestamp */
  void saveToLog(FILE * logfile);
  /**
   * subscribe to published data */
  void subscribe(const char * msg, int client);
  /**
   * Send data in this item to client
   * as subscribed */
  virtual bool sendToClient(int client);
  /**
   * special version for optitrack, where reply is generated on the fly with this frametime */
  virtual bool sendToClient(int client, double frametime);
  /**
   * service all subscribers now, can not wait, as data bay be overwritten 
   * usefull for e.g. logfile data and help messages
   */
  void serviceSubscribersNow(int dataSource);
  /**
   * print status to client for publish subscribers. */
  void printStatus(int client);
  /**
   * tell subscriptions that data is updated */
  void updated();
};


class USubscription
{
public:
  /// destination for subscription
  int client;
  /// how many times, -1 = no end, 0=stop, 1=just once, 2..=twice or more, then stop
  int count;
  /// how many ticks between update, 0=all and immediate delivery
  int interval; 
  /// active flag (else ready for reuse)
  bool active = false;
  /// data is updated
  bool updated = false;
  
private:
  /// counting down counter for subscriptions 
  int countDown;
public:
  /**
   * should data be delivered on this subscription
   * should count down
   * Returns true if time to send data */
  bool timeToDeliverTick();
  /**
   * prints status for this subscriber */
  void printStatus(UDataBase * db, int client);
  /**
   * Reload subscription 
   * but do not decrease counter if not send */
  void reload(bool sendOK)
  {
    if (count > 0 and sendOK)
    {
      count--;
      if (count == 0)
        active = false;
    }
    countDown = interval;
  }
};

#endif // UPUBLISH_H_INCLUDED
