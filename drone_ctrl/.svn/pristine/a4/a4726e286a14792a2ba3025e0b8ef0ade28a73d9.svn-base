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


#ifndef UDATA_H_INCLUDED
#define UDATA_H_INCLUDED

#include <sys/time.h>
#include <stdio.h>
#include <mutex>
#include "utime.h"
#include "urun.h"
#include "ulogfile.h"
#include "udatabase.h"


class UPublish;


/**
 * base class for all data items */
class UData : public URun, public ULogFile
{ // base class for data
public:
  timeval dataTime;
  /**
   * parent class */
  UDataBase * db = NULL;
  /** source of latest data received (decoded) */
  int dataID = -1;    // source of latest updated data
  /** source ID when this data element generates messages */
  int sourceID = -1;
  
public:
  /**
   * constructor */
  UData(UDataBase * database)
  {
    db = database;
  }
  /** destructor  */
  ~UData()
  { // make sure file is closed
    closeLog();
  }
  /**
   * open logfile - must be overwritten if log-file is used */
  virtual void openLog() {};
  // set update time
  inline void updated(const char * msg, int dataSource, const char * key)
  {
    dataID = dataSource;
    gettimeofday(&dataTime, NULL);
    addRaw(msg, key);
    updateCnt++;
  }
  /** open or close log from data with this key 
   * \return true if the key match. */
  virtual bool logOpenClose(const char * params, const char * key);
  // get time since update
  float getTimeSinceUpdate()
  {
    timeval t;
    gettimeofday(&t, NULL);
    return getTimeDiff(t, dataTime);
  }  
  /**
   * subscribe to source data */
  virtual void subscribeFromHW(bool fromTeensy) {};
  /**
   * subscribe to data in this data element */
  virtual bool subscribeFromThis(const char * message, const char * key, int client);
  /**
   * send message to a client from this data item.
   * Most data items has no contact with clients and will thus
   * just return false.
   * \param message to client
   * \param toClient is the destination client number
   * \returns false if client is not serviced by this data object. */
  virtual bool sendReplyFromHere(const char * message, int toClient)
  {
    return false;
  };
  /**
   * print status */
  virtual void printStatus(int client) 
  {
    printf("--no status--\n");
  }
  /**
   * print status for base elements */
  void printDataStatus(int client);
  /**
   * send on-line command help to a client.
   * help text is send if there is a key-match or key==nullptr.
   * \param client is the client ID
   * \param key is specific key, if help for one key only, else nullptr.
   * \returns true there is a key-match, if no key-match or key=nullptr, then returns false.*/
  bool sendHelp(const int client, const char * key)
  {
    bool found = hasKey(key);
    if (key == nullptr or found)
      sendHelp(client);
    return found;
  };
  /**
   * send on-line command help to a client.
   * \param client is the client ID */
  virtual void sendHelp(const int client) 
  {
    printf("# - help not defined for this datatype\n");
  };
  /**
   * decode an incoming message that might be consumed by this data object */
  virtual bool decode(const char * message, int source, const char * key, 
                      bool isCheckedOK) 
  {
    return false;
  };
  /** 
   * Check for regular updates and subscriptions */
  virtual void tick();
  /**
   * A message needs to be forwarded to a client.
   * mostly reply to a data subscription
   * \param message to forward 
   * \param client destination client. 
   * \returns true if send (else the client may have left) */
  inline bool forwardReply(const char * message, int client)
  {
    return db->sendReply(message, client);
  }
  /**
   * \returns key from this item, or NULL if no such item */
  const char * getKey(int i)
  {
    char * key = NULL;
    if (i < rawCnt and i >= 0)
      key = raw[i]->key;
    return key;
  }
  /**
   * Check if this data object has this key */
  bool hasKey(const char * key);

protected:
  /** Add keyword handled by this data element 
   * saved with raw version of used data */
  void addKey(const char * key);
  /**
   * send list of keys to this client */
  void sendKeyList(int client);
  
protected:
  /**
   * storage for received data elements */
  static const int MAX_KEY_COUNT = 4;    // max number of raw message from handled keys
  UPublish * raw[MAX_KEY_COUNT];
  int  rawCnt = 0;          // used entries  
  /** add raw string to buffer for subscribtion by clients
   *  add only if there is parameters, i.e. a space after the keyword.
   *  */
  void addRaw(const char * message, const char * key);
  /**
   * Count of updates */
  int updateCnt = 0;
};




#endif // UDATA_H_INCLUDED
