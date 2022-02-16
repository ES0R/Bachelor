/***************************************************************************
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
#ifndef USERVERCLIENT_H
#define USERVERCLIENT_H

#include <arpa/inet.h>
#include <pthread.h>
#include <string.h>

#include "utime.h"


class UServerPort;

/**
Class of functions to handle one accepted connection for a socket server.

@author Christian Andersen
*/
class UServerClient
{
public:
  /**
  Constructor
  \param index is the client number of this client connection.
  \param parent port is a pointer to the port object where this client belong. */
  UServerClient(int index);
  /**
  Destructor */
  virtual  ~UServerClient();
  /**
   * Set message handler */
  void setHandler(UServerPort * messageHandler)
  {
    handler = messageHandler;
  }
  /**
  This is also the place to send any welcome messages.
  The virtual function is called before receive thread is activated.
  Must returns true if connection is acceptable. */
  virtual bool justConnected();
  /**
  returns true if connection is open. */
  bool isActive() {return connected;};
  /**
  Get connection stream handle - for use in poll, etc. */
  inline int getCnn() { return conn; };
  /**
  Send data to client.
  if connection to client is open the data is send.
  returns false if client do not exist or data could not be send
  within timeout period. */
  bool blockSend(const char * buffer, int length, int msTimeout);
  /**
   * send string to client - with a timeout if the connection is too slow or dead */
  bool sendString(const char * string, int msTimeout = 100)
  {
    int n = strlen(string);
    return blockSend(string, n, msTimeout);
  }
  /**
  Set connection handle (clnt) and the struct with the client info - name etc. */
  bool initConnection(int clientNumber, int clnt, struct sockaddr_in from,
                     UTime * aliveTime);
  /**
  Get IP number of client as string. */
  char * getClientName();
  /**
  Stop connection and send a HUP (hangup) if
  'sendHUP' is true. */
  void stopConnection(bool sendHUP);
  /**
  Data is available to receive, so
  receive the data and process as needed (i.e. put in queue) */
  bool receiveData();
  /**
  Lock communication to client.
  Returns false, if an error occured.
  Returns true when the client can be and is locked. */
  inline bool lock()
    { return (pthread_mutex_lock(&txlock) == 0); };
  /**
  Lock communication to client and return emmidiately.
  Returns false, if an error occured or the client is locked already.
  Returns true if the client is locked by the calling thread. */
  inline bool tryLock()
    { return (pthread_mutex_trylock(&txlock) == 0); };
  /**
  Unlock communication to client and return emmidiately. */
  inline void unlock()
    { pthread_mutex_unlock(&txlock); };
  /**
  Ressource is updated - action may be needed */
  void resourceUpdated();
  /**
  Open a logfile dataPath with the application name with client-number added,
  e.g. /mnt/ram/userver_client_1.log. */
  bool logOpen();
  /**
  Close logfile.
  Resets any flags used (e.g. log also reply) */
  void logClose();
  /**
   * Get serial number for the connection */
  inline int getSerial()
  { return clientSerial; };
  /**
   * Get pointer to time of last received message for this client */
  const UTime * getTimeOfLastMessage()
  { return &msgBufTime; };
  /**
   * Send message to client with alive time for for the main thread in this server */
  void sendAliveReply();
  /**
   * Get time since last message received or send (in seconds) */
  inline float getTimeSinceLast()
  {
    float t1 = msgBufTime.getTimePassed();
    float t2 = msgSendTime.getTimePassed();
    // printf("# time since rx = %f, tx = %f\n");
    if (t1 < t2)
      return t1;
    else
      return t2;
  }

protected:
  /**
  * Called when data or additional data is received from
  * socket port. message is in msgBuff with a length of msgBuffCnt. 
  * \param msg is the 0-terminated message line (but without \n)
  *            NB! message may be changed using strtok function
  */
  void gotNewMessage(char * msg);

private:
  /**
  Is a client connected */
  bool connected;
  /** Structure with adress information etc. */
  struct sockaddr_in clientInfo; // address info for connection
  /** Connection handle */
  int conn; // connection handle
  static const int MAX_MSG_LENGTH = 1000;
  /**
  Message buffer for partially received messages */
  char msgBuff[MAX_MSG_LENGTH];
  /**
  Already received part of message buffer */
  int msgBufCnt;
  /**
  Index number of client - set at creation */
  int clientIndex;
  /** lock, when reading or writing.
  This lock is locked, when a function is initiated and
  unlocked after the function returns.
  So this lock should not be tested or used inside a function. */
  pthread_mutex_t txlock; // pthread_mutex_lock
  /**
   * Server alive time.
   * The last time the server thread was reported alive.
   */
  UTime * serverAlive;
  /**
   * pointer to handler of all messaged from client */
  UServerPort * handler;

public:
  /**
  The client connection serial number - set at creation, and is unique for the session */
  int clientSerial;
  /**
   * connect time - set when connection was established */
  UTime connectTime;
  /**
   *  Time of last received data,
   *  used to empty part of buffer if out of byte sync. */
  UTime msgBufTime;
  /**
   * Time when last message was (sucessfully) send */
  UTime msgSendTime;
  /**
  Time of last punk beeing send (to test if client connection is alive) */
  UTime punkTime;
  /**
   *  statistics - messages send to this client */
  unsigned int msgSend;
  /**
   *  statistics - Number of received TCP/IP packeges */
  unsigned int tcpipReceived;
  /**
   *  statistics - Number of received messages */
  unsigned int msgReceived;
  /**
   *  Statistics - skipped bytes in seach of byte-sync */
  unsigned int msgSkippedBytes;
};

#endif
