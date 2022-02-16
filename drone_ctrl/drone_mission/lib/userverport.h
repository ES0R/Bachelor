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
#ifndef USERVERPORT_H
#define USERVERPORT_H

#include <pthread.h>
#include <arpa/inet.h>

#include "userverclient.h"
#include "urun.h"
#include <udata.h>

/**
Number of clients beeing served at one time */
#define MAX_SOCKET_CLIENTS    20


/**
Max length of attributes in namespace message */
#define MAX_NAMESPACE_ATTRIBUTE_LENGTH 200

class UDataBase;

/**
Class used by a socket server to control and accept connections to one port number.
The port number are determined during initialization.

@author Christian Andersen
*/
class UServerPort : public UData
{
public:
  /**
  Constructor */
  UServerPort(UDataBase * database, const char * port, int id);
  /**
  Destructor */
  virtual ~UServerPort();
  /**
   * Send string to client 
   * \param data is the string to be send
   * \param client is index to client list
   * \returns true if send (else client is assumed to have left the building */
  bool sendString(const char * data, int client);
  /**
   *  Set server port number */
//   void setHandler(UDataBase * messageHandler);
  /**
   *  Get number of  active clients */
  int getActiveClientCnt();
  /**
   * debug print status to console */
  void printStatus(int client) override;
  /**
   * Print status to string buffer
   * \param buffer is a string buffer
   * \param bufferSize is the buffer size, not to be exceeded */
  void printStatus(char * buffer, int bufferSize);
  /**
   * send a message to one of the connected clients */
  bool sendReplyFromHere(const char * message, int toClient) override;
  /**
   * send help */
  void sendHelp(const int client) override;
  
protected:

  /**
  Get server port number */
  inline int getPort() { return serverPort; };
  
  /**
  Get hostname - just a wrap of the normal 'gethostname' call.
  Returns a pointer to the provided string. */
  char * getHostName();
  /**
  The thread that handles all port tasks.
  Runs until stop-flag is set, or if port can not be bound.
  Should not be called!, call start() to create the thread
  calling this function. */
  void run();
  /**
  Print server status to console,
  preseded by this string (preStr). */
  void print(const char * preStr);
  /**
  Return number of initialized clients.
  The clients may be active or inactive (closed) */
  inline int getClientCnt() { return clientCnt; };
  /**
  Function to trigger events, when new data is available.
  Should be overwritten by decendent classes. */
//   virtual void messageReceived();
  /**
  This function is called, when a new client is connected */
  virtual void gotNewClient(UServerClient * cnn);
  /**
  Is server open for connections - i.e. port number is valid */
  bool isOpen4Connections()
  { return open4Connections; };
  /**
   * Alive call from server thread */
  void serverIsAlive()
  { serverAlive.now(); };
  /**
   * Get time passed since the server was last reported alive.
   * \returns time in seconds since that server thread last set the alive timestamp. */
  double serverAliveLast();
  /**
   * Set the allow connections flag
   * Existing connections are not closed
   * \param value set to true, if to allow new connections */
  void setAllowConnections(bool value)
  { allowConnections = value; };
  /**
   * Get the allow connections flag
   * if false, then server do not allow (new) connections
   * \param value set to true, if to allow connections */
  bool getAllowConnections()
  { return allowConnections; };
  /**
   * Get client number of latest clinet connected - mostly for debug */
  int getLastClient()
  { return lastClientNumber; };
  /**
   * Should be called whenever a client is disconnected - to allow cleanup of obligations for this client */
  void connectionLost(int client);
  /**
   * Get last used client serial number */
  inline int getLastClientSerial()
  { return lastClientSerial; };
  
  /**
   *  Get handle to connection - returns NULL if out of range */
  UServerClient * getClient(const int i);
  /**
   * update status string for subscribers */
  void updateStatusString();
  
  
public:
  static const int MAX_HOSTNAME_LEN = 100;
  char hostname[MAX_HOSTNAME_LEN];
  /**
   *  Is the selected port valid, i.e. open for connections */
  bool open4Connections;
  /**
   * server port number */
  int serverPort;
  /**
   * handle incoming command from a client */
  void handleCommand(const char * message, int clientNumber);
  
  
protected:
  /**
  Get a free client handle */
  int getFreeClientHandle();
  /**
  Service receive channel of all clients.
  Returns true is fata were processed.
  Returns false if nothing to process, either
  after poll-timeout or just no connections. */
  bool serviceClients(int msTimeout);
  /**
   * Should server allow connections, if not, then close server port - until changed. */
  bool allowConnections;

private:
  /**
  Running is true when accepting-calls thread is running */
  bool running;
  /**
  Structure for the accepted connections */
  UServerClient * client[MAX_SOCKET_CLIENTS];
  /**
  Last used client connection */
  int clientCnt;
  /**
  Number of active clients */
  int clientCntActive;
  /**
  Print more to info connection / console */
  bool verboseMessages;
  /**
  Statistics - receive loops */
  int recvLoops;
  /**
   * The last time the server thread was reported alive */
  UTime serverAlive;
  /**
   * client number to be used used for the last connection (-1 = no clients yet) */
  int lastClientNumber;
  /**
   * last client serial number used */
  int lastClientSerial;
  
  /**
   * message handler */
//   UHandler * handler;
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////



#endif
