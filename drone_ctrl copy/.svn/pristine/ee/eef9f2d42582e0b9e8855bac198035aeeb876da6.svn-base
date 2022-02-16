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

#include <string.h>
#include "upublish.h"
#include "udata.h"



void UPublish::save4publish(const char* message)
{
  strncpy(raw, message, MAX_MESSAGE_LENGTH);
  dataTime.now();
  // flag for all potential subscribers that there is new data
  updated();
}

void UPublish::saveToLog(FILE * logfile)
{
  if (logfile != NULL)
  {
    fprintf(logfile, "%ld.%03ld %s", dataTime.getSec(), dataTime.getMilisec(), raw);
  }
}

bool UPublish::sendToClient(int client)
{
  bool sendOK = true;
  if (raw[0] != '\0')
    sendOK = parent->forwardReply(raw, client);
  return sendOK;
}

bool UPublish::sendToClient(int client, double frametime)
{
  bool sendOK = true;
  sendOK = parent->forwardReply("# UPublish::sendToClient (should be overwritten)\n", client);
  return sendOK;
}

void UPublish::serviceSubscribersNow(int dataSource)
{ // For subscribers that need all messages, e.g. logfile or help
  // as the data has been updated, and subscribers for ALL updates (i.e. interval==0)
  // must be served
  for (int i = 0; i < subsCnt; i++)
  {
    if (subs[i]->active and subs[i]->client != dataSource and subs[i]->interval == 0)
    {
      bool sendOK = sendToClient(subs[i]->client);
      if (not sendOK)
        subs[i]->active = false;
      // don't send this update twice
      subs[i]->reload(sendOK);
      subs[i]->updated = false;
    }
  }
}

void UPublish::tick()
{ // service regulars
  for (int i = 0; i < subsCnt; i++)
  { // regular subscribers (that do not require ALL updates, i.e. interval != 0)
    if (subs[i]->timeToDeliverTick())
    {
//       printf("# UPublish::tick (%d), %s count=%d, active=%d\n", i, key, subs[i]->count, subs[i]->active);
      bool sendOK = false;
      if (subs[i]->updated or subs[i]->count == 1)
      {
        if (parent->dataID != subs[i]->client)
        { // don't send data back to source
          sendOK = sendToClient(subs[i]->client);
          if (not sendOK)
            subs[i]->active = false;
//           printf("#    UPublish::tick send %d, active=%d\n", subs[i]->count, subs[i]->active);
        }
        // prepare for next
        subs[i]->updated = false;
      }
      subs[i]->reload(sendOK);
    }
  }
}

void UPublish::subscribe(const char * params, int client)
{ // find start of keyword
  // msg is:
  // sub <key> [A [B]]
  // params is string with A B
  // where A is number of replies (-1 is no end, 0=stop, 1 is just once, 2 is twice ... (default = 1)
  //   and B is interval between updates in ticks (default = 100 ticks) 1 second
  //
  const char * p1, *p2 = params;
  while (not isdigit(*p2) and *p2 != '\0' and *p2 != '-')
    p2++;
  // get subscription parameters A and B
  int count = 1;
  int interval = 100;
//   printf("# UPublish::subscribe *p2 = %x (%c), params='%s'\n", *p2, *p2, params);
  if (*p2 != '\0')
  {
    count = strtol(p2, (char**)&p1, 10); // A;
    interval = strtol(p1, (char**)&p2, 10); // B
    if (p2 == p1)
    { // last parameter is missing
      // default is once every second
      interval = 100;
    }
  }
  // printf("# UPublish::subscribe count=%d interval=%d\n", count, interval);
  // find any current subscription
  bool found = false;
  USubscription * sub = nullptr;
  for (int i = 0; i < subsCnt; i++)
  {
    sub = subs[i];
    found =  sub->client == client;
  }
  if (found)
  { // found item for this client
    sub->interval = interval;
    sub->updated = true;
    sub->reload(false);
    sub->active = count != 0;
    sub->count = count;
  }
  else
  { // new client - add an entry
    for (int i = 0; i < MAX_SUBSCRIBERS; i++)
    {
      sub = subs[i];
      if (sub == nullptr)
      {
        sub = new USubscription();
        subs[i] = sub;
        subsCnt = i + 1;
        break;
      }
      else if (not sub->active)
        break;
    }
    if (sub != nullptr and not sub->active)
    { // usable
      sub->client = client;
      sub->interval = interval;
      sub->reload(false);
      sub->count = count;
      sub->active = count != 0;
      sub->updated = true;
    }
    else
      printf("# No more subscriptions possible (has %d/%d) for this data (%s %s)\n", subsCnt, MAX_SUBSCRIBERS, key, params);
  }
}

void UPublish::printStatus(int client)
{
  for (int i = 0; i < subsCnt; i++)
  {
    if (subs[i]->active)
      subs[i]->printStatus(parent->db, client);
  }
}

void UPublish::updated()
{
  dataTime.now();
  for (int i = 0; i < subsCnt; i++)
  {
    subs[i]->updated = true;
  }
}


// ////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////

bool USubscription::timeToDeliverTick()
{
  if (active and interval > 0)
  {
    countDown--;
  }
  return countDown == 0 and active and interval > 0;
}

void USubscription::printStatus(UDataBase * db, int client)
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "#    client %d, flag %d, interval %d ticks\n", client, count, interval);
}
