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
#include <unistd.h>
#include "udata.h"
#include "upublish.h"
#include "udatabase.h"

void UData::addKey(const char* key)
{
  if (rawCnt < MAX_KEY_COUNT)
  {
    raw[rawCnt] = new UPublish(this, key);
    rawCnt++;
  }
  else
    printf("# too many keys (%d) for this data item (%s)\n", rawCnt, key);
}

void UData::addRaw(const char* message, const char * key)
{
  // if one datatype only, then key is not needed.
  if (rawCnt == 1)
  { // data type requires that subscribers must have all data
    raw[0]->save4publish(message);
    // service any immediate clients
    raw[0]->serviceSubscribersNow(dataID);
//     printf("# addRaw: got %d types in %s", rawCnt, message);
  }
  else
  { // more data types, need the key to determine
    const char * p2 = &message[strlen(key)];
    int d = -1;
    if (strlen(p2) > 1)
    { // there is more than the key, so find published entry
      for (int i = 0; i < rawCnt; i++)
      { // find entry for this data
        if (strcmp(raw[i]->key, key) == 0)
        {
          d = i;
          break;
        }
      }
      if (d >= 0)
      { // data type found
        raw[d]->save4publish(message);
        // service any immediate clients
        raw[d]->serviceSubscribersNow(dataID);
      }
      else
        printf("# error in saving raw data for (%s)\n", message);
    }
//     printf("# addRaw: got %d/%d types, looking for key=%s, p2=%s", d, rawCnt, key, p2);
  }
}

void UData::tick()
{
  for (int i = 0; i < rawCnt; i++)
    raw[i]->tick();
}

bool UData::subscribeFromThis(const char* params, const char * key, int client)
{
  bool found = false;
//  printf("# UData::subscribeFr: %d %s %d\n", rawCnt, key, client);
  for (int i = 0; i < rawCnt; i++)
  {
//    printf("# debug c %d/%d\n", i, rawCnt);
    if (strcmp(raw[i]->key, key) == 0)
    {
      found = true;
//      printf("# debug a %d\n", i);
//      usleep(100000);
      raw[i]->subscribe(params, client);
//      printf("# debug b %d \n", i);
//      usleep(100000);
      break;
    }
//    printf("# subscribe key=%s == %s, found=%d (%d/%d)\n", key, raw[i]->key, found, i, rawCnt);
  }
  return found;
}

void UData::printDataStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "# logfile active=%d, msg count=%d\n", logfile != NULL, updateCnt);
  db->sendReply(s, client);
  for (int i = 0; i < rawCnt; i++)
    raw[i]->printStatus(client);
}


//////////////////////////////////////////////////////////////////////

bool UData::logOpenClose(const char * params, const char* key)
{
  bool found = hasKey(key);
  if (found)
  {
    bool close = strncmp(params, "close", 5) == 0;
//     printf("# UData::logOpenClose: close=%d, key=%s\n", close, key);
    if (close)
      closeLog();
    else
      openLog();
  }
  return found;
}

bool UData::hasKey(const char* key)
{
  bool found = false;
  if (key != nullptr)
    for (int i = 0; i < rawCnt; i++)
    {
      if (strcmp(raw[i]->key, key) == 0)
      {
        found = true;
        break;
      }
    }
  return found;
}

void UData::sendKeyList(int client)
{
  const int MSL = 100;
  char s[MSL];
  //
  if (rawCnt > 0)
  {
    char * p1 = s;
    int n = 0;
    snprintf(p1, MSL-n, "# keys:");
    n = strlen(p1);
    p1 = &s[n];
    for (int i=0; i < rawCnt; i++)
    { // space separated
      *p1++ = ' ';
      n++;
      strncpy(p1, raw[i]->key, MSL-n-1);
      n += strlen(p1);
      p1 = &s[n];
    }
    // terminate string
    *p1++ = '\r';
    *p1 = '\n';
  }
  else
  {
    strncpy(s, "# No keys defined\r\n", MSL);
  }
  db->sendReply(s, client);
}
