/***************************************************************************
 *   Copyright (C) 2019-2020 by DTU (Christian Andersen)                        *
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

#include <math.h>
#include <udatabase.h>
#include <cstring>
#include "uunhandled.h"


UUnhandled::UUnhandled(UDataBase * data)
: UData(data)
{
  // openLog();
  // add published keywords
  addKey("unhandled");
}

bool UUnhandled::decodeUnhandled(const char * msg, int source)
{ // use everything
  bool used = true;
  updated(msg, source, "unhandled");
  if (rawCnt > 0)
    raw[0]->saveToLog(logfile);
  return used;
}


void UUnhandled::printStatus(int client)
{
  db->sendReply("# ------- Unhandled messages ----------\n", client);
  sendKeyList(client);
  printDataStatus(client);
}

void UUnhandled::openLog()
{ // open pose log
  UData::createLogfile("log_unhandled");
  if (logfile != NULL)
  {
    fprintf(logfile, "%% robobot mission pose logfile\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 message\n");
  }
}


void UUnhandled::sendHelp(const int client)
{
  db->sendReply("# ------- Unhandled ----------\r\n", client);
  db->sendReply("# keywords: unhandled     Any unhandled message, subscribe to all (no filter)\r\n", client);
}
