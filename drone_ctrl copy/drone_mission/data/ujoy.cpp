/***************************************************************************
 *   Copyright (C) 2019 by DTU (Christian Andersen)                        *
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

#include "ujoy.h"
#include <udatabase.h>

UJoy::UJoy(UDataBase * data)
: UData(data)
{
  // add published keywords
  addKey("joy");
}

//
bool UJoy::decode(const char * msg, int source, const char * key, bool isCheckedOK)
{ 
  bool used = true;
  if (strcmp(key, "joy") == 0)
  {
    // joy 1 1 8 11 0 -2 -32767 0 -7513 -32767 0 0 0 0 0 0 0 0 0 0 0 0 0
    // 1 : joyRunning, 
    // 1 : manOverride, 
    // 8 : number_of_axes, 
    // 11 : number_of_buttons    
    // 0 -2 -32767 0 -7513 -32767 0 0 : axis values
    // 0 0 0 0 0 0 0 0 0 0 0 : button values (1=pressed)
    char * p1 = (char*)&msg[4];
    int n = strtol(p1, &p1, 10);
    if (n == 0)
      printf("UJoy::decode: No joystick / gamepad\n");
    manual = strtol(p1, &p1, 10) == 1;
    int a = strtol(p1, &p1, 10);
    int b = strtol(p1, &p1, 10);
    if (b > 11)
      b = 11;
    for (int i = 0; i < a; i++)
    {
      if (i < 8)
        axes[i] = strtol(p1, &p1, 10);
    }      
    for (int i = 0; i < b; i++)
    {
      if (i < 11)
      {
        button[i] = strtol(p1, &p1, 10);
        if (button[i])
        {
          printf("UJoy::decode: button %d pressed\n", i);
        }
      }
    }
    updated(msg, source, key);
    if (logIsOpen())
    {
      fprintf(logfile, "%ld.%03ld ", 
              dataTime.tv_sec, dataTime.tv_usec / 1000);
      for (int i = 0; i < 8; i++)
        fprintf(logfile, "%d ", axes[i]);
      for (int i = 0; i < 11; i++)
        fprintf(logfile, "%d ", button[i]);
      fprintf(logfile,"\n");
    }
  }
  else
    used = false;
  return used;
}

void UJoy::subscribeFromHW(bool fromTeensy)
{
  if (not fromTeensy)
  {
    db->sendToDataSource("joy subscribe 1\n", UDataBase::Teensy); // button and axis
    db->sendToDataSource("joy get\n", 1);          // get data now
  }
}

void UJoy::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  db->sendReply("# ------- JOY (gamepad) ----------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# manual control = %d\n", manual);
  db->sendReply(s, client);
  
  snprintf(s, MSL, "# axis      1 to 8 : ");
  int n = strlen(s);
  char * p1 = &s[n];
  for (int i = 0; i < 8; i++)
  {
    snprintf(p1, MSL -n - 2, "%d, ", axes[i]);
    n += strlen(p1);
    p1 = &s[n];
  }
  *p1++ = '\n';
  *p1++ = '\0';
  db->sendReply(s, client);
  //
  snprintf(s, MSL, "# buttons 1 to 11: ");
  n = strlen(s);
  p1 = &s[n];
  for (int i = 0; i < 11; i++)
  {
    snprintf(p1, MSL - n - 2, "%d, ", button[i]);
    n += strlen(p1);
    p1 = &s[n];
  }
  *p1++ = '\n';
  *p1++ = '\0';
  db->sendReply(s, client);
  snprintf(s, MSL, "# data age %.3fs\n", getTimeSinceUpdate());
  db->sendReply(s, client);
  printDataStatus(client);
}

/**
 * open logfile and print description */
void UJoy::openLog()
{
  UData::createLogfile("log_joy");
  if (logIsOpen())
  {
    fprintf(logfile, "%% robobot mission joystick (gamepad) log\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2-8 Axis position\n");
    fprintf(logfile, "%% 9-19 buttons pressed\n");
  }
}

void UJoy::sendHelp(int client)
{
  db->sendReply("# ------- Gamepad ----------\r\n", client);
  db->sendReply("# keywords: joy     Holds: buttons and axis\r\n", client);
}
