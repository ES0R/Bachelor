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

#include <udatabase.h>
#include <linux/joystick.h>
#include "ugamepad.h"

UGamepad::UGamepad(UDataBase * data)
: UData(data)
{
  // add published keywords
  addKey("gamepad");
}

//
// bool UGamepad::decode(const char * msg, int source, const char * key, bool isCheckedOK)
// { 
//   return false;
// }


void UGamepad::updateData()
{
  char * s = raw[0]->raw; 
  const int max = UPublish::MAX_MESSAGE_LENGTH;
  char * p1 = s;
  int n = 0;
  snprintf(s, max, "joy %d 0 %d %d", connected, number_of_axes, number_of_buttons);
  for (int i = 0; i < number_of_axes; i++)
  {
    n += strlen(p1);
    p1 = &s[n];
    snprintf(p1, max - n, " %d", joyValues.axes[i]);
  }
  for (int i = 0; i < number_of_buttons; i++)
  {
    n += strlen(p1);
    p1 = &s[n];
    snprintf(p1, max - n, " %d", joyValues.button[i]);
  }
  // add newline
  n += strlen(p1);
  p1 = &s[n];
  snprintf(p1, max - n, "\r\n");
  // mark as updated
  raw[0]->updated();
  updateCnt++;
  // printf("UGamepad:: update %d \n", updateCnt);
}


void UGamepad::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  db->sendReply("# ------- Gamepad ----------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# connected = %d\n", connected);
  db->sendReply(s, client);
  if (connected)
  {
    int n = 0;
    char * p1 = s;
    snprintf(p1, MSL, "# %d axis:", number_of_axes);
    for (int i = 0; i < number_of_axes; i++)
    {
      n += strlen(p1);
      p1 = &s[n];
      snprintf(p1, MSL - n, " %d", joyValues.axes[i]);
    }
    // add newline
    n += strlen(p1);
    p1 = &s[n];
    snprintf(p1, MSL - n, "\r\n");
    db->sendReply(s, client);
    // and buttons
    p1 = s;
    n = 0;
    snprintf(p1, MSL, "# %d buttons:", number_of_buttons);
    for (int i = 0; i < number_of_buttons; i++)
    {
      n += strlen(p1);
      p1 = &s[n];
      snprintf(p1, MSL - n, " %d", joyValues.button[i]);
    }
    // add newline
    n += strlen(p1);
    p1 = &s[n];
    snprintf(p1, MSL - n, "\r\n");
    db->sendReply(s, client);
  }
  // also the standard items
  printDataStatus(client);
}

// void UGamepad::tick()
// { // get time since last update
// //   float dt = raw[0]->dataTime.getTimePassed();
//   // probably not relevant
// }



/**
 * open logfile and print description */
void UGamepad::openLog()
{
  UData::createLogfile("log_gamepad");
  if (logIsOpen())
  {
    fprintf(logfile, "%% Gamepad log\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 number of axis\n");
    fprintf(logfile, "%% 3..%d axis values\n", number_of_axes + 2);
    fprintf(logfile, "%% %d number of buttons\n", number_of_axes + 3);
    fprintf(logfile, "%% %d..%d button values\n", number_of_axes + 4, number_of_buttons + number_of_axes + 3);
  }
}

void UGamepad::sendHelp(int client)
{
  db->sendReply("# ------- Gamepad ----------\r\n", client);
  db->sendReply("# keywords: gamepad     Button values\r\n", client);
}


void UGamepad::run()
{ // try to open port
  connected = initJoy();  
  if (not connected)
  { // no such device
    fprintf(stderr, "# UGamepad:: Can't open joystick device: %s\n",joyDevice);
  }
  while (not th1stop)
  {
    if (not connected)
    { // don't try too often (every 2 seconds)
      sleep(2);
      // retry
      connected = initJoy();  
    }
    else
    { // Device is present
      bool gotEvent= getNewJsData();
      if (gotEvent)
        updateData();
      // sleep for 10ms
      usleep(10000);
    }
  }
  if (connected) 
  { // close device nicely
    connected = false;
    if (jDev >= 0) 
      close(jDev);
    jDev = -1;
  }
}


bool UGamepad::initJoy(void) 
{ //Open first serial js port
  jDev = open (joyDevice, O_RDWR | O_NONBLOCK);
  if (jDev >= 0)
  { // Joystic device found
//     printf("# UGamepad::Opened joystick on port: %s\n",joyDevice);
    //Query and print the joystick name
    char name[128];
    if (ioctl(jDev, JSIOCGNAME(sizeof(name)), name) < 0)
      strncpy(name, "Unknown", sizeof(name));
    //Query number of axes and buttons
    ioctl (jDev, JSIOCGAXES, &number_of_axes);
    ioctl (jDev, JSIOCGBUTTONS, &number_of_buttons);
    printf("# UGamepad:: model: %s with axes %d, buttons %d\n", name, number_of_axes,number_of_buttons);    
  }
  return jDev >= 0;
}  


/** read form joystick driver */
bool UGamepad::getNewJsData()
{
  struct js_event jse;
  bool lostConnection = false;
  bool isOK = false;
  // read full struct or nothing
  int bytes = read(jDev, &jse, sizeof(jse));
  // detect errors
  if (bytes == -1) 
  { // error - an error occurred while reading
    switch (errno)
    { // may be an error, or just nothing send (buffer full)
      case EAGAIN:
        //not all send - just continue
        //         printf("UJoy::getNewJsData: waiting - got EAGAIN (%d bytes)\n", bytes);
        usleep(100);
        break;
      default:
        perror("UGamepad::getNewJsData (other error device error): ");
        lostConnection = true;
        break;
    }
  } 
  if (lostConnection)
  {
    connected = false;
    if (jDev) 
    {
      close(jDev);
      jDev = -1;
      updateData();
    }
  }
  if (connected and bytes > 0)
  {
    if (bytes != sizeof(jse) and bytes > 0) 
    { // size error
      printf("UGamepad::getNewJsData: Unexpected byte count from joystick:%d - continues\n", bytes);
    } 
    else 
    { //Proper joystick package has been recieved
      //Joystick package parser
      isOK = true;
      jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
      switch(jse.type) {
        // changed axis position
        case JS_EVENT_AXIS:
          if (jse.number < 16) {
            joyValues.axes[jse.number] = jse.value;
          }
          break;
          // changed button state
        case JS_EVENT_BUTTON:
          if (jse.number < 16) {
            joyValues.button[jse.number] = jse.value;
          }
          break;
        default:
          printf("UGamepad::getNewJsData : got bad data (event=%d, time=%d) - ignores\n", jse.type, jse.time);
          isOK = false;
          break;
      }
      //       printf("UJoy::getNewJsData : got %d bytes (event=%d, time=%d, value=%d) - debug\n", bytes, jse.type, jse.time, jse.value);
    }
  }
  return isOK;
}


