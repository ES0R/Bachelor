 /***************************************************************************
 * 
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
 
#include <avr/io.h>
#include <avr/interrupt.h>
 
#include "uled.h"
#include "eeconfig.h"
#include "main.h"
#include "logger.h"

void ULEDs::tick(uint32_t tickCnt)
{
  if (loggingIsSet != logger->isLogging())
  { // show that logging is active (if so)
    if (logger->isLogging())
    { // set every other to manual
      for (int i = 0; i < LEDS_CNT; i += 3)
        setLed(i, 0, 0, 200, true);
    }
    else
    { // reset to auto roll state
      for (int i = 0; i < LEDS_CNT; i += 3)
        ledsauto[i] = true;
    }
    loggingIsSet = logger->isLogging();
  }
  if (rc->isRcLost() != (bool)wasRcLost and not state->isArmed())
  { // set to inactive
    if (rc->isRcLost())
    { // set low speed
      setRollSpeed(5);
      wasRcLost = 1;
    }
    else
      wasRcLost = 0;
  }
  //
  if (ledsOn > 0)
  { // make circular movement of a set of LEDs
    if (tickCnt % rollDelay == 1)
    {
      setLed(ledNow, 0, 0, 0);
      int ledOn = (ledNow + ledsOn) % LEDS_CNT;
      switch (wasRcLost)
      {
        case 0: setLed(ledOn, rollColor[0], rollColor[1], rollColor[2]); break;
        case 1: 
          if (command->usbActive())
          {
            setLed(ledOn, 0, 0x80, 0x80); 
//             command->usb_send_str("# USB active\n");
          }
          else
          {
            setLed(ledOn, 0xf0, 0, 0); 
//             command->usb_send_str("# USB not active\n");
          }
          break;
        case 2: setLed(ledOn, 0, 0xf0, 0); break;
        case 3: setLed(ledOn, 0, 0, 0xf0); break;
        default: break;
      }
      if (wasRcLost > 0)
        wasRcLost = wasRcLost % 3 + 1;
      //
      ledNow++;
      if (ledNow >= LEDS_CNT)
        ledNow = 0;
    }
    else if (tickCnt % rollDelay == 2)
    {
      sendToBand();
    }
  }
}

void ULEDs::sendToBand()
{ // use default settings (4MHz, MSB first, data valid on positive edge (MODE0))
  SPI.beginTransaction(SPISettings());
  digitalWrite(LED_band_Pin, HIGH);
  for(int n = 0; n < LEDS_CNT; n++) 
  {
    for (int m = 0; m < 3; m++)
    {
      SPI.transfer(rgb[n][m]);
    }
  }
  digitalWrite(LED_band_Pin, LOW);
  SPI.endTransaction();
}


void ULEDs::sendHelp()
{
  const int MRL = 150;
  char reply[MRL];
  command->usb_send_str("# ----- LED control -----\r\n");
  snprintf(reply, MRL, "#   led N R G B Set led number N (0..11) to this RGB value (0..255).\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   ledb R G B  Set band color RGB value (0..255).\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   leds        Set to default values (controlled by drone_ctrl)\r\n");
  command->usb_send_str(reply);
}

bool ULEDs::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "leds ", 4) == 0)
  { // Set all LEDs to auto
    for (int i = 0; i < LEDS_CNT; i++)
      ledsauto[i] = true;
  }
  else if (strncmp(buf, "ledb ", 4) == 0)
  { // set LED band color - will change with arm state.
    const char * p1 = &buf[4];
    if (strlen(p1) > 5)
    {
      uint8_t r = strtol(p1,(char**)&p1, 10);
      uint8_t g = strtol(p1,(char**)&p1, 10);
      uint8_t b = strtol(p1,(char**)&p1, 10);
      setBandColor(r, g, b);
    }
    else
      command->usb_send_str("# too few parameters: led n r g b\n");
  }
  else if (strncmp(buf, "led ", 3) == 0)
  {
    const char * p1 = &buf[3];
    if (strlen(p1) > 5)
    {
      int n = strtol(p1,(char**)&p1, 10);
      uint8_t r = strtol(p1,(char**)&p1, 10);
      uint8_t g = strtol(p1,(char**)&p1, 10);
      uint8_t b = strtol(p1,(char**)&p1, 10);
      setLed(n, r, g, b, true);
    }
    else
      command->usb_send_str("# too few parameters: led n r g b\n");
  }
  else
    used = false;
  return used;
}

void ULEDs::setBandColor(uint8_t r, uint8_t g, uint8_t b)
{
  rollColor[0] = r;
  rollColor[1] = g;
  rollColor[2] = b;
//   command->usb_send_str("message setting band color\n");
}

void ULEDs::stateSet(UState::ArmState armState)
{
  switch (armState)
  {
    case UState::Init:
      setBandColor(100, 100, 100);
      setRollSpeed(4);
      break;
    case UState::Disarmed:
      setBandColor(0, 0, 100);
      setRollSpeed(2);
      break;
    case UState::Armed:
      setRollSpeed(1);
      if (state->isBypass())
        setBandColor(180, 150, 0);
      else
        setBandColor(0, 200, 0);
      break;
    case UState::Fail:
      setRollSpeed(0.5);
      setBandColor(200, 0, 0);
      break;
  }
}

void ULEDs::setRollSpeed(float spr)
{ // spr is in seconds per full roll
  rollDelay = int(400.0*spr)/LEDS_CNT;
  if (rollDelay < 3)
    rollDelay = 3;
}

void ULEDs::eePromSave()
{
}

void ULEDs::eePromLoad()
{
}
