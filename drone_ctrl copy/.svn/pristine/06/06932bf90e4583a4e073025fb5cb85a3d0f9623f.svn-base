 /***************************************************************************
 * 
 *   Copyright (C) 2020 by DTU                             *
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

#ifndef ULED_H
#define ULED_H

#include <SPI.h>
#include "main.h"
#include "command.h"
#include "ustate.h"

class ULEDs 
{
protected:
  // number of LEDS 
  static const int LEDS_CNT = 12;
  // auto or from user interface
  bool ledsauto[LEDS_CNT];
  // RGB values
  uint8_t rgb[LEDS_CNT][3];
  // roll time in ticks (each 2.5ms), i.e. 1. second
  int rollDelay = 400/LEDS_CNT;
  // roll color
  uint8_t rollColor[3] = {0, 0x36, 0};
  // roll LEDs (on at anyone time)
  int ledsOn = 4;
  // current led
  int ledNow = 0;
  // Enable pin for LED band
  int LED_band_Pin = 7;
  // LED affecting flags
  bool loggingIsSet = false;
  int wasRcLost = false;
  
  
public:
  void setup()
  { // set the SPI client Select Pins as outputs:
    for (int i = 0; i < LEDS_CNT; i++)
      ledsauto[i] = true;
    pinMode (LED_band_Pin, OUTPUT);
    SPI.begin(); 
  }
  /// update LEDS at regular interval
  void tick(uint32_t tickCnt);
  /**
   * send values to LED */
  void sendToBand();
  /**
   * send help */
  void sendHelp();
  /**
   * decode command for this unit */
  bool decode(const char * buf);
  /**
   * save scale and offset */
  void eePromSave();
  /**
   * load scale and offset */
  void eePromLoad();
  /**
   * set band color */
  void setBandColor(uint8_t r, uint8_t g, uint8_t b);
  /**
   * set LED roll speed
   * \param spr is seconds per full roll. */
  void setRollSpeed(float spr);
  /** set LED */
  inline void setLed(int n, uint8_t r, uint8_t g, uint8_t b, bool manual = false)
  {
    if (n >= 0 and n < LEDS_CNT)
    {
      if (manual)
        ledsauto[n] = not manual;
      if (ledsauto[n] or manual)
      {
        rgb[n][0] = r;
        rgb[n][1] = b; // order seems wrong relative to WS2801 spec
        rgb[n][2] = g;
      }
    }
  }
  /**
   * set band color from ArmState */
  void stateSet(UState::ArmState armState);
};


#endif
