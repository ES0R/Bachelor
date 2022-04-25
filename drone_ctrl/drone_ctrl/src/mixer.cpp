 /***************************************************************************
 * 
 * The main control functions are in control.cpp file
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

#include "control.h"
#include "mixer.h"
#include "command.h"
#include "uesc.h"
#include "ustate.h"
#include "eeconfig.h"

void UMixer::setup()
{ // set start offset for all actuators
  // to be used alone when not armed and
  // added to mixer output when armed
  // a value from 0..1023 corresponds to 1..2 ms PWM signal
  // can be negative, if idle is shorter than 1ms.
  switch (droneType)
  {
    case x_config:
    case plus_config:
      // normal drone, idle value is no power (1ms pulse)
      for (int i = 0; i < esc->MAX_ESC_CNT; i++)
        esc->idleValue[i] = 0;
      break;
      esc->esc_installed = 6;
    case drone2x2:
      // 4 motors, no power is 1ms - zero all
      esc->esc_installed = 6;
      for (int i = 0; i < esc->MAX_ESC_CNT; i++)
        esc->idleValue[i] = 0;
      // except two axes use servo with center  is neutral
      esc->idleValue[4] = 512; // is prop tilt servo 1 - mid position ~1.5ms
      esc->idleValue[5] = 512; // is prop tilt servo 2
      break;
    default:
      command->usb_send_str("#unknown drone configuration 2x2, 4+, 4x, 6+, 6x, 8+ or 8x is OK\n");
      break;
  }
}


UMixer::UMixer()
{  
}

void UMixer::sendStatus()
{
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "mixi %.3f %.3f %.3f %.3f %d\n", 
           uHeight, 
           uRoll, 
           uPitch, 
           uYaw,
           esc->escLimitMax
          );
  command->usb_send_str(s);
}

void UMixer::sendMixerConfig()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "mixcon %d %d   %d %d %d %d %d %d %d %d\n", 
           esc->esc_installed,
           droneType,
           esc->idleValue[0],
           esc->idleValue[1],
           esc->idleValue[2],
           esc->idleValue[3],
           esc->idleValue[4],
           esc->idleValue[5],
           esc->idleValue[6],
           esc->idleValue[7]
  );
  command->usb_send_str(s);
}

void UMixer::sendHelp()
{
  command->usb_send_str("# ----- mixer -----\r\n");
  command->usb_send_str( "#   mix          Get mixer input values (controller output)\r\n");
  command->usb_send_str( "#   mixc         Get mixer (drone) configuration and ESC offset\r\n");
  command->usb_send_str( "#   mixo 1 2 3 4 5 6 7 8   Set mixer offset for each ESC (in microseconds)\r\n");
  command->usb_send_str( "#   mixv n x     Set mixer configuration n=ESCs, x:1=x,2=+,3=2x1 or 2x2\r\n");
}

bool UMixer::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "mixc ", 4) == 0)
  { // must be tested before mix
    sendMixerConfig();
  }
  if (strncmp(buf, "mixo ", 4) == 0)
  { // must be tested before mix
    const char * p1 = &buf[4];
    for (int k = 0; k < esc->MAX_ESC_CNT; k++)
    {
      esc->idleValue[k] = strtol(p1, (char **)&p1, 10);
    }
  }
  if (strncmp(buf, "mixv ", 4) == 0)
  { // must be tested before mix
    const char * p1 = &buf[4];
    int n = strtol(p1, (char **)&p1, 10);
    int x = strtol(p1, (char **)&p1, 10);
    if (n > 1 and n <= esc->MAX_ESC_CNT)
      esc->esc_installed = n;
    switch (x)
    {
      case 1: droneType = x_config; break;
      case 2: droneType = plus_config; break;
      case 3: droneType = drone2x2; break;
      default:
        droneType = x_config; 
        command->usb_send_str("#unknown drone configuration 1,2,3 is OK\n");
        break;
    }
  }
  else if (strncmp(buf, "mix ", 3) == 0)
  { // mixer input status
    sendStatus();
  }
  else
    used = false;
  return used;
}


void UMixer::tick()
{
  float uh = uHeight;
  float landingFactor;
  if (state->isLanding(&landingFactor))
  { // slowly decrease trust as suggested by controler
    uh *= landingFactor;
  }
  switch (droneType)
  {
    case x_config:
      if (esc->esc_installed == 4)
      { // motor 1 front right running CV, 
        // motor numbering clockwise
        esc->escRef[0] = uh - uRoll - uPitch - uYaw + esc->idleValue[0];
        esc->escRef[1] = uh - uRoll + uPitch + uYaw + esc->idleValue[1]; 
        esc->escRef[2] = uh + uRoll + uPitch - uYaw + esc->idleValue[2];
        esc->escRef[3] = uh + uRoll - uPitch + uYaw + esc->idleValue[3]; 
      }
      else if (esc->esc_installed == 6)
      { // motor 1 front right running CV, 
        // motor numbering clockwise
        esc->escRef[0] = uh - uRoll - uPitch - uYaw + esc->idleValue[0];
        esc->escRef[1] = uh - uRoll          + uYaw + esc->idleValue[1];
        esc->escRef[2] = uh - uRoll + uPitch - uYaw + esc->idleValue[2];
        esc->escRef[3] = uh + uRoll + uPitch + uYaw + esc->idleValue[3];
        esc->escRef[4] = uh + uRoll          - uYaw + esc->idleValue[4];
        esc->escRef[5] = uh + uRoll - uPitch + uYaw + esc->idleValue[5];
      }
      else if (esc->esc_installed == 8)
      { // x configuration
        // motor 1 front right CV, 
        // motor numbering clockwise
        esc->escRef[0] = uh - uRoll - uPitch - uYaw + esc->idleValue[0];
        esc->escRef[1] = uh - uRoll - uPitch + uYaw + esc->idleValue[1];
        esc->escRef[2] = uh - uRoll + uPitch - uYaw + esc->idleValue[2];
        esc->escRef[3] = uh - uRoll + uPitch + uYaw + esc->idleValue[3];
        esc->escRef[4] = uh + uRoll + uPitch - uYaw + esc->idleValue[4];
        esc->escRef[5] = uh + uRoll + uPitch + uYaw + esc->idleValue[5];
        esc->escRef[6] = uh + uRoll - uPitch - uYaw + esc->idleValue[6];
        esc->escRef[7] = uh + uRoll - uPitch + uYaw + esc->idleValue[7];
      }
      break;
    case plus_config:
      if (esc->esc_installed == 4)
      { // + configuration
        // motor 1 front CV, 
        // motor numbering clockwise
        esc->escRef[0] = uh         - uPitch - uYaw + esc->idleValue[0];
        esc->escRef[1] = uh - uRoll          + uYaw + esc->idleValue[1];
        esc->escRef[2] = uh + uRoll          - uYaw + esc->idleValue[2];
        esc->escRef[3] = uh         + uPitch + uYaw + esc->idleValue[3];
      }
      else if (esc->esc_installed == 6)
      { // + configuration
        // motor 1 front CV, 
        // motor numbering clockwise
        esc->escRef[0] = uh         - uPitch - uYaw + esc->idleValue[0];
        esc->escRef[1] = uh - uRoll - uPitch + uYaw + esc->idleValue[1];
        esc->escRef[2] = uh - uRoll + uPitch - uYaw + esc->idleValue[2];
        esc->escRef[3] = uh         + uPitch + uYaw + esc->idleValue[3];
        esc->escRef[4] = uh + uRoll + uPitch - uYaw + esc->idleValue[4];
        esc->escRef[5] = uh + uRoll - uPitch + uYaw + esc->idleValue[5];
      }
      else if (esc->esc_installed == 8)
      { // + configuration
        // motor 1 front CV, 
        // motor numbering clockwise
        esc->escRef[0] = uh         - uPitch - uYaw + esc->idleValue[0];
        esc->escRef[1] = uh - uRoll - uPitch + uYaw + esc->idleValue[1];
        esc->escRef[2] = uh - uRoll          - uYaw + esc->idleValue[2];
        esc->escRef[3] = uh - uRoll + uPitch + uYaw + esc->idleValue[3];
        esc->escRef[4] = uh         + uPitch - uYaw + esc->idleValue[4];
        esc->escRef[5] = uh + uRoll + uPitch + uYaw + esc->idleValue[5];
        esc->escRef[6] = uh + uRoll          - uYaw + esc->idleValue[6];
        esc->escRef[7] = uh + uRoll - uPitch + uYaw + esc->idleValue[7];
      }
      break;
    case drone2x2: // or 2x1
      // + configuration
      // also valid for 2x1 configuration, when ESC 1 and 3 used, leaving esc 2 and 4 empty.
      // motor 1 right upper CV, 2 right lower CCV, 3 left upper CCV, 4 left lower CV
      // servo right = esc 4, servo left = esc 5
      // motor numbering clockwise
      esc->escRef[0] = uh - uRoll + esc->idleValue[0];
      esc->escRef[1] = uh - uRoll + esc->idleValue[1]; // not used by 2x1
      esc->escRef[2] = uh + uRoll + esc->idleValue[2];
      esc->escRef[3] = uh + uRoll + esc->idleValue[3]; // not used by 2x1
      esc->escRef[4] = ( uPitch - uYaw  + esc->idleValue[4])* 0.61; //0.61 accounting for different servos (199dg / 121dg)
      esc->escRef[5] = (-uPitch - uYaw  + esc->idleValue[5]); 

      break;
    break;
    default:
      command->usb_send_str("#unknown drone configuration 2x1, 2x2, 4+, 4x, 6+, 6x, 8+ or 8x is OK\n");
      break;
  }
}

bool UMixer::canArm()
{
  //bool can = false;
  //can = uHeight < 100;
  //return can;
  bool can = true;
  return can;
}



void UMixer::eePromSave()
{
  // save desired PWM FRQ
  eeConfig->pushByte((uint8_t)droneType);
}

void UMixer::eePromLoad()
{
  int d = eeConfig->readByte();
  if (d > 0 and d < unknown)
    droneType = (DroneType) d;
}
