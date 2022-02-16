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
#include "upropshield.h"
#include "command.h"
#include "ultrasound.h"
#include "uheight.h"
#include "eeconfig.h"
#include "ustate.h"

UHeight::UHeight()
{
  height = 0;
  heightVelocity = 0;
}

void UHeight::setup()
{ // initialize
  int1.inUse = true;
  int1.setParamsIntegrator(1, 100);
  int2.inUse = true;
  int2.setParamsIntegrator(1, 100);
  // 1st order integrator with pole in zero replaced with pole at beta lower frq
  // requires that input (or output) is multiplied with tau^2*beta
  lowPass.setParamsTauTau(0.0, 0.1, 0);
}


void UHeight::sendStatus()
{
  const char MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "mhgt %.3f %.3f\n", height, heightVelocity);
  command->usb_send_str(s);
}

void UHeight::sendFilterStatus()
{
  const char MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "fht %.3f\n", lowPassTau);
  command->usb_send_str(s);
}

void UHeight::sendHelp()
{
  command->usb_send_str("# ----- Height data -----\r\n");
  command->usb_send_str( "#   hgt          Get merged height estimate.\r\n");
  command->usb_send_str( "#   fhgt         Get height filter low pass tau.\r\n");
  command->usb_send_str( "#   hfilt tau    Set height filter.\r\n");
}

bool UHeight::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "hgt ", 3) == 0)
  { // report ultrasound height and height velocity
    sendStatus();
  }
  else if (strncmp(buf, "fhgt ", 4) == 0)
  { // report ultrasound height and height velocity
    sendFilterStatus();
  }
  else if (strncmp(buf, "hfilt ", 5) == 0)
  {
    char * p1 = (char *)&buf[5];
    lowPassTau = 0.1;
    lowPassTau = strtof(p1, &p1);
    if (lowPassTau >= 0.001)
      lowPass.setParamsTauTau(0.0, lowPassTau, 0);
    else
      command->usb_send_str("# height filter tau must be > 0\n");
  }
  else
    used = false;
  return used;
}

void UHeight::altComplFiltTick(uint32_t mainLoop)
{ // Height filter constants taken from "Altitude data fusion utilising differential 
  // measurement and complementary filter" by Sheng Wei, Gu Dan and Hu Chen
  // the altitude in this filter is without the altitude offset.
  // offset must be removed later.
  const float k1 = 0.25;
  const float k2 = 0.015;
  float accw[3];
  // get acc vector in world coordinates
  imu->mrotd2w.rotateD2W(imu->acc, accw);
  // acc prefilter
  if (mainLoop < 3)
  { // to avoid bad initial values
    int1.resetControl();
    int2.resetControl();
    lowPass.resetControl();
  }
  // acc in g, above gravity, converted to m/s^2
  heightAcc = (accw[2] - 1.0)*9.82;
  float nybaro = imu->altitude - altitudef;
  int1.x[0] = heightAcc + k2*nybaro;
  int1.controlTick();
  float z_vel = int1.y[0] + k1*nybaro;
  int2.x[0] = z_vel;
  int2.controlTick();
  altitudef = int2.y[0];
  lowPass.x[0] = z_vel;
  lowPass.controlTick();
  z_vel_fil = lowPass.y[0];
  if (false)
  { // debug
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# int1.x[0]=%g,[1]=%g, int2.x[0]=%g, [1]=%g, lowPass.x[0]=%g,y=%g\n", int1.x[0], int1.x[1], int2.x[0], int2.x[1], lowPass.x[0], z_vel_fil);
    command->usb_send_str(s);
  } // debug end
  
}


void UHeight::combineHeight(uint32_t  mainLoop)
{
  // complementary altitude filter with accelerometer
  altComplFiltTick(mainLoop);
  
  float cdv;
  // get ultrasound height
  float uh = uhgt->sonarHeight;
  bool useSonar = uh < uhgt->heightLimit and uhgt->sonarValid and uh > 0;
  if (useSonar)
  { // change height offset for pressure height to match
    imu->altOffset = altitudef - uh;
    // use sonar height directly
    height = uh;
  }
  else
  { // no (valid) sonar data 
    // reject negative height, if on-ground
    height = altitudef - imu->altOffset;
    if (state->flightState == UState::OnGround)
    { // must be an error - force zero height
      imu->altOffset = altitudef;
      height = 0;
    }
  }
  // height velocity
  if (useSonar and fabs(uhgt->sonarVelocity) < 15.0)
    cdv = uhgt->sonarVelocity * mergeFactor + z_vel_fil * (1.0 - mergeFactor);
  else
    cdv = z_vel_fil;
  heightVelocity = cdv;
}

void UHeight::sendPose()
{
  const int MSL = 120;
  char s[MSL];
  snprintf(s, MSL, "pos %5.3f %5.3f %5.3f %.2f %.3f %d\n", 
            imu->roll, imu->pitch, imu->yaw, height, heightVelocity, txcnt++);
  command->usb_send_str(s, false);
}


void UHeight::eePromLoad()
{
  filterTau = eeConfig->readFloat();
  lowPassTau = eeConfig->readFloat();
}

void UHeight::eePromSave()
{
  eeConfig->pushFloat(filterTau);
  eeConfig->pushFloat(lowPassTau);
}


