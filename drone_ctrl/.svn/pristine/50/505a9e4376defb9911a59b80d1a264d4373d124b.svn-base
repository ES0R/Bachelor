 /***************************************************************************
 *   Copyright (C) 2020 by DTU                             *
 *   jca@elektro.dtu.dk 
 *
 * Main control PID functions 
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
#include "eeconfig.h"
#include "upropshield.h"
#include "mixer.h"
#include "uheight.h"
#include "ustate.h"

/** constructor */
UControl::UControl()
{ // create regulators on heap
  ctrlVelRoll = new UControlBase("vroll"); 
  ctrlVelPitch = new UControlBase("vpitch");
  ctrlVelYaw = new UControlBase("vyaw");
  // position control
    ctrlVelHeight = new UControlBase("height"); 
  ctrlRoll = new UControlBase("roll"); 
  ctrlPitch = new UControlBase("pitch");
//   ctrlYaw = new UControlBase("yaw");
  //
  controlActive = false;
}


///////////////////////////////////////////////////////////////////////

void UControl::eePromSaveCtrl()
{ // must be called in in right order
  ctrlVelRoll->eePromSave();
//   command->usb_send_str("# vroll\n");
  ctrlVelPitch->eePromSave();
//   command->usb_send_str("# vpitch\n");
  ctrlVelYaw->eePromSave();
//   command->usb_send_str("# vyaw\n");
  ctrlVelHeight->eePromSave();
//   command->usb_send_str("# height\n");
  ctrlRoll->eePromSave();
//   command->usb_send_str("# roll\n");
  ctrlPitch->eePromSave();
//   command->usb_send_str("# pitch\n");
  //   ctrlYaw->eePromSave();
  eeConfig->pushFloat(refHeightMin);
  eeConfig->pushFloat(refHeightMax);
  eeConfig->pushFloat(refRollMin);
  eeConfig->pushFloat(refRollMax);
  eeConfig->pushFloat(refPitchMin);
  eeConfig->pushFloat(refPitchMax);
  eeConfig->pushFloat(refYawMin);
  eeConfig->pushFloat(refYawMax);
  
}

void UControl::eePromLoadCtrl()
{ // must be called in in right order
  ctrlVelRoll->eePromLoad();
  ctrlVelPitch->eePromLoad();
  ctrlVelYaw->eePromLoad();
  ctrlVelHeight->eePromLoad();
  ctrlRoll->eePromLoad();
  ctrlPitch->eePromLoad();
  refHeightMin = eeConfig->readFloat();
  refHeightMax = eeConfig->readFloat();
  refRollMin = eeConfig->readFloat();
  refRollMax = eeConfig->readFloat();
  refPitchMin = eeConfig->readFloat();
  refPitchMax = eeConfig->readFloat();
  refYawMin = eeConfig->readFloat();
  refYawMax = eeConfig->readFloat();
  //   ctrlYaw->eePromLoad();
  command->usb_send_str("# loaded data for control from EEflash\n");
}


void UControl::setRegulatorInOut()
{ // set initial input and output for regulators
  // turn controllers
  /* template: void setInputOutput(float * referenceInput, 
                                   float * measurementInput, 
                                   float * outputValue, 
                                   float * gyroInput = NULL); */
  // height velocity control (should probably be renamed)
  ctrlVelHeight->setInputOutput(&refHeight, &hgt->heightVelocity, &mixer->uHeight, &hgt->heightAcc);
  // gyro is in degree per second
  ctrlVelRoll->setInputOutput(&refRollRate, &imu->gyro[0], &mixer->uRoll);
  ctrlVelPitch->setInputOutput(&refPitchRate, &imu->gyro[1], &mixer->uPitch);
  ctrlVelYaw->setInputOutput(&refYawRate, &imu->gyro[2], &mixer->uYaw);
  // pose angle control - all degrees - using gyro if there is a lead in feedback branch
  ctrlRoll->setInputOutput(&refRoll, &imu->rollDeg, &refRollRate, &imu->gyro[0]);
  ctrlPitch->setInputOutput(&refPitch, &imu->pitchDeg, &refPitchRate, &imu->gyro[1]);
//   ctrlYaw->setInputOutput(&refYaw, &imu->yaw, &refVelPitch, &imu->gyro[2]);
}

bool UControl::decode(const char* buf)
{
  bool found = true;
  if (strncmp(buf, "ctrl ", 4) == 0)
  { // a control issue
    decodeCtrl(&buf[4]);
  }
  else if (strncmp(buf, "ref ", 3) == 0)
  { // a control issue
    decodeRef(&buf[3]);
  }
  else if (strncmp(buf, "limit ", 5) == 0)
  { // a control issue
    decodeLimit(&buf[5]);
  }
  else
    found = false;
  return found;
}



bool UControl::decodeCtrl(const char* buf)
{
  const char * p1 = buf;
  const char * p2;
  bool found;
  bool moreParams = false;
  while (isspace(*p1)) p1++;
  p2 = strchr(p1, ' ');
  if (p2 != NULL)
  { // look for controller keyword start
    while (isspace(*p2)) p2++; // p2 is controller ID
    moreParams = *p2 > ' ';
  }
  if (moreParams)
  { // further parameters, so set controller
    // command->usb_send_str("# found params - setting controller\n");
    found = setRegulator(p1);
  }
  else
  {  // no further parameters, so must be request for status
    // command->usb_send_str("# no extra - sending status\n");
    found = sendStatusControl(p1);
  }
  if (not found)
  {
    command->usb_send_str("# missing parameters\n");
    sendHelp();
  }
  return found;
}

bool UControl::decodeRef(const char* buf)
{
  const char * p1 = buf;
  const char * p2;
  bool moreParams = false;
  while (isspace(*p1)) p1++;
  p2 = strchr(p1, ' ');
  moreParams = p2 != NULL;
  if (moreParams)
  { // further parameters, so set controller
    float h = strtof(p1,(char**)&p2);
    float r  = strtof(p2, (char**)&p2);
    float p = strtof(p2, (char**)&p2);
    float y   = strtof(p2, (char**)&p1);
    if (p1 > p2)
    { // all params received OK
      if (state->isUsbControl())
        // we are in auto mode with control from USB
        setRef(h, r, p, y);
      else
        command->usb_send_str("# Not in control\r\n");
    }
    else
      // error in decoding
      command->usb_send_str("# error in decoding ref, requires 0 or 4 parameters\r\n");
  }
  else
  {  // no further parameters, so must be request for status
    const int MSL = 150;
    char s[MSL];
    snprintf(s, MSL, "refi %g %g %g %g\n", refHeight,
             refRoll, refPitch, refYaw);
    command->usb_send_str(s);
  }
  return true;
}

bool UControl::decodeLimit(const char* buf)
{
  const char * p1 = buf;
  const char * p2;
  bool moreParams = false;
  while (isspace(*p1)) p1++;
  p2 = strchr(p1, ' ');
  moreParams = p2 != NULL;
  if (moreParams)
  { // further parameters, so set controller
    // format:
    // 'limit' height min, height max, roll lim, pitch lim, yaw rate limit
    //
    refHeightMin = strtof(p1,(char**)&p2); 
    refHeightMax = strtof(p2,(char**)&p2); 
    //
    refRollMax = strtof(p2, (char**)&p2);
    refRollMin = -refRollMax;
    //
    refPitchMax = strtof(p2, (char**)&p2);
    refPitchMin = - refPitchMax;
    //
    refYawMax = strtof(p2, (char**)&p1);
    refYawMin = -refYawMax;
    //
    if (p1 == p2)
      // error in decoding
      command->usb_send_str("# error in decoding limits, requires 0 or 4 parameters\r\n");
  }
  else
  {  // no further parameters, so must be request for status
    const int MSL = 150;
    char s[MSL];
    snprintf(s, MSL, "limiti %.2f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", 
             refHeightMin, refHeightMax, 
             refRollMin, refRollMax,
             refPitchMin, refPitchMax,
             refYawMin, refYawMax
            );
    command->usb_send_str(s);
  }
  return true;
}

void UControl::sendHelp()
{
  // control set example
  // cheight 1 3 0 1 9999.99 1 0 1 1 1 0.029 0.001 0 1 1 1 99.99
  // 1 cheight  1 // use (any part of controller)
  // 2          3 // Kp
  // 3          0 1 9999.99 1 // integrator (use, tau, limit, (and_zero - not implemented))
  // 7          0 1 1 // lead forward (use, zero, pole)
  // 10         1 0.029 0.001 // lead backward (use, zero, pole)
  // 13         0 1 1 // pre-filt (use, zero, pole)
  // 16         1 -0.5 +12 // output limit (use, min max)
  command->usb_send_str( "# - control ------\r\n");
  command->usb_send_str( "#   ctrl XXXX y y... Set XXXX controller \r\n");
  command->usb_send_str( "#                    y y...=use,Kp,useI,taui,ilimit,1,useLeadf,tz,tp,\r\n");
  command->usb_send_str( "#                           useLeadb,tz,tp,usePre,tz,tp,useULim Ulim\r\n");
  command->usb_send_str( "#   ctrl XXXX    Get parameters from controller XXXX\r\n");
  command->usb_send_str( "#                XXXX = vroll, roll, vpitch, pitch, vyaw, yaw, height\r\n");
  command->usb_send_str( "#   ref h r p y  Set control reference H=0..1000, r,p,y= -500..500\r\n");
  command->usb_send_str( "#   ref          Get control reference\r\n");
  command->usb_send_str( "#   limit h H R P Y  Set min trust, max trust, +/- roll, +/- pitch, +/- yaw rate\r\n");
  command->usb_send_str( "#   limit          Get limit values\r\n");
}


bool UControl::setRegulator(const char* line)
{ // set parameters from string
  bool used;
  // set also regulator for other wheel with same values
  used = ctrlVelRoll->setRegulator(line);
  if (not used)
    used = ctrlVelPitch->setRegulator(line);
  if (not used)
    used = ctrlVelYaw->setRegulator(line);
  if (not used)
    used = ctrlVelHeight->setRegulator(line);
  if (not used)
    used = ctrlRoll->setRegulator(line);
  if (not used)
    used = ctrlPitch->setRegulator(line);
//   if (not used)
//     used = ctrlYaw->setRegulator(line);
  
  if (used)
    command->usb_send_str("# Used OK\n");
  else
    command->usb_send_str("# Not used OK\n");
  
  return used;
}

bool UControl::sendStatusControl ( const char* line )
{ // line is request ID for the controller to send
  const int MSL = 270;
  char s[MSL] = "control ";
  bool isOK = true;
  int n = strlen(s);
  char * p1 = &s[n];
  if (ctrlVelRoll->isMe(line))
    n += ctrlVelRoll->getRegulator(p1, MSL - n);
  else if (ctrlVelPitch->isMe(line))
    n += ctrlVelPitch->getRegulator(p1, MSL - n);
  else if (ctrlVelYaw->isMe(line))
    n += ctrlVelYaw->getRegulator(p1, MSL - n);
  else if (ctrlVelHeight->isMe(line))
    n += ctrlVelHeight->getRegulator(p1, MSL - n);
  else if (ctrlRoll->isMe(line))
    n += ctrlRoll->getRegulator(p1, MSL - n);
  else if (ctrlPitch->isMe(line))
    n += ctrlPitch->getRegulator(p1, MSL - n);
//   else if (ctrlYaw->isMe(line))
//     n += ctrlYaw->getRegulator(p1, MSL - n);
  else
  {
    isOK = false;
    n = 0;
    command->usb_send_str("# controller not found\n");
  }
  if (isOK)
  {
    strncpy(&s[n], "\r\n", MSL - n);
    command->usb_send_str(s, true);
  }
  return isOK;
}


void UControl::resetControl()
{
  ctrlVelRoll->resetControl();
  ctrlVelPitch->resetControl();
  ctrlVelYaw->resetControl();
  ctrlVelHeight->resetControl();
  ctrlRoll->resetControl();
  ctrlPitch->resetControl();
  // ctrlYaw->resetControl();
    refRollRate   = 0;
    refPitchRate = 0;  
    refYawRate = 0;  
  refHeight = 0;
  refRoll = 0;
  refPitch = 0;
  refYaw = 0;
}

//////////////////////////////////////////////////////////////////////////

void UControl::controlTick(void)  
{
  if (controlActive)
  { // Do control tick for all enabled controls
    if (ctrlVelHeight->use)
            ctrlVelHeight->controlTick();
    else
    { // is no height controller, then stick is feed directly to mixer
      // ref height is from 0 to 1.0
      // esc takes 0 to 1024
      mixer->uHeight = refHeight;
    }
    // Roll angle
    if (ctrlRoll->use)
    {
      ctrlRoll->controlTick();
    }
    else
            refRollRate = 0; // no ctrl (disabled)
    // Pitch angle
    if (ctrlPitch->use)
      ctrlPitch->controlTick();
    else
            refPitchRate = 0; // no ctrl (disabled)
    // Yaw
        refYawRate = refYaw; // bypass (Yaw velocity control)
    //
    // Roll angle rate
    if (ctrlVelRoll->use)
    { // velocity control active
      ctrlVelRoll->controlTick();
    }
    else
      mixer->uRoll = 0; // disabled
    // Pitch angle rate
    if (ctrlVelPitch->use)
      ctrlVelPitch->controlTick();
    else
      mixer->uPitch = 0; // disabled
    // Yaw angle rate
    if (ctrlVelYaw->use)
      ctrlVelYaw->controlTick();
    else
    { // normal mode is direct Yaw velocity control
      // should probably reduce to +/- 45 deg/s (from +/- 180 deg from stick)
      mixer->uYaw = 0;
    }
    // limit yaw, roll and pitch, if not flying
    // based on minimum trust
    if (true)
    { // scale roll, pitch and Yaw with trust 
      // to ensure no motor is running, if no (height) trust
      float trustNotFlying = 200.0; // scale from 0 to 1000
      if (mixer->uHeight < trustNotFlying)
      { // reduce roll, pitch and yaw relative to trust, if too low to fly
        mixer->uRoll *= mixer->uHeight/trustNotFlying;
        if (mixer->droneType != mixer->drone2x2)
        {
          mixer->uPitch *= mixer->uHeight/trustNotFlying;
          mixer->uYaw *= mixer->uHeight/trustNotFlying;
        }
      }
    }
  }
}

// bool UControl::canArm()
// {
//   bool canArm;
//   canArm = refHeight < 50;
//   canArm &= fabs(refYaw) < 2.0;
//   canArm &= fabs(refRoll) < 2.0;
//   canArm &= fabs(refPitch) < 2.0;
//   if (not canArm)
//   { // send message
//     if (refHeight >= 50)
//       command->usb_send_str("message Trust/height velocity too high (>3%)\n");
//     if (fabs(refYaw) > 2.0)
//       command->usb_send_str("message Yaw rate too high (> 2 deg/s)\n");
//     if (fabs(refRoll) > 2.0)
//       command->usb_send_str("message Roll angle too high (> 2 deg/s)\n");
//     if (fabs(refPitch) > 2.0)
//       command->usb_send_str("message Pitch angle too high (> 2 deg/s)\n");
//   }  
//   return canArm;
// }

void UControl::setRef(float height, float roll, float pitch, float yaw)
{
  if (height < refHeightMin)
    refHeight = refHeightMin;
  else if (height > refHeightMax)
    refHeight = refHeightMax;
  else
    refHeight = height;
  if (roll < refRollMin)
    refRoll = refRollMin;
  else if (roll > refRollMax)
    refRoll = refRollMax;
  else
    refRoll = roll;
  if (pitch < refPitchMin)
    refPitch = refPitchMin;
  else if (pitch > refPitchMax)
    refPitch = refPitchMax;
  else
    refPitch = pitch;
  if (yaw < refYawMin)
    refYaw = refYawMin;
  else if (yaw > refYawMax)
    refYaw = refYawMax;
  else
    refYaw = yaw;
}

