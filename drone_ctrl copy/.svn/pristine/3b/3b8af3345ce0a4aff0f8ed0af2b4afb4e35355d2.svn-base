/***************************************************************************
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
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


#include "controlbase.h"
#include "eeconfig.h"
#include <unistd.h>

//#include "motor_controller.h"

//bool historyInvalid = false;

////////////////////////////////////////////////////////////

UTransferFunctionPZ::UTransferFunctionPZ()
{
  sampleTime = SAMPLETIME;
  inUse = false;
  tau_den = 1.0;
  tau_num = 1.0;
}


void UTransferFunctionPZ::setParamsTauTau(float tau_numerator, float tau_denominator, float out_limit)
{
  tau_num = tau_numerator;
  tau_den = tau_denominator;
  inUse = out_limit <= 0.0;
  initControl();
}

////////////////////////////////////////////////////////////

void UTransferFunctionPZ::initControl()
{ // include pole zero init
  if (inUse)
  { /* Pole - Zero 1. order filter 
    * Gd:=(td * s + 1)/(al*td*s + 1);
    * denominator (nævner)
    * Gzd2:=expand(Gzd*z^(-1));
    *
    *      (2*al*td + T)u + (T- 2*al*td) u/z
    *      tau_den = td*al;
    * numerator (tæller)
    * Gzn2:=expand(Gzn*z^(-1));
    *                                 
    *      (T + 2*td)e + (T - 2*td) e/z
    *      tau_num = td
    *
    * controller code: u[0] =  -u[1]*zu[1] + e*ze[0] + e[1] * ze[1];
    */
    zu[0] = 2 * tau_den + sampleTime; // denominator 
    zu[1] = (sampleTime - 2 * tau_den) / zu[0];
    ze[0] = (sampleTime + 2 * tau_num) / zu[0]; // numerator
    ze[1] = (sampleTime - 2 * tau_num) / zu[0];
  }
  else
  { // no function (transfer = 0)
    zu[0] = 1;
    zu[1] = 1; // subtract old value - if any
    ze[0] = 0; // do not use input
    ze[1] = 0;
  }
}

void UTransferFunctionPZ::resetControl()
{
  x[0] = 0;
  x[1] = 0;
  y[0] = 0;
  y[1] = 0;
}

////////////////////////////////////////////////////////////

/**
 * simple filter 1. order with output limit*/
void UTransferFunctionPZ::controlTick()
{
  if (inUse)
  {
    y[0] = -zu[1] * y[1] + ze[0] * x[0] + ze[1] * x[1];
    y[1] = y[0];
    x[1] = x[0];
  }
  else
    y[0] = 0.0;
}


const char * UTransferFunctionPZ::decodeFromString(const char * line)
{
  const char * p1 = line;
  char * p2;
  inUse = strtol(p1, &p2, 0);
  if (p2 > p1)
  {
    p1 = p2;
    tau_num = strtof(p1, &p2);
  }
  if (p2 > p1) 
  {
    p1 = p2;
    tau_den = strtof(p1, &p2);
  }
  initControl();
  if (p2 > p1)
    return p2;
  else
    return NULL;
}

int UTransferFunctionPZ::getToString ( char* buf, int bufCnt )
{
  snprintf(buf, bufCnt, "%d %g %g ", inUse, tau_num, tau_den);
  return strlen(buf);
}

void UTransferFunctionPZ::eePromSave()
{
  if (inUse)
  {
    eeConfig->pushFloat(tau_num);
    eeConfig->pushFloat(tau_den);
  }
}

void UTransferFunctionPZ::eePromLoad()
{
  if (inUse)
  {
    tau_num = eeConfig->readFloat();
    tau_den = eeConfig->readFloat();
  }
  initControl();
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

UTransferFunctionI::UTransferFunctionI()
{
  sampleTime = SAMPLETIME;
  inUse = false;
  tau_den = 1.0;
  limit = 1e6;
  limit_use = false;
  andZero = true;
}

////////////////////////////////////////////////////////////

void UTransferFunctionI::setParamsIntegrator(float tau, float i_limit)
{
  tau_den = tau;
  limit = i_limit;
  initControl();
}

////////////////////////////////////////////////////////////

const char * UTransferFunctionI::decodeFromString(const char * line)
{ // from e.g. "1 0.22 4.77"
  const char * p1 = line;
  const char * p2 = p1;
  inUse = strtol(p1, (char**)&p2, 0);
  if (p2 > p1)
  {
    p1 = p2;
    tau_den = strtof(p1, (char**)&p2);
  }
  if (p2 > p1)
  {
    p1 = p2;
    limit = strtof(p1, (char**)&p2);
    if (limit < 1e6 and limit >= 0.0)
      limit_use = true;
  }
  if (p2 > p1)
  {
    p1 = p2;
    andZero = strtol(p1, (char**)&p2, 0);
  }
  initControl();
  if (p2 > p1)
    return p2;
  else
    return NULL;
}

int UTransferFunctionI::getToString(char * buf, int bufCnt)
{
  float v;
  if (limit_use)
    v = limit;
  else
    v = 1e6;
  snprintf(buf, bufCnt, "%d %g %g %d ", inUse, tau_den, v, andZero);
  return strlen(buf);
}

////////////////////////////////////////////////////////////

void UTransferFunctionI::initControl()
{ // include pole zero init
  if (inUse)
  { // first order integrator
    /*
     * GC(s) = u/e = 1/(tau_i s)
     * Gzd2:=expand(Gzd*z^(-1));
     *                        
     *    (2*ti)u + (2*ti)u/z
     *                        
     * Gzn2:=expand(Gzn*z^(-1));
     *                
     *    T*e + T*e/z 
     *                
     *  u[0] = u[1] + T/(2*ti) * e[0] + T/(2*ti) * e[1]
     * 
     */
    zu[0] = 2 * tau_den; // denominator
    zu[1] = -1;
    ze[0] = sampleTime / zu[0]; // numerator
    ze[1] = sampleTime / zu[0];
    limit_use = limit < 99;
  }
  else
  { // no added value from integrator
    zu[0] = 1;
    zu[1] = 1;
    ze[0] = 0;
    ze[1] = 0;
  }
}

void UTransferFunctionI::resetControl()
{
  x[0] = 0;
  x[1] = 0;
  y[0] = 0;
  y[1] = 0;
}


////////////////////////////////////////////////////////////

/**
 * simple filter 1. order */
void UTransferFunctionI::controlTick()
{
  if (inUse)
  {
    y[0] = -zu[1] * y[1] + ze[0] * x[0] + ze[1] * x[1];
    if (limit_use)
    { // integrator limitor in use
      if (y[0] > limit)
        y[0] = limit;
      else if (y[0] < -limit)
        y[0] = -limit;
    }
    y[1] = y[0];
    x[1] = x[0];
  }
  else
    y[0] = 0.0;
}

void UTransferFunctionI::eePromSave()
{
  if (inUse)
  {
    eeConfig->pushFloat(tau_den);
    eeConfig->pushFloat(limit);
  }
}

void UTransferFunctionI::eePromLoad()
{
  if (inUse)
  {
    tau_den = eeConfig->readFloat();
    limit = eeConfig->readFloat();
  }
  initControl();
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


UControlBase::UControlBase(const char * idKey)
{
  use = false;
  kp = 1.0;
  ffKp = 0.0;
//   rateLimitUse = false;
  outLimitUse = false;
  outLimitMax = 1e6;
  outLimitMin = -1e6;
  strncpy(key, idKey, MKL);
  ffUse = false;
  ffKp = 1.0;
  input = NULL;
  measurement = NULL;
  gyro = NULL;
  output = NULL;
  plusMinusPiCheck = false;
}

////////////////////////////////////////////////////////////////////////

void UControlBase::resetControl()
{
  preFilt.resetControl();
  integrator.resetControl();
  leadFwd.resetControl();
  leadBack.resetControl();
  ffFilt.resetControl();
}


////////////////////////////////////////////////////////////////////////

bool UControlBase::setRegulator(const char * line)
{
  const char * p1 = line;
  char * p2;
  while (isSpace(*p1)) p1++;
  bool used = strncmp(line, key, strlen(key)) == 0;
  // format
  // cheight 1 3 0 1 9999.99 1 0 1 1 1 0.029 0.001 0 1 1 1 99.99 1 0.77 1 2.2 3.3 0
  // 1 cheight 1 // use (any part of controller)
  // 2          3 // Kp
  // 3          0 1 9999.99 1 // integrator (use, tau, limit, (and_zero - not implemented))
  // 7          0 1 1 // lead forward (use, zero, pole)
  // 10         1 0.029 0.001 // lead backward (use, zero, pole)
  // 13         0 1 1 // pre-filt (use, zero, pole)
  // 16         1 99.99 -99.9 // output limit (use, limitMax, limitMin)
  // 19         1 0.77 1 2.2 3.3 // Feed forward [use, Kff, useFilt, tau_zero, tau_pole]
  // 24         0 // use angle adjust for value jump at +/- PI
  if (used)
  { // debug
//     const int MSL = 200;
//     char s[MSL];
//     snprintf(s, MSL, "# UControlBase: set %s\n", line);
//     command->usb_send_str(s);
    // debug end
    p1 += strlen(key);
    use = strtol(p1, &p2, 10);
    if (p2 > p1)
    {
      p1 = p2;
      kp = strtof(p1, &p2);
    }
    if (p2 > p1)
    {
      p1 = integrator.decodeFromString(p2);
      if (p1 != NULL)
      {
        p1 = leadFwd.decodeFromString(p1);
        // debug
//         command->usb_send_str("# UControlBase::setRegulator (2): ");
//         command->usb_send_str(p1);
//         command->usb_send_str("\n");
        // debug end
      }
      if (p1 != NULL)
      {
        p1 = leadBack.decodeFromString(p1);    
        if (gyro != NULL)
        { // uses differential input for d-terms
          // use filter on diff input
          leadBack.tau_num = 0;
          leadBack.initControl();
        }
      }
      if (p1 != NULL)
        p1 = preFilt.decodeFromString(p1);    
      if (p1 != NULL)
        outLimitUse = strtol(p1, &p2, 0);
      if (p1 != NULL and p2 > p1)
      { // output limit
        float limin = strtof(p2, &p2);
        float limax = strtof(p2, &p2);
        if (limin > limax)
        { // one value only - make +/-
          limax = limin;
          limin = -limax;
        }
        else if (limin == limax)
          limin = limax - 1.0;
        outLimitMin = limin;
        outLimitMax = limax;
      }
      if (p1 != NULL and p2 > p1)
      { // feed forward
        ffUse = strtol(p2, (char**)&p1, 10);
        ffKp = strtod(p1, (char**)&p1);
        p1 = ffFilt.decodeFromString(p1);
      }
      if (*p1 != '\0')
      {
        plusMinusPiCheck = strtol(p1, NULL, 10);
        // debug
//         const int MSL = 100;
//         char s[MSL];
//         snprintf(s, MSL, "# set +-PI check to %d\n", plusMinusPiCheck);
//         command->usb_send_str(s);
        // debug end
      }
      else
        command->usb_send_str("# UControlBase:: failed read set control (next value is empty)\n");
    }
  }
  return used;
}

int UControlBase::getRegulator ( char* buf, int bufCnt )
{
  char * p1 = buf;
  int n;
  n = snprintf(p1, bufCnt, "%s %d %g ", key, use, kp);
  p1 = &buf[n];
  n += integrator.getToString(p1, bufCnt - n);
  p1 = &buf[n];
  n += leadFwd.getToString(p1, bufCnt - n);
  p1 = &buf[n];
  n += leadBack.getToString(p1, bufCnt - n);
  p1 = &buf[n];
  n += preFilt.getToString(p1, bufCnt - n);
  p1 = &buf[n];
  n += snprintf(p1, bufCnt - n, "%d %g %g ", outLimitUse, outLimitMin, outLimitMax);
  p1 = &buf[n];
  n += snprintf(p1, bufCnt - n, "%d %g ", ffUse, ffKp);
  p1 = &buf[n];
  n += ffFilt.getToString(p1, bufCnt - n);
  p1 = &buf[n];
  n += snprintf(p1, bufCnt - n, "%d %d", plusMinusPiCheck, gyro != NULL);
  return n;
}


void UControlBase::setInputOutput(float * referenceInput, float * measurementInput, float * outputValue, float * gyroInput /*= NULL*/)
{
  input = referenceInput;
  measurement = measurementInput;
  output = outputValue;
  gyro = gyroInput;
}

void UControlBase::controlTick(bool logExtra)
{
//   const int MSL = 120;
//   char s[MSL];
  if (ffUse)
  {
    if (ffFilt.inUse)
    {
      ffFilt.x[0] = *input * ffKp;
      ffFilt.controlTick();
    }
    else
      ffFilt.y[0] = *input * ffKp;
  }
  else
    ffFilt.y[0] = 0;
  // remaining filter - the PID loop
  if (use)
  { // control loop is in use
    // pre filter (pole zero part)
    if (preFilt.inUse)
    { // pre filter (pole zero part) is in use
      preFilt.x[0] = *input;
      preFilt.controlTick();
      preOut = preFilt.y[0];
    }
    else
      preOut = *input;
    // handle feed back part (lead - and potentially gyro
    if (leadBack.inUse)
    { // filter in the feed-back branch
      if (gyro != NULL)
      { // the lead is using an available differentiated input (e.g. a gyro in an angle control)
        // filter using the pole after summation of differential and measurement
        leadBack.x[0] = *measurement + leadBack.tau_num * *gyro;
        // leadBack is modified with no zero
        leadBack.controlTick();
        backEst = leadBack.y[0];
      }
      else
      { // a real lead
        if (plusMinusPiCheck)
        { // resolve discontinuity at +/- PI
          float a = *measurement - leadBack.x[1];
          if (a > M_PI)
          { // adjust old value of x and y to match
            leadBack.x[1] += 2.0 * M_PI;
            leadBack.y[1] += 2.0 * M_PI;
          }
          else if (a < - M_PI)
          { // adjust old value of x and y to match
            leadBack.x[1] -= 2.0 * M_PI;
            leadBack.y[1] -= 2.0 * M_PI;
          }
        }
        leadBack.x[0] = *measurement;
        leadBack.controlTick();
        backEst = leadBack.y[0];
      }
    }
    else
    {
      backEst = *measurement;
    }
    // if input is angle that could fold at +/- pi, then
    // change reference input after pre-filter to same 
    // (half) revolution as measurements
    if (plusMinusPiCheck)
    {
      float a = preOut - *measurement;
      if (a > M_PI)
        preOut -= 2.0 * M_PI;
      else if (a < -M_PI)
        preOut += 2.0 * M_PI;
    }
    // now error can be calculated
    eu = preOut - backEst;
    // multiply with proportional gain
    eu *= kp;
    // Lead filter in forward branch
    if (leadFwd.inUse)
    {
      leadFwd.x[0] = eu;
      leadFwd.controlTick();
      eu = leadFwd.y[0];
    }
    // integrator 
    if (integrator.inUse)
    { // add value from integrator
      integrator.x[0] = eu;
      if (not outLimitUsed)
        // stop integrator if output is saturated
        integrator.controlTick();
      u1 = eu + integrator.y[0];
    }
    else
      u1 = eu; // no itegrator
    // no extra post processing
    // sum with feed forward
    u = u1 + ffFilt.y[0];
    if (outLimitUse)
    {
      if (u > outLimitMax)
      {
        *output = outLimitMax;
        outLimitUsed = true;
      }
      else if (u < outLimitMin)
      {
        *output = outLimitMin;
        outLimitUsed = true;
      }
      else
      {
        *output = u;
        outLimitUsed = false;
      }
    }
    else
    {
      *output = u;
      outLimitUsed = false;
    }
  }
}


void UControlBase::eePromSave()
{
  uint16_t f = 0; // use flags 
  if (use)                 f |= (1 << 0);
  if (integrator.inUse)    f |= (1 << 1);
  if (leadFwd.inUse)       f |= (1 << 2);
  if (leadBack.inUse)      f |= (1 << 3);
  if (preFilt.inUse)       f |= (1 << 4);
  if (outLimitUse)         f |= (1 << 5);
  if (ffUse)               f |= (1 << 6);
  if (ffFilt.inUse)        f |= (1 << 7);
  if (plusMinusPiCheck)    f |= (1 << 8);
  eeConfig->pushWord(f);
  if (use)
  {
    eeConfig->pushFloat(kp);
    integrator.eePromSave();
    leadFwd.eePromSave();
    leadBack.eePromSave();
    preFilt.eePromSave();
    if (ffUse)
    {
      eeConfig->pushFloat(ffKp);
      ffFilt.eePromSave();
    }
  }
  if (outLimitUse)
  { // always save limits
    eeConfig->pushFloat(outLimitMin);
    eeConfig->pushFloat(outLimitMax);
  }
}

void UControlBase::eePromLoad()
{
//   const int MSL = 100;
//   char s[MSL];
  uint16_t f; // use flags 
  f = eeConfig->readWord();
  use =                 f & (1 << 0);
  integrator.inUse =    f & (1 << 1);
  leadFwd.inUse =       f & (1 << 2);
  leadBack.inUse =      f & (1 << 3);
  preFilt.inUse =       f & (1 << 4);
  outLimitUse =         f & (1 << 5);
  ffUse =               f & (1 << 6);
  ffFilt.inUse =        f & (1 << 7);
  plusMinusPiCheck =    f & (1 << 8);
  if (use)
  {
    kp = eeConfig->readFloat();
    integrator.eePromLoad();
    leadFwd.eePromLoad();
    leadBack.eePromLoad();
    preFilt.eePromLoad();
    if (ffUse)
    {
      ffKp = eeConfig->readFloat();
      ffFilt.eePromLoad();
    }
  }
  if (outLimitUse)
  {
    outLimitMin = eeConfig->readFloat();
    outLimitMax = eeConfig->readFloat();
  }
}

bool UControlBase::isMe ( const char* id )
{
  return strncmp(key, id, strlen(key)) == 0;
}

/**
 * \param entry is the log structure to fill
 * */
void UControlBase::toLog(CtrlLogData* entry)
{ // time is already filled
  entry->ref = *input;
  entry->ref2 = preOut;
  entry->ep = (preOut - backEst) * kp;
  entry->up = eu;
  entry->ui = integrator.y[0];
  //entry->u1  = u1; // pre limiter
  entry->u  = *output;     // after limiter
  entry->m  = *measurement;
  entry->m2 = backEst;
  entry->uf = ffFilt.y[0];
}

void UControlBase::getCtrlLog(CtrlLogData * data, int cnt, int dataInterval)
{
  if (cnt == 0 or data == NULL)
    command->usb_send_str("% no data in log\r\n");
  else
  {
    const int MSL = 300;
    char s[MSL];
    //
    snprintf(s, MSL, "%% Controller %s\r\n", key);
    command->usb_send_str(s);
    snprintf(s, MSL, "%% Log with %d values (sample interval=%gms)\r\n", cnt, dataInterval/100.0);
    command->usb_send_str(s);
    snprintf(s, MSL, "%% Uses separate derivative input (gyro) %d\r\n", gyro != NULL);
    command->usb_send_str(s);
    command->usb_send_str("% data format:\r\n");
    command->usb_send_str("% 1   Time (ms)\r\n");
    command->usb_send_str("% 2   Control reference input (setpoint) (r)\r\n");
    command->usb_send_str("% 3   after pre-filter (r2)\r\n");
    command->usb_send_str("% 4   error after Kp (ep)\r\n");
    command->usb_send_str("% 5   error after Kp and lead/lag, before I (up)\r\n");
    command->usb_send_str("% 6   integrator output (ui)\r\n");
    command->usb_send_str("% 7   Controller output (u) after limiter\r\n");
    command->usb_send_str("% 8   measurement (m)\r\n");
    command->usb_send_str("% 9   measurement after filter (m2)\r\n");
    command->usb_send_str("% 10  Feed forward output (uf)\r\n");
    command->usb_send_str("% \r\n");
    //
    for (int i = 0; i < cnt; i++)
    {
      snprintf(s, MSL, "%.2f %.5f %.5f %.5f %.2f %.2f %.2f %.5f %.5f %.5f\r\n",
               float(data[i].time)/100.0, // ms
               data[i].ref,
               data[i].ref2,
               data[i].ep,
               data[i].up,
               data[i].ui,
               data[i].u,
               data[i].m,
               data[i].m2,
               data[i].uf
      );
      command->usb_send_str(s);
    }
    command->usb_send_str("logdata end\r\n");
  }
}



