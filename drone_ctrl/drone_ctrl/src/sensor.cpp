/*
 * Copyright (C) 2021  Christian <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ADC.h>
#include <../teensy3/kinetis.h>
#include <../libraries/ADC/ADC.h>
// #include "../teensy3/kinetis.h"
// #include "../teensy3/pins_arduino.h"
#include "../teensy3/core_pins.h"
#include "pins.h"
#include "sensor.h"
#include "command.h"
#include "uesc.h"
#include "eeconfig.h"
#include "logger.h"
#include "ustate.h"


ADC * adc = new ADC();
uint16_t adcInt0Cnt = 0;
uint16_t adcInt1Cnt = 0;
//uint16_t adcStartCnt = 0;
uint16_t adcValue[ADC_NUM_ALL] = {0};
bool adcRising[ADC_NUM_ALL];
bool adcCCV = true;
//uint32_t adcStartTime, adcConvertTime;
int adcPin[ADC_NUM_ALL] =
{
  PIN_AD_SENSOR_0,
  PIN_AD_SENSOR_1,
  PIN_TEMP_1,
  PIN_TEMP_2,
  PIN_CURRENT_1,
  PIN_CURRENT_2,
  PIN_BATT_VOLT
};


// If you enable interrupts make sure to call readSingle() to clear the interrupt.
void adc0_isr()
{
  uint16_t v = adc->readSingle ( ADC_0 );
  if ( sensor->adcSeq < ADC_NUM_ALL )
  { // save value
    adcValue[sensor->adcSeq] = v;
  }
  sensor->adcSeq++;
//   sensor->adcFinished = true;
  digitalWriteFast ( PIN_ADC_CONV_OUT, LOW );
  if ( sensor->adcSeq < ADC_NUM_ALL ) // start new and re-enable interrupt
  { // start next conversion
    adc->startSingleRead ( adcPin[sensor->adcSeq] );
    digitalWriteFast ( PIN_ADC_CONV_OUT, HIGH );
  }
  else     // finished
  {
    //sensor->adcConvertUs = (ARM_DWT_CYCCNT - sensor->adcStartTime)/(F_CPU/1000000);
    digitalWriteFast ( PIN_ADC_CONV_OUT, LOW );
  }
  adcInt0Cnt++;
}
// 
// //////////////////////////////////////////////////////////
// 
void adc1_isr()
{
  uint16_t v = adc->readSingle ( ADC_1 );
  if ( sensor->adcSeq < ADC_NUM_ALL )
  { // save raw value
    adcValue[sensor->adcSeq] = v;
  }
  sensor->adcSeq++;
//   sensor->adcFinished = true;
  digitalWriteFast ( PIN_ADC_CONV_OUT, LOW );
  
  if ( sensor->adcSeq < ADC_NUM_ALL ) // start new and re-enable interrupt
  { // start conversion
    adc->startSingleRead ( adcPin[sensor->adcSeq] );
    digitalWriteFast ( PIN_ADC_CONV_OUT, HIGH );
  }
  else     // finished
  { // save convert time
    //sensor->adcConvertUs = (ARM_DWT_CYCCNT - sensor->adcStartTime)/(F_CPU/1000000);
    digitalWriteFast ( PIN_ADC_CONV_OUT, LOW );
  }
  adcInt1Cnt++;
}



USensor::USensor()
{
}

void USensor::InitSensor()
{
  const int useADCresolution = 12;
  // init AD converter
  adc->adc0->setResolution (useADCresolution);
  adc->adc1->setResolution (useADCresolution);
  adc->adc0->setReference (ADC_REFERENCE::REF_3V3);
  adc->adc1->setReference ( ADC_REFERENCE::REF_3V3);
  adc->adc0->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED);
  adc->adc1->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED);
  // more pins
  pinMode ( PIN_TEMP_1, INPUT ); // motor temp (A19)
  pinMode ( PIN_TEMP_2, INPUT ); // esc temp (A20)
  // enable interrupt for the remaining ADC oprations
  adc->adc0->enableInterrupts(adc0_isr);
  adc->adc1->enableInterrupts(adc1_isr);
}


bool USensor::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "sensor ", 6) == 0)
  {
    sendSensorData();
  }
  else if (strncmp(buf, "temp ", 4) == 0)
  {
    sendTempData();
  }
  else if (strncmp(buf, "batt ", 4) == 0)
  {
    sendCurrentVolt();
  }
  else if (strncmp(buf, "seq ", 3) == 0)
  {
    startMotorLogSequence(buf);
  }
  // current measurement
  else if (strncmp(buf, "amps ", 4) == 0)
  {
    sendCurrentVals();
  }
  else if (strncmp(buf, "time", 4) == 0)
    sendTimingInfo();
  else if (strncmp(buf, "onbat", 5) == 0)
    setOnBattery(&buf[5]);
  else if (strncmp(buf, "sensi", 5) == 0)
    setSensorsInstalled(&buf[5]);
  else if (strncmp(buf, "adctick ", 7) == 0)
  {
    adcTick();
  }
  else
    used = false;
  return used;
}

void USensor::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  command->usb_send_str("# ------ Sensor -------\r\n");
  snprintf(reply, MRL, "#   temp         Send temperature info (t1 [C],t2 [C], ad2,ad3).\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   time         Send timing info (cycle time[us], sense [us], +control[us], +logetc [us], adc cycle [us], adc cycles/control).\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   batt         Send battery voltage and current (volt [V], amp [A], lipo cells [N], ad4,ad5,ad6).\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   amps         Send current [A], ad4, ad5, adFactor, offset0, offset1.\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   sensor       Send RPS sensor data (rps1, rps2, CCV, present.\r\n");
  command->usb_send_str(reply);
}


void USensor::adcTick()
{ // new ADC values, convert to SI units and filter
  float filt1, filt2;
//   if (adcFullCycles == adcFullCyclesMax)
    filt1 = 1.0/float(adcFullCyclesMax);
//   else
//     filt1 = 1.0;  
  filt2 = 1.0 - filt1;
  
  if (adcTickCnt == 1)
  { // first real value
    for (int i = 0; i < RPS_SENSORS; i++)
    {
      rpsMin[i] = adcValue[i]*minMaxFilt;
      rpsMax[i] = adcValue[i]*minMaxFilt;
    }
    batteryVoltagef = 0.0;
    temp1f = float(adcValue[2]);// *3.3 / 4096.0 * 100.0;
    temp2f = temp1f;
  }
  else if (adcTickCnt > 1)
  { // RPM sensor value calculation
    //
    // check for no RPM sensor - (pin 1 and 4 connected on RPM plug)
    // rpmSensorPresent = adcValue[1] > 2; - not always valid - skip to manual
    //
    if (rpmSensorPresent)
    { // calculate RPM value for each sensor
      for (int i = 0; i < RPS_SENSORS; i++)
      {
        if (adcValue[i]*minMaxFilt > rpsMax[i])
        {
          rpsMax[i] = adcValue[i]*minMaxFilt;
          //         command->usb_send_str("# max set (a or b)\r\n");
        }
        rpsMax[i]--;
        //
        if (adcValue[i]*minMaxFilt < rpsMin[i])
        {
          rpsMin[i] = adcValue[i]*minMaxFilt;
          //         command->usb_send_str("# min set (a or b)\r\n");
        }
        rpsMin[i]++;
        //
        // test for crossing (raising edge)
        span[i] = (rpsMax[i] - rpsMin[i])/minMaxFilt;
        midt[i] = (rpsMax[i] + rpsMin[i])/(2*minMaxFilt);
        if (span[i] < spanLimit)
        { // too long time or not started
          rps[i] = 0;
        }
        else
        {
          if (rpsVal[i])
          { // test for going low
            if (adcValue[i] < midt[i] - span[i] / 5)
            {
              rpsVal[i] = false;
              //           command->usb_send_str("#going low (a or b)\r\n");
              uint32_t dt = ARM_DWT_CYCCNT - edgeTimeDown[i];
              if (edgeTimeDown[i] > 0 and dt > 0)
              { // count Rotation per second
                float w = (F_CPU/(dt/10))/10.0; 
                rps[i] = (rps[i] * 2.0 + w/2.0)/3.0;
              }
              edgeTimeDown[i] = ARM_DWT_CYCCNT;
            }
          }
          else
          { // test for rising edge
            if (adcValue[i] > midt[i] + span[i]/5)
            { // rising edge (shadow starts)
              adcRising[i] = not rpsVal[i];
              rpsVal[i] = true;
              //           command->usb_send_str("#going high (a or b)\r\n");
              uint32_t dt = ARM_DWT_CYCCNT - edgeTimeUp[i];
              if (edgeTimeUp[i] > 0 and dt > 0)
              { // count Rotation per second
                float w = (F_CPU/(dt/10))/10.0; 
                rps[i] = (rps[i] * 2.0 + w/2.0)/3.0;
              }
              edgeTimeUp[i] = ARM_DWT_CYCCNT;
            }
          }
        }
      }
      if (adcRising[0])
      { // sensor 2 has not seen shadow
        adcCCV = not adcValue[1];
      }
    }
  }
  // save current data
  static const int CURRENT_OFFSET_SAMPLES = 100;
  if (currentOffsetCnt < CURRENT_OFFSET_SAMPLES)
  { // sum measurements - assuming zero current (base load ignored ~0.35A with 4 motors)
    if (adcTickCnt > 1)
    { // skipping first measurement
      currentOffset[0] += adcValue[4];
      currentOffset[1] += adcValue[5];
      currentOffsetCnt++;
      if (currentOffsetCnt == CURRENT_OFFSET_SAMPLES)
      { // reduce to average
        currentOffset[0] /= CURRENT_OFFSET_SAMPLES;
        currentOffset[1] /= CURRENT_OFFSET_SAMPLES;
      }
    }
  }
  else
  { // calculate current in amps
    // average over N samples (measurements per cycle)
    const float assumedBaseCurrent = 0; // Amps
    current =  ((adcValue[4] - currentOffset[0]) + 
                (adcValue[5] - currentOffset[1])) * currentFactor + 
                assumedBaseCurrent;
    currentf = currentf * filt2 + current * filt1;
  }
  // battery voltage
  // divided by 18kOhm and 2kOhm, i.e. 1V in to AD is 10V battery
  batteryVoltage = float(adcValue[6]) * (3.3 / 4096.0 * (18.0 + 2.0)/2.0);
  batteryVoltagef = batteryVoltagef * filt2 + batteryVoltage * filt1;
  // temperature
  float temp = float(adcValue[2]) *3.3 / 4096.0 * 100.0;
  temp1f = temp1f * filt2 + temp * filt1;
  temp = float(adcValue[3]) *3.3 / 4096.0 * 100.0;
  temp2f = temp2f * filt2 + temp * filt1;
  // get time passed since last ADC cycle
  // start new ADC cycle
  adcSeq = 0;
//   adcFinished = false;
  // timing of all ADC operations visible on pin 35
  adc->startSingleRead ( adcPin[adcSeq] );
  digitalWriteFast ( PIN_ADC_CONV_OUT, HIGH );
  //
  adcTickCnt++;
}

bool USensor::isBatteryCriticallyLow()
{ 
  return batteryLow > 200;
}


void USensor::readSensorTick(uint32_t tickCnt)
{ // battery monitoring
  if (batteryVoltagef > 5.0)
  { // we are on battery and not just powered by USB
    if (batteryVoltagef > lipoCellCount*batteryVoltageMaximum)
    { // number of cells is too low
      lipoCellCount = int(batteryVoltagef / batteryVoltageMinimum);
    }
    if (batteryVoltagef < batteryVoltageMinimum * lipoCellCount)
    { // crossed the low value
      if (batteryLow < 1000)
        batteryLow++;
    }
    else if (batteryLow > 0)
      batteryLow--;
  }
  // all ADCs are interrupt driven
  // so nothing to do here, except for
  if (logStepping)
  { // this is to give ESC some step input for logging
    // used for making motor model, including ESC
    // started by 'Autostep' in GUI (Logging tab)
    int32_t dt = ARM_DWT_CYCCNT - logStartTime;
    if (dt < 0)
    {
      dt += 1 << 31;
      command->usb_send_str("# log seq - roll over corrected\r\n");
    }
    float ms = dt/float(F_CPU/1000);
    if (tickCnt % 50 == 10)
    {
      const int MSL = 220;
      char s[MSL];
      snprintf(s, MSL, "# Seq tick %ld: ms=%.1f, i=%.0f, size=%d vals, step %d cnt, PWM=%d (0..1024), start=%d, dt=%ld\r\n",
               tickCnt, ms,
               logInterval, 
               logStepSize, 
               logStep, 
               logStepSize * logStep + logStartValue/100, 
               logStartValue/100, 
               dt);
      command->usb_send_str(s);
    }
    if ((ms > logInterval*logStep) and logStep <= logSteps)
    {
      esc->setEscPWM(0, logStepSize * logStep + logStartValue/100, true, 0, 0); 
      logStep++;
      command->usb_send_str("#---- step ----\n");
    }
    if (not logger->isLogging())
    { // no more space in logger for log, so stop
      logStepping = false;
      // stop motor
      esc->setEscPWM(0, 0, true, 0, 0); 
      command->usb_send_str("# log sequence ended\r\n");
    }
  }
}

void USensor::sendTempData()
{
  const int MSL = 260;
  char s[MSL];
  // konverter til oC
  float temp1 = float(adcValue[2]) *3.3 / 4096.0 * 100.0;
  float temp2 = float(adcValue[3]) *3.3 / 4096.0 * 100.0;
  snprintf(s, MSL, "tmp %.2f %.2f %d %d %d\r\n", temp1, temp2, tempSensorPresent, adcValue[2], adcValue[3]);
  // ADC values for debug only
  command->usb_send_str(s);
}  

void USensor::sendSensorData()
{ // reduced to RPS sensor
  const int MSL = 260;
  char s[MSL];
  //
//   snprintf(s, MSL, "sen %g %g %d\r\n"
//                    "# idle limit: %d,\r\n"
//                    "# adc time=%ldus, cycle: %ldus, pin1: %ld - %ld = %ld pin2: %ld - %ld = %ld\n\r", 
//            rps[0], rps[1], adcCCV, spanLimit, 
//            adcConvertUs, adcIntervalUs,
//            rpsMax[0], rpsMin[0], rpsMax[0] - rpsMin[0] , 
//            rpsMax[1], rpsMin[1], rpsMax[1] - rpsMin[1]);
  //
  snprintf(s, MSL, "sen %g %g %d %d %d %d\r\n", rps[0], rps[1], adcCCV, rpmSensorPresent, adcValue[0], adcValue[1]);
  command->usb_send_str(s);
//  uint16_t a, b;
//   a = analogRead(A0);
//   b = analogRead(A1);
//   snprintf(s, MSL, "# ADC raw: %d %d %d %d\r\n", adcValue[0], adcValue[1], adcValue[2], adcValue[3]);
  command->usb_send_str(s);
}

void USensor::sendCurrentVolt()
{
  const int MSL = 260;
  char s[MSL];
  //
  snprintf(s, MSL, "bat %g %g %d %d %d %d %d %d\r\n", batteryVoltagef, currentf, lipoCellCount, onBattery, currentSensorPresent,
           adcValue[4], adcValue[5], adcValue[6]); // ADC values are for debug only
  command->usb_send_str(s);
}

void USensor::sendCurrentVals()
{
  const int MSL = 260;
  char s[MSL];
  //
  snprintf(s, MSL, "curDebug %g %d %d %g %ld %ld\r\n", currentf, adcValue[4], adcValue[5], 
           currentFactor, currentOffset[0], currentOffset[1]);
  command->usb_send_str(s);
}

void USensor::adcClear()
{
  for (int i = 0; i < ADC_NUM_ALL; i++)
    adcValue[i] = 0;
  command->usb_send_str("# ADC cleared\r\n");
}

void USensor::sendTimingInfo()
{ // send basic timing info - control and ADC conversions per sample.
  const int MSL = 260;
  char s[MSL];
  //
  float sense_us = controlUsedTime[0] / c_us; 
  float ctrl_us = controlUsedTime[1] / c_us;
  float cycle_us = controlUsedTime[2] / c_us;
  float cycle_actual10 = controlUsedTime[4] / c_us;
  //
  snprintf(s, MSL, "tim %.2f %.2f %.2f %.2f %.2f %d\r\n", cycle_actual10,
           sense_us, ctrl_us, cycle_us, adc_us, adcFullCycles);
  command->usb_send_str(s);
}

void USensor::setOnBattery(const char * buf)
{
  onBattery = strtol(buf, NULL, 10) == 1;
}

void USensor::setSensorsInstalled(const char* buf)
{
  const char * p1 = buf;
  currentSensorPresent = strtol(p1, (char**) &p1, 10) == 1;
  tempSensorPresent = strtol(p1, (char**) &p1, 10) == 1;
  rpmSensorPresent = strtol(p1, (char**) &p1, 10) == 1;
}


/**
 * start a step sequence on motor 1 and log data
 * number of steps is starting
 * at current value (0..1024) and ending at end value, divided into number of steps
 * format:
 * "seq 220 4 500" is
 * seq  : keyword
 * 220  : time in each step (220ms)
 *   4  : number of steps
 * 500  : end value 
 *  2.5 : log interval
 * i.e. is current value is 100, then start log, wait 220ms, 
 * step to 200, wait, 300, wait, 400, 
 * wait, 500, wait until log is full, then stop */
void USensor::startMotorLogSequence(const char * buf)
{
  char * p1 = (char *)&buf[4];
  float interval = strtof(p1, &p1);
  int steps = strtol(p1, &p1, 10);
  int final = strtol(p1, &p1, 10);
  float logms = strtof(p1, &p1);
  if (logms > 0.0)
  {
    logger->setSampleRate(logms);
    logger->startSensorLog();
    // initiate step sequence
    logStartTime = ARM_DWT_CYCCNT;
    logStartValue = esc->escValue[0];
    logInterval = interval;
    logSteps = steps;
    logStep = 1;
    logStepSize = (final - logStartValue/100)/steps;
    logStepping = true;
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# got start=%d, stepSize=%d, final=%d, steps=%d\r\n", logStartValue, logStepSize, final, logSteps);
    command->usb_send_str(s);
  }
  else
  {
    command->usb_send_str("# failed to understand all parameters in:");
    command->usb_send_str(buf);
    command->usb_send_str("\n\r");
  }
}


void USensor::eePromLoad()
{ // read in same order as write/push
  char flag = eeConfig->readByte();
  currentSensorPresent = (flag & 0x01) != 0;
  tempSensorPresent = (flag & 0x03) != 0;
  rpmSensorPresent = (flag & 0x04) != 0;
  /*float notUsed =*/ eeConfig->readFloat();
}

void USensor::eePromSave()
{
  char flag = 0;
  if (currentSensorPresent)
    flag |= 0x01;
  if (tempSensorPresent)
    flag |= 0x02;
  if (rpmSensorPresent)
    flag |= 0x04;
  eeConfig->pushByte(flag);
  eeConfig->pushFloat(batteryVoltageMinimum);
}
