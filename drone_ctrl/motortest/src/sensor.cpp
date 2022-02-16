/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2020  Christian <email>
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


ADC * adc = new ADC();
uint16_t adcInt0Cnt = 0;
uint16_t adcInt1Cnt = 0;
//uint16_t adcStartCnt = 0;
uint16_t adcValue[ADC_NUM_ALL];
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

// typedef struct aaa
// {
//   uint32_t time;
//   uint16_t adc[2];
//   float    rps[2];
//   uint16_t max[2],min[2];
//   char     val[2];
// } SensorData;

// assumed size 24 bytes
//const int MAX_DATA_CNT = 75000/24;
SensorData data[MAX_DATA_CNT];
int dataIdx = 0;
// bool dataLog = false;
// int32_t dataInterval = 1000; // microsecond
// int32_t dataLogLast = 0; // microsecond

// If you enable interrupts make sure to call readSingle() to clear the interrupt.
void adc0_isr()
{
  uint16_t v = adc->readSingle ( ADC_0 );
  if ( sensor->adcSeq < ADC_NUM_ALL )
  { // save value
    adcValue[sensor->adcSeq] = v;
  }
  sensor->adcSeq++;
  if ( sensor->adcSeq < ADC_NUM_ALL ) // start new and re-enable interrupt
  { // start next conversion
    adc->startSingleRead ( adcPin[sensor->adcSeq] );
  }
  else     // finished
  {
    sensor->adcConvertUs = (ARM_DWT_CYCCNT - sensor->adcStartTime)/(F_CPU/1000000);
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
  { // IR sensor use RAW AD - averaged later
    adcValue[sensor->adcSeq] = v;
  }
  sensor->adcSeq++;
  if ( sensor->adcSeq < ADC_NUM_ALL ) // start new and re-enable interrupt
  { // start conversion
    adc->startSingleRead ( adcPin[sensor->adcSeq] );
  }
  else     // finished
  { // save convert time
    sensor->adcConvertUs = (ARM_DWT_CYCCNT - sensor->adcStartTime)/(F_CPU/1000000);
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
  adc->setResolution ( useADCresolution, ADC_0 );
  adc->setResolution ( useADCresolution, ADC_1 );
  adc->setReference ( ADC_REFERENCE::REF_3V3, ADC_0 );
  adc->setReference ( ADC_REFERENCE::REF_3V3, ADC_1 );
  adc->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED, ADC_0 );
  adc->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED, ADC_1 );
  // more pins
  pinMode ( PIN_TEMP_1, INPUT ); // motor temp (A19)
  pinMode ( PIN_TEMP_2, INPUT ); // esc temp (A20)
  // enable interrupt for the remaining ADC oprations
  adc->enableInterrupts ( ADC_0 );
  adc->enableInterrupts ( ADC_1 );
}

void USensor::adcTick()
{ // new ADC values, convert to RPM
//   int32_t dt;
  float filt1 = 1.0/float(sensor->adc_per_cycle);
  float filt2 = 1.0 - filt1;
//   usb_send_str("#adcTick\r\n");
  if (adcTickCnt > 1)
  { // sensor value calculation
    for (int i = 0; i < RPS_SENSORS; i++)
    {
      if (adcValue[i]*minMaxFilt > rpsMax[i])
      {
        rpsMax[i] = adcValue[i]*minMaxFilt;
//         usb_send_str("# max set (a or b)\r\n");
      }
        rpsMax[i]--;
      //
      if (adcValue[i]*minMaxFilt < rpsMin[i])
      {
        rpsMin[i] = adcValue[i]*minMaxFilt;
//         usb_send_str("# min set (a or b)\r\n");
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
  //           usb_send_str("#going low (a or b)\r\n");
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
          { // rising edge (shaddow starts)
            rpsVal[i] = true;
            adcRising[i] = true;
  //           usb_send_str("#going high (a or b)\r\n");
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
    {
      adcCCV = not adcValue[1];
    }
  }
  else if (adcTickCnt == 1)
  { // first real value
    for (int i = 0; i < RPS_SENSORS; i++)
    {
      rpsMin[i] = adcValue[i]*minMaxFilt;
      rpsMax[i] = adcValue[i]*minMaxFilt;
    }
    batteryVoltagef = 0.0;
    temp1f = float(adcValue[2]) *3.3 / 4096.0 * 100.0;
    temp2f = temp1f;
  }
  // save current data
  if (currentOffsetCnt < 100)
  { // sum measurements - assuming zero current
    if (adcTickCnt > 1)
    { // skipping first measurement
      currentOffset[0] += adcValue[4];
      currentOffset[1] += adcValue[5];
      currentOffsetCnt++;
      if (currentOffsetCnt == 100)
      { // reduce to average
        currentOffset[0] /= 100;
        currentOffset[1] /= 100;
      }
    }
  }
  else
  { // calculate current in amps
    // average over 10 samples
    current =  ((adcValue[4] - currentOffset[0]) + 
               (adcValue[5] - currentOffset[1])) * currentFactor;
    currentf = currentf * filt2 + current * filt1;
  }
  // battery voltage
  // divided by 18kOhm and 2kOhm, i.e. 1V oto AD is 10V battery
  batteryVoltage = adcValue[6] * 3.3 / 4096.0 * (18.0 + 2.0)/2.0;
  batteryVoltagef = batteryVoltagef * filt2 + batteryVoltage * filt1;
  // temperature
  float temp = float(adcValue[2]) *3.3 / 4096.0 * 100.0;
  temp1f = temp1f * filt2 + temp * filt1;
  temp = float(adcValue[3]) *3.3 / 4096.0 * 100.0;
  temp2f = temp2f * filt2 + temp * filt1;
  // get time passed since last ADC cycle
//   dt = ARM_DWT_CYCCNT - adcIntervalStart;
//   adcIntervalStart = ARM_DWT_CYCCNT;
//  adcIntervalUs = dt/(F_CPU/1000000);
  if (dataLog and dataIdx < MAX_DATA_CNT)
  { // check time
    int32_t tm = ARM_DWT_CYCCNT/(F_CPU/100000); // 100th of a ms
    int32_t dti = tm - dataLogLast;
//     int32_t dt0 = dti;
    if (dti < 0)
    { // time roll over
      dataLogLast = tm;
      dataLogLastTm = tm - dataInterval;
    }
    else if (dti > 10*dataInterval)
    { // first time
      dataLogLast = tm;
      logTime = 0;
      dti = dataInterval;
      dataLogLastTm = tm - dataInterval;
    }
    //
    if (dti >= dataInterval)
    { // save to buffer
      data[dataIdx].time = logTime;
      data[dataIdx].rps[0] = rps[0];
      data[dataIdx].rps[1] = rps[1];
      data[dataIdx].esc = esc->escVal[0];
      data[dataIdx].current = currentf;
      data[dataIdx].battery = batteryVoltagef;
      data[dataIdx].tempMotor = temp1f; // float(adcValue[2]) *3.3 / 4096.0 * 100.0;
      data[dataIdx].tempEsc = temp2f; // float(adcValue[3]) *3.3 / 4096.0 * 100.0;
//       data[dataIdx].dt = dataLogLast;
//       data[dataIdx].dt0 = dt0;
//       data[dataIdx].tm = tm;
      dataIdx++;
      logTime += tm - dataLogLastTm;
      dataLogLastTm = tm;
      if (dataIdx >= MAX_DATA_CNT)
      { // buffer full?
        dataLog = false;
        usb_send_str("\nlogfull\r\n");
      }
      if (logTime > 0)
        dataLogLast += dataInterval;
    }
  }
  // start new ADC cycle
  adcSeq = 0;
  // timing of all ADC operations
  adcStartTime = ARM_DWT_CYCCNT;
  adc->startSingleRead ( adcPin[adcSeq] );
  digitalWriteFast ( PIN_ADC_CONV_OUT, HIGH );
  adcTickCnt++;
}

void USensor::readSensorTick()
{ // all ADC is interrupt driven
  if (logStepping)
  {
    int32_t dt = ARM_DWT_CYCCNT - logStartTime;
    if (dt < 0)
    {
      dt += 1 << 31;
      usb_send_str("# log seq - roll over corrected\r\n");
    }
    float ms = dt/float(F_CPU/1000);
    if (tickCnt % 50 == 10)
    {
      const int MSL = 220;
      char s[MSL];
      snprintf(s, MSL, "# Seq tick %d: ms=%.1f, i=%.0f, size=%d vals, step %d cnt, PWM=%d (0..1024), start=%d, dt=%ld\r\n",
               tickCnt, ms,
               logInterval, 
               logStepSize, 
               logStep, 
               logStepSize * logStep + logStartValue/100, 
               logStartValue/100, 
               dt);
      usb_send_str(s);
    }
    if ((ms > logInterval*logStep) and logStep <= logSteps)
    {
      esc->setEscPWM(0, logStepSize * logStep + logStartValue/100, 1, 0); 
      logStep++;
      usb_send_str("#---- step ----\n");
    }
    if (not dataLog)
    {
      logStepping = false;
      // stop motor
      esc->setEscPWM(0, 0, 1, 0); 
      usb_send_str("# log sequence ended\r\n");
    }
  }
  tickCnt++;
}

void USensor::sendTempData()
{
  const int MSL = 260;
  char s[MSL];
  // konverter til oC
  float temp1 = float(adcValue[2]) *3.3 / 4096.0 * 100.0;
  float temp2 = float(adcValue[3]) *3.3 / 4096.0 * 100.0;
  snprintf(s, MSL, "tmp %g %g %d %d\r\n", temp1, temp2, adcValue[2], adcValue[3]);
  usb_send_str(s);
}  

void USensor::sendSensorData()
{
  const int MSL = 260;
  char s[MSL];
  //
  snprintf(s, MSL, "sen %g %g %d\r\n"
                   "# idle limit: %d,\r\n"
                   "# adc time=%ldus, cycle: %ldus, pin1: %ld - %ld = %ld pin2: %ld - %ld = %ld\n\r", 
           rps[0], rps[1], adcCCV, spanLimit, 
           adcConvertUs, adcIntervalUs,
           rpsMax[0], rpsMin[0], rpsMax[0] - rpsMin[0] , 
           rpsMax[1], rpsMin[1], rpsMax[1] - rpsMin[1]);
  usb_send_str(s);
//  uint16_t a, b;
//   a = analogRead(A0);
//   b = analogRead(A1);
  snprintf(s, MSL, "# ADC raw: %d %d %d %d\r\n", adcValue[0], adcValue[1], adcValue[2], adcValue[3]);
  usb_send_str(s);
}

void USensor::sendCurrentVolt()
{
  const int MSL = 260;
  char s[MSL];
  //
  snprintf(s, MSL, "bat %g %g %d %d %d\r\n", batteryVoltage, currentf, adcValue[4], adcValue[5], adcValue[6]);
  usb_send_str(s);
}

void USensor::sendCurrentVals()
{
  const int MSL = 260;
  char s[MSL];
  //
  snprintf(s, MSL, "curDebug %g %d %d %g %ld %ld\r\n", current, adcValue[4], adcValue[5], 
           currentFactor, currentOffset[0], currentOffset[1]);
  usb_send_str(s);
}

void USensor::adcClear()
{
  for (int i = 0; i < ADC_NUM_ALL; i++)
    adcValue[i] = 0;
  usb_send_str("\n\r# cleared\r\n");
}


void USensor::startLog(float rate)
{
  dataInterval = rate*100; // ms
  dataIdx = 0;
  dataLog = true;
  logTime = 0;
  const int MSL = 90;
  char s[MSL];
  snprintf(s, MSL, "# stating log %g ms (dataInterval=%f ms)\r\n", rate, dataInterval/100.0);
  usb_send_str(s);
}


void USensor::getLog()
{
  if (dataIdx == 0)
    usb_send_str("% no data in log\r\n");
  else
  {
    const int MSL = 300;
    char s[MSL];
    //
    snprintf(s, MSL, "%% data log for sensor has %d values (interval=%gms)\n\r", dataIdx, dataInterval/100.0);
    usb_send_str(s);
    usb_send_str("% data format:\r\n");
    usb_send_str("% 1   Time (ms)\r\n");
    usb_send_str("% 2,3 (a,b) calculated rotations per second\r\n");
    usb_send_str("% 4   ESC PW (ms)\r\n");
    usb_send_str("% 5   Battery voltage (V)\r\n");
    usb_send_str("% 6   Battery current (A)\r\n");
    usb_send_str("% 7   Motor temp (deg C)\r\n");
    usb_send_str("% 8   ESC temp (deg C)\r\n");
    usb_send_str("% \r\n");
    //
    for (int i = 0; i < dataIdx; i++)
    {
      snprintf(s, MSL, "%.2f %.2f %.2f %.3f %.2f %.2f %.1f %.1f\r\n",
               float(data[i].time)/100.0,
               data[i].rps[0], data[i].rps[1],
               data[i].esc,
               data[i].battery,
               data[i].current,
               data[i].tempMotor,
               data[i].tempEsc
//                data[i].dt,
//                data[i].dt0,
//                data[i].tm
      );
      usb_send_str(s);
    }
    usb_send_str("logdata end\n");
  }
}

void USensor::sendTimingInfo()
{
  const int MSL = 260;
  char s[MSL];
  //
//   float c_us = (F_CPU / 1000000); // CPU cycles per us
  float sense_us = controlUsedTime[0] / c_us;
  float ctrl_us = controlUsedTime[1] / c_us;
  float cycle_us = controlUsedTime[2] / c_us;
//   float adc_us = controlUsedTime[3] / c_us;
  float cycle_actual10 = controlUsedTime[4] / c_us;
//   int adc_per_cycle = float(CONTROL_PERIOD_10us * cycleTimerInUs) / adc_us;
  //
  snprintf(s, MSL, "tim %.2f %.2f %.2f %.2f %.2f %d\r\n", cycle_actual10,
           sense_us, ctrl_us, cycle_us, adc_us, adc_per_cycle);
  usb_send_str(s);
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
void USensor::startMotorLogSequence(char * buf)
{
  char * p1 = &buf[4];
  float interval = strtof(p1, &p1);
  int steps = strtol(p1, &p1, 10);
  int final = strtol(p1, &p1, 10);
  float logms = strtof(p1, &p1);
  if (logms > 0.0)
  {
    startLog(logms);
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
    usb_send_str(s);
  }
  else
  {
    usb_send_str("# failed to understand all parameters in:");
    usb_send_str(buf);
    usb_send_str("\n\r");
  }
}
