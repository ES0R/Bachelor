/*
 * Read sensors - e.g. for RPM detection
 * Copyright (C) 2020  DTU
 * jca@elektro.dtu.dk
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

#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"

// AD conversion
extern ADC * adc;       // ADC class
//extern bool adcHalf;    // for double ADC conversion for LS
extern int adcPin[ADC_NUM_ALL]; // pin sequence for ADC MUX


typedef struct aaa
{
  int32_t time;   // time in us (may fold)
  float    rps[2]; // calculated cycles per second
  float    esc;    // ESC pulse in ms.
  float current = 0;
  float battery = 0;
  float tempMotor =0;
  float tempEsc = 0;
//   int32_t dt = 0;
//   int32_t dt0 = 0;
//   int32_t tm = 0;
} SensorData;

// assumed size 36 bytes
static const int MAX_DATA_CNT = 100000/sizeof(SensorData);
extern SensorData data[MAX_DATA_CNT];
extern int dataIdx;
// extern bool dataLog;
// extern int32_t dataInterval;

/**
 * @todo write docs
 */
class USensor
{
public:
  // current ADC measurement index - shifted in interrupt-routine
  int adcSeq;
  // resulting rotation speed
  static const int RPS_SENSORS = 2;
  float rps[RPS_SENSORS] = {0.0};
  uint32_t adcIntervalUs = 1;
  uint32_t adcConvertUs = 1;
  uint32_t adcStartTime = 0;
  int spanLimit = 100;
  float batteryVoltage = 0;
  float batteryVoltagef = 0;
  int32_t logTime = 0;
  bool dataLog = false;
  int32_t dataInterval = 250; //millisecond * 100
  int32_t dataLogLast = 0; // millisecond * 100
  int32_t dataLogLastTm = 0; // millisecond * 100
  // 0: 1 ADC, 1: ADC+Ctrl, 2: full sample 3: ADC cycle time, 4 ctrl cycle time; [CPU clocks]
  uint32_t controlUsedTime[5] = {0}; 
  float c_us = (F_CPU / 1000000); // CPU cycles per us
  float adc_us;
  int adc_per_cycle;
  //
protected:
  // RPM sensor data
  int32_t rpsMin[RPS_SENSORS];
  int32_t rpsMax[RPS_SENSORS];
  bool rpsVal[RPS_SENSORS] = {false};
  // calculation result
  uint32_t edgeTimeUp[RPS_SENSORS] = {0};
  uint32_t edgeTimeDown[RPS_SENSORS] = {0};
  uint32_t adcIntervalStart = 0;
  // debug
  int adcTickCnt = 0;
  int tickCnt = 0;
  int32_t span[RPS_SENSORS];
  int32_t midt[RPS_SENSORS];
  // rps min/max filter
  const int minMaxFilt = 100;
  // current offset
  int32_t currentOffset[2] = {0,0};
  int currentOffsetCnt  = 0;
  // using ASC711EX at 5V should give 136mV per amp
  // AD is 12 bit and ref is 3.3V
  // sensor is connected negative, i.e. from 2.5V and down for increased current
  const float currentFactor = -3.3/4096.0 / 0.136; 
  float current = 0.0;
  /// filtered version
  float currentf = 0.0; 
  /// motor and ESC temp
  float temp1f, temp2f; 
  // log stepping
  /// time interval (ms) for logging
  float logInterval; 
  /// total steps - equal size
  int logSteps;      
  /// step number
  int logStep;    
  /// value increase each step (0..1024)
  int logStepSize;
  /// CPU count at start
  int logStartTime;  
  /// start ESC value (0..1024)
  int logStartValue; 
  /// is stepping active
  bool logStepping = false; 
  
  
public:
  /**
   * Default constructor
   */
  USensor();
  /**
   * initialize ADC interrupt */
  void InitSensor();
  /**
   * Do sensor processing - at tick time */
  void readSensorTick();
  /**
   * process ADC values - ADC cycle is finished */
  void adcTick();
  /**
   * send sensor data */
  void sendSensorData();
  /**
   * send voltage and current 
   * */
  void sendCurrentVolt();
  /**
   * debug af current calculation */
  void sendCurrentVals();
  /**
   * sendTimingInfo */
  void sendTimingInfo();
  /**
   * send temperatur
   * tmp t1 t2 ad19 ad20 */
  void sendTempData();
  /**
   * debug
   * */
  void adcClear();
  /**
   * start log with rate in ms */
  void startLog(float rate);
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
   * i.e. is current value is 100, then start log, wait 220ms, 
   * step to 200, wait, 300, wait, 400, 
   * wait, 500, wait until log is full, then stop */
  void startMotorLogSequence(char * buf);
  /**
   * get log */
  void getLog();
};

#endif // SENSOR_H
