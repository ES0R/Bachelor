/*
 * Read sensors - e.g. for RPM detection
 * Copyright (C) 2021  DTU
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


/**
 * @todo write docs
 */
class USensor
{
public:
  // current ADC measurement index - shifted in interrupt-routine
  int adcSeq;
//   bool adcFinished = true;
  // resulting rotation speed
  static const int RPS_SENSORS = 2;
  float rps[RPS_SENSORS] = {0.0};
  bool rpmSensorPresent = true;
  int spanLimit = 100;
  // battery monitoring
  int lipoCellCount = 0;
  float batteryVoltage = 0;
  float batteryVoltagef = 0;
  const float batteryVoltageMinimum = 3.35; // per cell
  const float batteryVoltageMaximum = 4.3; // per cell
  //
  uint32_t controlUsedTime[11] = {0}; 
  float c_us = (F_CPU / 1000000); // CPU cycles per us
  float adc_us;
//   int adc_per_cycle = 2;
  /// filtered version of current
  float currentf = 0.0; 
  /// motor and ESC temp (filtered)
  float temp1f = 0, temp2f = 0; 
  bool tempSensorPresent = true;
  bool currentSensorPresent = true;
  //
protected:
  // RPS sensor data
  int32_t rpsMin[RPS_SENSORS] = {0};
  int32_t rpsMax[RPS_SENSORS] = {0};
  bool rpsVal[RPS_SENSORS] = {false};
  // RPS calculation result
  uint32_t edgeTimeUp[RPS_SENSORS] = {0};
  uint32_t edgeTimeDown[RPS_SENSORS] = {0};
  int32_t span[RPS_SENSORS];
  int32_t midt[RPS_SENSORS];
  // RPS min/max filter
  const int minMaxFilt = 100;
  // ADC measurements each sample
  uint32_t adcIntervalStart = 0;
  int adcTickCnt = 0;
  // current offset
  int32_t currentOffset[2] = {0,0};
  int currentOffsetCnt  = 0;
  // using ASC711EX at 5V should give 136mV per amp - for 15.5 A version
  // using ASC711EX at 5V should give  68mV per amp - for 30 A version
  // AD is 12 bit and ref is 3.3V
  // sensor is connected negative, i.e. from 2.5V and down for increased current
  // and 
  //const float currentFactor = -3.3/4096.0 / 0.068; 
  const float currentFactor = -3.3/4096.0 / 0.136; 
  float current = 0.0;
  ///
  /// Variables for motor 1 ESC stepping for good log response
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
  /// Battery
  /// battery low counter
  int batteryLow = 0;
  /// running on battery - else on power-supply
  bool onBattery = true;
  
  
public:
  /**
   * Default constructor
   */
  USensor();
  /**
   * initialize ADC interrupt */
  void InitSensor();
  /**
   * decode data command */
  bool decode(const char * buf);
  /**
   * send help on messages */
  void sendHelp();
  /**
   * Do sensor processing - at tick time */
  void readSensorTick(uint32_t tickCnt);
  /**
   * Check if battery voltage is critical low. */
  bool isBatteryCriticallyLow();
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
   * set battery low voltage check active
   * */
  void setOnBattery(const char * buf);
  /**
   * sensors installed message */
  void setSensorsInstalled(const char * buf);
  /**
   * debug
   * */
  void adcClear();
  /**
   * start log with rate in ms */
  //void startLog(float rate);
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
  void startMotorLogSequence(const char * buf);
  /**
   * get log */
  //void getLog();
  /**
   * save steering configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
};

#endif // SENSOR_H
