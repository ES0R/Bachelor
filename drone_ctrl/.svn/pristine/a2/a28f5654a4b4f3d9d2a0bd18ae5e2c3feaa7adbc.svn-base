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

#ifndef ULOGGER_H
#define ULOGGER_H

#include <stdint.h>
#include "main.h"

class UControlBase;

// size of RAM allocated for logger (bytes)
#define MAX_LOGGER_SPACE 60000

struct SensorData
{
  int32_t time;   // time in us (may fold)
  float    rps[2]; // calculated cycles per second
  float    esc;    // ESC pulse in ms.
  float current = 0;
  float battery = 0;
  float tempMotor =0;
  float tempEsc = 0;
};

struct ImuLogData
{
  int32_t time;
  float acc[3]; // m/s2
  float gyro[3];// deg/s
  float pose[3];// radians (roll pitch yaw)
  float height; // m (pressure altitude)
  float heightf; // height filtered (m)
  float heightOffset; // height offset (m)
  float heightVelocity; // height offset (m)
  float sonarheight;  // from ultrasound measurement
  int16_t calibCnt;
  float temp;   // C (pressure sensor)
};

struct CtrlLogData
{
  int32_t time;
  float ref;  // reference input
  float ref2; // after pre-filter
  float ep;   // Error - after KP
  float up;   // control signal without I part (P and Lead/lag)
  float ui;   // control signal I-part
  float u;    // control signal after limiter
  float m;    // measurement value
  float m2;   // measurement after (lead/lag) filter
  float uf;   // output of feed forward
};

struct PoseCtrlData
{
  int32_t time;
  uint8_t state;
  uint8_t flightMode = 0; // not implemented yet
  float refHeight;
  float refRoll;
  float refPitch;
  float refYaw;
  float mixerHeight;
  float mixerRoll;
  float mixerPitch;
  float mixerYaw;
  float posHeight;
  float posRoll;
  float posPitch;
  float velYaw;
  float heightVel;
};

struct PitchCtrlData
{
  int32_t time;
  uint8_t state;
  uint8_t flightMode = 0; // not implemented yet
  float refPitchAngle;
  float refPitchRate;
  float mixerPitch;
  float gyroPitch;// deg/s
  float pitchAngle;
};

struct TimingData
{
  int32_t time;
  uint32_t adcSample;
  uint16_t adcPerCycle;
  uint16_t dummy;
  uint32_t cycleTime;
  uint32_t sensorFilterTime;
  uint32_t controlTime;
  uint32_t cycleProcessTime;
  uint32_t cycleStartTime;
  uint32_t time8;
  uint32_t time9;
  uint32_t time10;
  uint32_t readImuTime;
  uint32_t endFilterTime;
  float gyro, acc; // sample to see change
  uint8_t cntGyro, cntAccMag, cntAltitude;
};

// assumed size 36 bytes
// static const int MAX_DATA_CNT = 100000/sizeof(SensorData);
// extern SensorData data[MAX_DATA_CNT];
// extern int dataIdx;

extern uint8_t dataBuffer[MAX_LOGGER_SPACE];

class ULogger
{
public:
  /**
   * constructor */
  ULogger();
  /**
   * Setup */
  void setup()
  { // nothing to do
    // debug
//     startImuLog();
    // debug end
  }
  /**
   * send help options */
  void sendHelp();
  /**
   * decode logger command */
  bool decode(const char * buff);
  /**
   * Mixer tick
   * Transfers control output (roll, pitch, yaw, height)
   * to ESC values */
  void tick(uint32_t tickCnt);
  /**
   * Get log of the type last logged */
  bool sendLogToClient(int ticks_with_full_buffer, uint32_t time_to_send_last_in_us, int buff_free);
  /**
   * set sample rate */
  void setSampleRate(float ms)
  {
    interval_ms = ms;
  }
  /**
   * log motor 1 sensor values */
  void startSensorLog();
  /**
   * log IMU values */
  void startImuLog();
  /**
   * log values from one PID control block */
  void startCtrlLog();
  /**
   * log values from control ref, input to mixer and motor */
  void startPoseCtrlLog();
  /**
   * log values from control ref, input to mixer and motor */
  void startPitchCtrlLog(float sampleMs);
  /**
   * log timing */
  void startTimingLog(float sampleMs);
  /**
   * start USB IO log */
  void startUSBLog();
  /**
   * restart last log */
  void restartLog();
  /**
   * add a block of chars to log */
  void addUSBLogEntry(const char * str, int n, uint32_t preIOhb, int newEntry);
  /// is this log-type active
  bool logUSB = false;
  
  
  /**
   * stop logging */
  void stop();
  /**
   * Get current logger index */
  inline int getLoggerIdx()
  {
    return dataIdx;
  }
  /**
   * get maximum logger index */
  inline int getLoggerIdxMax()
  {
    return dataIdxMax;
  }
  /**
   * Id logging
   * \returns trus when logging. */
  inline bool isLogging()
  {
    return dataLog;
  }
  /// sending log to client
  bool dataLogOut = false; 
  
protected:
  /**
   * save motor 1 sensor data entry */
  void addSensorLogEntry();
  /**
   * save IMU data entry */
  void addImuLogEntry();
  /**
   * save desired control interface entry */
  void addCtrlLogEntry();
  /**
   * save desired control interface entry */
  void addPoseCtrlLogEntry();
  /**
   * save pitch data to log */
  void addPitchCtrlLogEntry();
  /**
   * save timing data to log */
  void addTimingLogEntry();
  /**
   * should be called by specific log-starter
   * */
  void initLog();
  /**
   * get log full from IMU log structure */
  void getImuLog();
  /**
   * get log full of motor 1 sensor values */
  void getSensorLog();
  /**
   * get log full of one controller */
  void getCtrlLog();
  /**
   * get log full of pose control values */
  void getPoseCtrlLog();
  /**
   * get pitch log */
  void getPitchCtrlLog();
  /**
   * Get timing log */
  void getTimingLog();
  /**
   * Get USB log */
  void getUSBLog();
  
  
  
protected:
//   uint8_t * dataBuffer = nullptr; //[MAX_LOGGER_SPACE];
  //
  bool dataLog = false; // main flag for logging
  int32_t logTime;  // for time interval calculation
  int32_t logTimeStart = 0;
  int dataInterval = 1;
  float interval_ms = 1; // log interval in ms
  int dataIdx = 0;       // status for logging
  int dataIdxOut = 0;    // sending log to client
  int dataIdxMax = 10;   // set when log starts
  float pitchRateFilt = 0;   // filtered version of pitch rate
  int skipped_ticks = 0; // skipped ticks during get-log with no log-line send (buffer full)
  uint32_t time_to_send_max = 0;
  // USB buffer statistics
  int buffer_max = 0;
  int buffer_min = 100000;
  // flow control
public:
  bool flowCtrl = false;
  int  receivedLine = 0;
  int  receivedLineInterval = 10;
protected:
  //
  bool logSensor;    // Sensor logging active
  SensorData * sdata = (SensorData *) dataBuffer;
  // 
  bool logImu;    // Sensor logging active
  ImuLogData * idata = (ImuLogData *) dataBuffer;
  // 
  bool logPose;    // Sensor logging active
  PoseCtrlData * pdata = (PoseCtrlData *) dataBuffer;
  //
  bool logCtrl;    // Sensor logging active
  UControlBase * interface = NULL;
  CtrlLogData * cdata = (CtrlLogData *) dataBuffer;
  //
  bool logPitch = false;
  PitchCtrlData * pitchData = (PitchCtrlData * ) dataBuffer;
  //
  bool logTiming = false;
  TimingData * timingData = (TimingData *) dataBuffer;
  // 
  const char * usbIOData = (const char *) dataBuffer;
  // tick count at start of logging
  uint32_t usbLogStart;
};

#endif
