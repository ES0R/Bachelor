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

#include <string.h>
 #include "control.h"
#include "logger.h"
#include "command.h"
#include "uesc.h"
#include "sensor.h"
#include "upropshield.h"
#include "controlbase.h"
#include "ustate.h"
#include "mixer.h"
#include "ultrasound.h"
#include "uheight.h"
#include "logger.h"


uint8_t dataBuffer[MAX_LOGGER_SPACE];


ULogger::ULogger()
{
}

void ULogger::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  snprintf(reply, MRL, "#   log motor m  Start motor sensor log every m ms (>=0.1ms)\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   log imu m    Start imu log every m ms (>=2.5ms)\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   log XXX m    Start control log every m ms (XXX = vroll, roll, vpitch, pitch, vyaw, height)\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   log pose m   Start pose control log every m ms (>=2.5ms)\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   log get      Get log (has=%d/%d entries)\r\n", logger->getLoggerIdx(), logger->getLoggerIdxMax());
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   log usb      log USB io\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   log stop     Stop current log (no effect if log is full already)\r\n");
  command->usb_send_str(reply);
}

void ULogger::stop()
{
  dataLog = false;
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "message stopping log at index %d\r\n", dataIdx);
  command->usb_send_str(s);
}


void ULogger::tick(uint32_t tickCnt)
{ // check RC-logging switch
  if(rc->logswitch() and not isLogging())
  { // log always run until log is full
    setSampleRate(10); // ms - not sure
    startPoseCtrlLog();
    // tell Raspberry pi we are logging (log optitrack and gnss (GPS))
    command->usb_send_str("log opti gnss\n");
    command->sendInfoAsCommentWithTime("pose-log started by switch\n", "");
//    startPitchCtrlLog(5);
//    command->usb_send_str("# pitch-log started by switch\n");
  } 
  //
  if (isLogging())
  { // logging active
    bool logNow;
    if (dataInterval <= 1)
      logNow = true;
    else if (tickCnt % dataInterval == 0)
      logNow = true;
    else
      logNow = false;
    // call relevant logger interface
    if (logPitch)
    { // filter gyro data
      pitchRateFilt = (pitchRateFilt * (dataInterval - 1) + imu->gyro[0]) / dataInterval;
    }
    // particular dataset
    if (logNow)
    {
      if (logUSB)
        dataIdx--; // adds directly from IO, so counter index increase
      else if (logSensor)
        addSensorLogEntry();
      else if (logImu)
        addImuLogEntry();
      else if (logCtrl)
        addCtrlLogEntry();
      else if (logPose)
        addPoseCtrlLogEntry();
      else if (logPitch)
        addPitchCtrlLogEntry();
      else if (logTiming)
        addTimingLogEntry();
      else
      {
        command->usb_send_str("# Nothing to log - unknown interface\n");
        stop();
        return;
      }
      //
      // post entry processing
      dataIdx++;
      if (dataIdx % 30 == 0)
      {
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "# log index %d/%d\n", dataIdx, dataIdxMax);
        command->usb_send_str(s);
      }
      if (dataIdx >= dataIdxMax)
      { // buffer full, stop logging
        stop();
        command->usb_send_str("logfull\r\n");
      }
    }
  }
}

void ULogger::initLog()
{
  dataInterval = rintf(interval_ms/1000.0/(SAMPLETIME)); // in number of samples at current sample interval
  dataIdx = 0;
  dataLog = true;
  // remember to off all (other) log-flags (@todo replace with an enum)
  logSensor = false;
  logImu = false;
  logCtrl = false;
  logPose = false;
  logPitch = false;
  logUSB = false;
  logTimeStart = hb10us;
  memset(dataBuffer, '+', MAX_LOGGER_SPACE);
  // 
  const int MSL = 90;
  char s[MSL];
  snprintf(s, MSL, "# stating log %g ms (dataInterval=%d)\r\n", interval_ms, dataInterval);
  command->usb_send_str(s);
}

void ULogger::restartLog()
{
  if (not (logCtrl or logImu or logPitch or logPose or logSensor or logTiming))
    // no log ever started, use sensor log
    startSensorLog();
  else if (not dataLog)
  { // not logging - restart last log
    dataIdx = 0;
    dataLog = true;
    logTimeStart = hb10us;
  }
}


void ULogger::startSensorLog()
{
  initLog();
  dataIdxMax = MAX_LOGGER_SPACE / sizeof(SensorData);
  logSensor = true;
  //
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "message Sensor log %d entries (%d bytes)\r\n", dataIdxMax, dataIdxMax * sizeof(SensorData));
  command->usb_send_str(s);
}

bool ULogger::sendLogToClient(int ticks_with_full_buffer, uint32_t time_to_send_last_in_us, int buff_free)
{ 
  bool clearToSend = true;
  if (dataIdx == 0)
  {
    command->usb_send_str("% no data in log :)\r\n");
    dataLogOut = false;
  }
  else
  { // send one line to client
    const int MSL = 200;
    char s[MSL];
    skipped_ticks += ticks_with_full_buffer;
    if (time_to_send_last_in_us > time_to_send_max)
      time_to_send_max = time_to_send_last_in_us;
    if (buff_free > buffer_max)
      buffer_max = buff_free;
    if (buff_free < buffer_min)
      buffer_min = buff_free;
    if (dataIdxOut % 10 == 0)
    {
      snprintf(s, MSL, "# log idx=%d/%d, skip %d/10 lines, %ldus, min=%d, max=%d free, flow=%d, rec-idx=%d\r\n", dataIdxOut, dataIdx, skipped_ticks, time_to_send_max, buffer_min, buffer_max, flowCtrl, receivedLine);
      command->usb_send_str(s);
      skipped_ticks = 0;
      time_to_send_max = 0;
      buffer_max = 0;
      buffer_min = 100000;
    }
    // flow control (flow control failed to work in all cases, skipped)
    if (false and flowCtrl)
    {
      if (receivedLine < (dataIdxOut - (2 + receivedLineInterval)))
        clearToSend = false;
    }
    //
    if (clearToSend)
    {
      if (logSensor)
        getSensorLog();
      else if (logImu)
        getImuLog();
      else if (logCtrl and interface != NULL)
        getCtrlLog();    
      else if (logPose)
        getPoseCtrlLog();
      else if (logPitch)
        getPitchCtrlLog();
      else if (logTiming)
        getTimingLog();
      else if (logUSB)
        getUSBLog();
      else
      {
        command->usb_send_str("% Log empty (unknown log structure)\r\n");
      }
      dataIdxOut++;
      if (dataIdxOut >= dataIdx)
      {
        dataLogOut = false;
        skipped_ticks = 0;
        snprintf(s, MSL, "message log end at idx=%d\r\n", dataIdxOut);
        command->usb_send_str(s);
      }
    }
  }
  return clearToSend;
}

void ULogger::getSensorLog()
{
  const int MSL = 300;
  char s[MSL];
  const int i = dataIdxOut;
  if (dataIdxOut == -1)
  {
    //
    snprintf(s, MSL, "%% data log for sensor has %d values (interval=%gms)\n\r", dataIdx, dataInterval/100.0);
    command->usb_send_str(s);
    command->usb_send_str("% data format:\r\n");
    command->usb_send_str("% 1   Time (ms)\r\n");
    command->usb_send_str("% 2,3 (a,b) calculated rotations per second (2 sensors)\r\n");
    command->usb_send_str("% 4   ESC value (0..1024 => 1-2ms) (motor 1)\r\n");
    command->usb_send_str("% 5   Battery voltage (V)\r\n");
    command->usb_send_str("% 6   Battery current (A)\r\n");
    command->usb_send_str("% 7   Motor temp (deg C)\r\n");
    command->usb_send_str("% 8   ESC temp (deg C)\r\n");
    command->usb_send_str("% \r\n");
  }
  else
  {
//     //
//     for (i = 0; i < dataIdx; i++)
//     {
    snprintf(s, MSL, "%.2f %.2f %.2f %.3f %.2f %.2f %.1f %.1f %d\r\n",
              float(sdata[i].time)/100.0,
              sdata[i].rps[0], sdata[i].rps[1],
              sdata[i].esc,
              sdata[i].battery,
              sdata[i].current,
              sdata[i].tempMotor,
              sdata[i].tempEsc,
              i
              //                data[i].dt,
              //                data[i].dt0,
              //                data[i].tm
    );
    command->usb_send_str(s, true);
  }
}

void ULogger::addSensorLogEntry()
{
  sdata[dataIdx].time = hb10us - logTimeStart;
  sdata[dataIdx].rps[0] = sensor->rps[0];
  sdata[dataIdx].rps[1] = sensor->rps[1];
  if (esc->escManual[0])
    sdata[dataIdx].esc = esc->escRefMan[0];
  else
    sdata[dataIdx].esc = esc->escRef[0];
  sdata[dataIdx].current = sensor->currentf;
  sdata[dataIdx].battery = sensor->batteryVoltagef;
  sdata[dataIdx].tempMotor = sensor->temp1f; // float(adcValue[2]) *3.3 / 4096.0 * 100.0;
  sdata[dataIdx].tempEsc = sensor->temp2f; // float(adcValue[3]) *3.3 / 4096.0 * 100.0;
}

void ULogger::getImuLog()
{
  const int MSL = 350;
  char s[MSL];
  const int i = dataIdxOut;
  if (dataIdxOut == -1)
  {
    snprintf(s, MSL, "%% data log for IMU has %d values (interval=%g ms)\n\r", dataIdx, dataInterval*SAMPLETIME*1000);
    command->usb_send_str(s);
    snprintf(s, MSL, "%% updating using compass=%d\r\n", imu->useMag);
    float offs[9];
    command->usb_send_str(s);
    imu->getCalibration(offs);
    snprintf(s, MSL, "%% acc    offset (%f, %f, %f)\r\n", offs[0], offs[1], offs[2]);
    command->usb_send_str(s);
    snprintf(s, MSL, "%% gyro   offset (%f, %f, %f)\r\n", offs[3], offs[4], offs[5]);
    command->usb_send_str(s);
    snprintf(s, MSL, "%% height offset (%f)\r\n", imu->altOffset);
    command->usb_send_str(s);
    command->usb_send_str("% data format:\r\n");
    command->usb_send_str("% 1      Time (ms)\r\n");
    command->usb_send_str("% 2-4    Acc data (m/s2)\r\n");
    command->usb_send_str("% 5-7    Gyro data (deg/s)\r\n");
    command->usb_send_str("% 8      Altitude filtered (m)\r\n");
    command->usb_send_str("% 9      Altitude raw (m)\r\n");
    command->usb_send_str("% 10     Altitude offset (m)\r\n");
    command->usb_send_str("% 11     Altitude ultrasound (m)\r\n");
    command->usb_send_str("% 12     Altitude velocity (m/s)\r\n");
    command->usb_send_str("% 13-15  Roll, pitch, Yaw (radians)\r\n");
    command->usb_send_str("% 16     Temperature (deg C)\r\n");
    command->usb_send_str("% 17     calibration sample\r\n");
    command->usb_send_str("% \r\n");
  }
  else
  {
    snprintf(s, MSL, "%.2f %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %d\r\n",
               float(idata[i].time)/100.0,
               idata[i].acc[0], idata[i].acc[1], idata[i].acc[2],
               idata[i].gyro[0], idata[i].gyro[1],idata[i].gyro[2],
               idata[i].heightf, idata[i].height,
               idata[i].heightOffset, idata[i].sonarheight, idata[i].heightVelocity,
               idata[i].pose[0], idata[i].pose[1], idata[i].pose[2],
               idata[i].temp, idata[i].calibCnt
      );
      command->usb_send_str(s, true);
  }
}

void ULogger::startImuLog()
{
  initLog();
  dataIdxMax = MAX_LOGGER_SPACE / sizeof(ImuLogData);
  logImu = true;  
  //
  command->usb_send_str("message starting Imu log\r\n");
}

void ULogger::startCtrlLog()
{
  initLog();
  dataIdxMax = MAX_LOGGER_SPACE / sizeof(CtrlLogData);
  logCtrl = true;  
  //
  command->usb_send_str("message starting Ctrl log\r\n");
}

void ULogger::addImuLogEntry()
{
  idata[dataIdx].time = hb10us - logTimeStart;
  idata[dataIdx].acc[0] = imu->acc[0];
  idata[dataIdx].acc[1] = imu->acc[1];
  idata[dataIdx].acc[2] = imu->acc[2];
  idata[dataIdx].gyro[0] = imu->gyro[0];
  idata[dataIdx].gyro[1] = imu->gyro[1];
  idata[dataIdx].gyro[2] = imu->gyro[2];
  idata[dataIdx].height = imu->altitude - imu->altOffset;
  idata[dataIdx].heightf = hgt->height;
  idata[dataIdx].heightOffset = imu->altOffset;
  idata[dataIdx].heightVelocity = hgt->heightVelocity;
  idata[dataIdx].sonarheight = uhgt->sonarHeight;
  idata[dataIdx].pose[0] = imu->roll;
  idata[dataIdx].pose[1] = imu->pitch;
  idata[dataIdx].pose[2] = imu->yaw;
  idata[dataIdx].temp = imu->getTemperature();
  idata[dataIdx].calibCnt = imu->offsetSampleCnt;
}

bool ULogger::decode(const char * buff)
{
  const char * p1 = buff;
  while (*p1 == ' ')
    p1++;
  const char * a = p1;
  if (strncmp(p1, "get", 3) == 0)
  {  // download most recent log (if any)
    dataLogOut = true;
    dataIdxOut = -1;
    flowCtrl = true;
  }
  else if (strncmp(p1, "got", 3) == 0)
  { // we are using flow control
    p1 += 3;
    if (strlen(p1) > 4)
    {
      receivedLine = strtol(p1, (char **)&p1, 10);
      receivedLineInterval = strtol(p1, (char **)&p1, 10);
      if (receivedLine == 0 or receivedLineInterval == 0)
        flowCtrl = false;
      else
        flowCtrl = true;
    }
    // debug message
    if (false)
    {
      const int MSL = 100;
      char s[MSL];
      snprintf(s, MSL, "# now sending %d (on=%d at rx of log got %d %d)\n", dataIdxOut, dataLogOut, receivedLine, receivedLineInterval);
      command->usb_send_str(s);
    }
  }
  else
  { // go to next parameter
    const int MSL = 100;
    char s[MSL];
    interface = NULL;
    p1 = strchr(a, ' ');
    if (p1 != NULL)
    { // there is a sample time value
      interval_ms = strtof(p1, (char **)&p1);
      if (interval_ms < 1.0)
        interval_ms = SAMPLETIME * 1000;
    }
    if (strncmp(a, "motor", 5) == 0)
      startSensorLog();
    else if (strncmp(a, "pose", 4) == 0)
      startPoseCtrlLog();
    else if (strncmp(a, "pitch", 5) == 0)
      startPitchCtrlLog(interval_ms);
    else if (strncmp(a, "time", 4) == 0)
      startTimingLog(interval_ms);
    else if (strncmp(a, "imu", 3) == 0)
      startImuLog();
    else if (strncmp(a, "usb", 3) == 0)
      startUSBLog();
    else if (strncmp(a, "vroll", 5) == 0)
      interface = control->ctrlVelRoll;
    else if (strncmp(a, "roll", 4) == 0)
      interface = control->ctrlRoll;
    else if (strncmp(a, "vpitch", 6) == 0)
      interface = control->ctrlVelPitch;
    else if (strncmp(a, "pitch", 5) == 0)
      interface = control->ctrlPitch;
    else if (strncmp(a, "vyaw", 4) == 0)
      interface = control->ctrlVelYaw;
    else if (strncmp(a, "height", 6) == 0)
      interface = control->ctrlVelHeight;
    else if (strncmp(a, "stop", 4) == 0)
    {
      stop();
      command->usb_send_str("# logging stopped\r\n");
    }
    else
    {
      snprintf(s, MSL, "# failed to start log for interface %s\r\n", a);
      command->usb_send_str(s);
      return false;
    }
    if (interface != NULL)
      startCtrlLog();
    if (isLogging())
    {
      snprintf(s, MSL, "#setting log interval to %g ms for interface %s\r\n", interval_ms, a);
      command->usb_send_str(s);
    }
  }
  return true;
}


void ULogger::getCtrlLog()
{
  if (interface != NULL)
    interface->getCtrlLog(cdata, dataIdx, dataInterval);
}

void ULogger::addCtrlLogEntry()
{
  cdata[dataIdx].time = hb10us - logTimeStart;
  interface->toLog(&cdata[dataIdx]);
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void ULogger::startPoseCtrlLog()
{
  initLog();
  dataIdxMax = MAX_LOGGER_SPACE / sizeof(PoseCtrlData);
  logPose = true;  
  //
  command->usb_send_str("message starting Pose log\r\n");
}

void ULogger::addPoseCtrlLogEntry()
{
  pdata[dataIdx].time = hb10us - logTimeStart;
  pdata[dataIdx].state = state->getState();
  pdata[dataIdx].flightMode = state->flightState;
  pdata[dataIdx].refHeight = control->getRefHeight();
  pdata[dataIdx].refRoll = control->getRefRoll();
  pdata[dataIdx].refPitch = control->getRefPitch();
  pdata[dataIdx].refYaw = control->getRefYaw();
  pdata[dataIdx].mixerHeight = mixer->uHeight;
  pdata[dataIdx].mixerRoll = mixer->uRoll;
  pdata[dataIdx].mixerPitch = mixer->uPitch;
  pdata[dataIdx].mixerYaw = mixer->uYaw;
  pdata[dataIdx].posHeight = hgt->height;
  pdata[dataIdx].posRoll = imu->rollDeg;
  pdata[dataIdx].posPitch = imu->pitchDeg;
  pdata[dataIdx].velYaw = imu->gyro[2];
  pdata[dataIdx].heightVel = hgt->heightVelocity;
}


void ULogger::getPoseCtrlLog()
{
  const int MSL = 300;
  char s[MSL];
  const int i = dataIdxOut;
  if (dataIdxOut == -1)
  {  //
    snprintf(s, MSL, "%% data log for pose control, has %d values (interval=%g ms)\n\r", dataIdx, dataInterval*SAMPLETIME*1000);
    command->usb_send_str(s);
    command->usb_send_str("% data format:\r\n");
    command->usb_send_str("% 1     Time (ms)\r\n");
    command->usb_send_str("% 2     Arm state (init, disarmed, armed, fail)\r\n");
    command->usb_send_str("% 3     Flight state (on ground, starting, flight, landing)\r\n");
    command->usb_send_str("% 4     Height ref (trust ref)\r\n");
    command->usb_send_str("% 5     Roll ref (deg)\r\n");
    command->usb_send_str("% 6     Pitch ref (deg)\r\n");
    command->usb_send_str("% 7     Yaw ref (deg/s)\r\n");
    command->usb_send_str("% 8     Height value to mixer (trust ref) o/oo\r\n");
    command->usb_send_str("% 9     Roll value to mixer o/oo\r\n");
    command->usb_send_str("% 10    Pitch value to mixer o/oo\r\n");
    command->usb_send_str("% 11    Yaw value to mixer o/oo\r\n");
    command->usb_send_str("% 12    Height position (m) filtered \r\n");
    command->usb_send_str("% 13    Roll angle (deg)\r\n");
    command->usb_send_str("% 14    Pitch angle (deg)\r\n");
    command->usb_send_str("% 15    Yaw velocity (deg/s)\r\n");
    command->usb_send_str("% 16    Height velocity (m/s)\r\n");
    command->usb_send_str("%% \r\n");
  }
  else
  {
    snprintf(s, MSL, "%.2f %d %d %.5f %.5f %.5f %.5f %.1f %.1f %.1f %.1f %.5f %.5f %.5f %.5f %.3f \r\n",
            float(pdata[i].time)/100.0, // ms
            pdata[i].state, pdata[i].flightMode,
            pdata[i].refHeight, pdata[i].refRoll, pdata[i].refPitch, pdata[i].refYaw,
            pdata[i].mixerHeight, pdata[i].mixerRoll, pdata[i].mixerPitch, pdata[i].mixerYaw,
            pdata[i].posHeight, pdata[i].posRoll, pdata[i].posPitch, pdata[i].velYaw,pdata[i].heightVel
      );
    command->usb_send_str(s, true);
  }
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void ULogger::startPitchCtrlLog(float sampleMs)
{
  setSampleRate(sampleMs);
  initLog();
  dataIdxMax = MAX_LOGGER_SPACE / sizeof(PitchCtrlData);
  logPitch = true;  
  //
  command->usb_send_str("message starting Pitch log\r\n");
}

void ULogger::addPitchCtrlLogEntry()
{
  pitchData[dataIdx].time = hb10us - logTimeStart;
  pitchData[dataIdx].state = state->getState();
  pitchData[dataIdx].flightMode = state->flightState;
  pitchData[dataIdx].refPitchAngle = control->getRefPitch();
  pitchData[dataIdx].refPitchRate = control->getRefPitchRate();
  pitchData[dataIdx].gyroPitch = pitchRateFilt;
  pitchData[dataIdx].mixerPitch = mixer->uPitch;
  pitchData[dataIdx].pitchAngle = imu->pitchDeg;
}


void ULogger::getPitchCtrlLog()
{
  const int MSL = 300;
  char s[MSL];
  const int i = dataIdxOut;
  if (dataIdxOut == -1)
  {  //
    snprintf(s, MSL, "%% data log for pitch control, has %d values (interval=%g ms)\n\r", dataIdx, dataInterval*SAMPLETIME*1000);
    command->usb_send_str(s);
    command->usb_send_str("% data format:\r\n");
    command->usb_send_str("% 1     Time (ms)\r\n");
    command->usb_send_str("% 2     Arm state (init, disarmed, armed, fail)\r\n");
    command->usb_send_str("% 3     Flight state (on ground, starting, flight, landing)\r\n");
    command->usb_send_str("% 4     Pitch angle ref (deg)\r\n");
    command->usb_send_str("% 5     Pitch angle rate ref (deg/s)\r\n");
    command->usb_send_str("% 6     Pitch angle rate (gyro) (deg/s)\r\n");
    command->usb_send_str("% 7     Pitch value to mixer o/oo\r\n");
    command->usb_send_str("% 8     Pitch angle (deg)\r\n");
    command->usb_send_str("%% \r\n");
  }
  else
  {
      snprintf(s, MSL, "%.2f %d %d %g %.5f %.5f %g %.3f\r\n",
               float(pitchData[i].time)/100.0, // ms
               pitchData[i].state, pitchData[i].flightMode,
               pitchData[i].refPitchAngle,
               pitchData[i].refPitchRate,
               pitchData[i].gyroPitch,
               pitchData[i].mixerPitch,
               pitchData[i].pitchAngle
      );
      command->usb_send_str(s, true);
  }
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void ULogger::startTimingLog(float sampleMs)
{
  setSampleRate(sampleMs);
  initLog();
  dataIdxMax = MAX_LOGGER_SPACE / sizeof(TimingData);
  logTiming = true;  
  //
  command->usb_send_str("message starting Timing log\r\n");
}

void ULogger::addTimingLogEntry()
{
  timingData[dataIdx].time = hb10us - logTimeStart;
  timingData[dataIdx].adcSample = sensor->controlUsedTime[3];
  timingData[dataIdx].cycleTime = sensor->controlUsedTime[4];
  timingData[dataIdx].sensorFilterTime = sensor->controlUsedTime[0];
  timingData[dataIdx].controlTime = sensor->controlUsedTime[1];
  timingData[dataIdx].cycleProcessTime = sensor->controlUsedTime[2];
  timingData[dataIdx].adcPerCycle = adcFullCycles;
  timingData[dataIdx].cycleStartTime = sensor->controlUsedTime[5];
  timingData[dataIdx].time8 = sensor->controlUsedTime[8];
  timingData[dataIdx].time9 = sensor->controlUsedTime[9];
  timingData[dataIdx].time10 = sensor->controlUsedTime[10];
  timingData[dataIdx].readImuTime = sensor->controlUsedTime[6];
  timingData[dataIdx].endFilterTime = sensor->controlUsedTime[7];
  timingData[dataIdx].gyro = imu->gyro[0];
  timingData[dataIdx].acc = imu->accBoard[0];
  timingData[dataIdx].cntGyro = imu->asyncGyroCnt;
  timingData[dataIdx].cntAccMag = imu->asyncAccMagCnt;
  timingData[dataIdx].cntAltitude = imu->asyncAirPressureCnt;
}

/*
 *  uint32_t cycleStartTime;
 *  uint32_t readImuTime;
 *  uint32_t endFilterTime;
 */

void ULogger::getTimingLog()
{
  const int MSL = 300;
  char s[MSL];
  const int i = dataIdxOut;
  if (dataIdxOut == -1)
  { //
    snprintf(s, MSL, "%% data log for timing, has %d values (interval=%g ms)\n\r", dataIdx, dataInterval*SAMPLETIME*1000);
    command->usb_send_str(s);
    command->usb_send_str("% data format:\r\n");
    command->usb_send_str("% 1     Log sample time (ms)\r\n");
    command->usb_send_str("% 2     ADC time for all values (ms)\r\n");
    command->usb_send_str("% 3     ADC samples per cycle\r\n");
    command->usb_send_str("% 4     Cycle time (ms)\r\n");
    command->usb_send_str("% 5     Sensor filter time (ms)\r\n");
    command->usb_send_str("% 6     Control time (ms)\r\n");
    command->usb_send_str("% 7     Cycle process time (ms)\r\n");
    command->usb_send_str("% 8     Cycle start time (sec)\r\n");
    command->usb_send_str("% 9     Get IMU data time (ms)\r\n");
    command->usb_send_str("% 10    Board-coord_conv_time (ms)\r\n");
    command->usb_send_str("% 11    To end filter time (ms)\r\n");
    command->usb_send_str("% 12    gyro (roll) deg/s\r\n");
    command->usb_send_str("% 13    Acc (x) m/s^2\r\n");
    command->usb_send_str("% 14    Gyro reads this sample\r\n");
    command->usb_send_str("% 15    Acc/mag reads this sample\r\n");
    command->usb_send_str("% 16    Pressure reads this sample\r\n");
    command->usb_send_str("% 17    get Acc+Mag Data from I2C (ms)\r\n");
    command->usb_send_str("%% \r\n");
  }
  else
  {
    float ct = 1.0/ float(F_CPU/1000);
    snprintf(s, MSL, "%.2f %g %u %g %g %g %g %g %g %g %g %g %g %d %d %d %g\r\n",
              float(timingData[i].time)/100, // ms
              timingData[i].adcSample * ct,
              timingData[i].adcPerCycle,
              timingData[i].cycleTime * ct,
              timingData[i].sensorFilterTime * ct,
              (timingData[i].controlTime - timingData[i].sensorFilterTime) * ct,
              timingData[i].cycleProcessTime * ct,
              timingData[i].cycleStartTime * ct / 1000.0,
              (timingData[i].readImuTime - timingData[i].cycleStartTime) * ct,
              (timingData[i].readImuTime - timingData[i].time8) * ct,
              (timingData[i].endFilterTime - timingData[i].cycleStartTime) * ct,
              timingData[i].gyro,
              timingData[i].acc,
              timingData[i].cntGyro,
              timingData[i].cntAccMag,
              timingData[i].cntAltitude,
              (timingData[i].time10 - timingData[i].time9) * ct
    );
    command->usb_send_str(s, true);
  }
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void ULogger::startUSBLog()
{
  initLog();
  dataIdxMax = MAX_LOGGER_SPACE;
  logUSB = true;
  usbLogStart = hbTimerCnt;
  dataInterval = 10000; // just to stop tick-based log-entry iterating for no use
  // clear buffer
  // memset(dataBuffer, 0, dataIdxMax);
  //
  command->usb_send_str("message starting USB log\r\n");
}

void ULogger::addUSBLogEntry(const char * str, int n, uint32_t preIOhb, int newEntry)
{
  bool toStop = false;
  if (dataIdxMax - dataIdx > 15)
  {
    char * p1 = (char *)&dataBuffer[dataIdx];
    if (newEntry != 0)
    {
      char d = 'o'; // direction flag
      if (newEntry < 0)
        d = 'i';
      snprintf(p1, 15, "%lu%c%lu:", preIOhb - usbLogStart, d, hbTimerCnt - usbLogStart);
      int nt = strlen(p1);
      p1 += nt;
      dataIdx += nt;
    }
    if (dataIdxMax - dataIdx > n)
    { // space for data
      strncpy(p1, str, n);
      p1 += n;
      dataIdx += n;
      // terminate as c-string
//      dataBuffer[dataIdx] = 0;
    }
    else
      toStop = true;
  }
  else
  {
    toStop  =true;
  }
  if (toStop)
  {
    stop();
    // add an extra zero
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "%% log full (%d/%d chars)\r\n", dataIdx, dataIdxMax);
    command->usb_send_str(s);
  }
}

void ULogger::getUSBLog()
{
  const int MSL = 300;
  char s[MSL];
  if (dataIdxOut == -1)
  {  
    /// @todo - it seems like not all data gets transferred with a "log get"
    ///         it might be that the host end of USB say: wait, buffer full!
    usbIOData = (const char *)dataBuffer;
    command->usb_send_str("% USB log\r\n");
    snprintf(s, MSL, "%% All text, %d/%d chars\r\n", dataIdx, dataIdxMax);
    command->usb_send_str(s);
    command->usb_send_str("% data format:  '1i2:data\r\n");
    snprintf(s, MSL, "%% 1     hb tick before IO (each %.1f ms)\r\n", SAMPLETIME * 1000.0);
    command->usb_send_str(s);
    command->usb_send_str("% i|o   data direction i=to Teensu, o=from Teensy\r\n");
    command->usb_send_str("% 2     hb tick after io\r\n");
    command->usb_send_str("% 3     io message\r\n");
  }
  else
  { // then the data
    int n = dataIdx - dataIdxOut;
    int m = 1;
    while (m < n and usbIOData[m] >= ' ')
      m++;
    while (m < n and usbIOData[m] < ' ')
      m++;
    if (false)
    { // debug
//       snprintf(s, MSL, "# log: %d has %d chars, idx %d/%d ", lines, m, cnt, dataIdx);
      if (true)
      { // add text too
        int w = strlen(s);
        char * p1 = &s[w];
        int u = m;
        if (w + u >= MSL)
          u = MSL - w - 1;
        strncpy(p1, usbIOData, u);
        p1[u] = '\0';
      }
      command->usb_send_str(s, true);
      // debug end
    }
    else
    {
      command->usb_send_block(usbIOData, m, true);
    }
    usbIOData += m;
  }
}
