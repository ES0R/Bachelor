/***************************************************************************
 *   Copyright (C) 2021 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Class to handle the Prop shield from PJRC
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


#include "upropshield.h"
#include "main.h"
#include "command.h"
#include "eeconfig.h"
#include "uheight.h"
#include "sensor.h"

int int2Cnt = 0;

void isrPropShield()
{ // interrupt test
  // never called?
  int2Cnt++;
}

void UPropShield::setup()
{ // setup Propshield sensors
  begin();
  // set Madgwick filter
  sampleRate = 400;
  filter.begin(sampleRate);
  // error check
  boardOK = not (err_FXAS21002 or err_FXOS8700 or err_MPL3115);
  if (not boardOK)
  { // sensor error
//     digitalWriteFast(PIN_LED_1, 1);
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# IMU sensor error: gyro %d, acc %d, baro %d\n",err_FXAS21002, err_FXOS8700, err_MPL3115);
    command->usb_send_str(s);
  }
  // set sensor board orientation
  boardOrientation.set(rX, rY, rZ);
  // use sensor interrupt (for at least gyro)
  if (false)
  { // does not work - using poll
    pinMode ( PIN_SENSOR_INT, INPUT );
    attachInterrupt(PIN_SENSOR_INT, isrPropShield, FALLING);
  }
}

int UPropShield::getInt2Cnt()
{
  return int2Cnt;
}


void UPropShield::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  command->usb_send_str("# ----- IMU (prop shield) --------\r\n");
  snprintf(reply, MRL, "#   imuraw       get IMU data (integer) (debug).\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   usemag 0/1   Use magnetometer (is %d).\r\n", imu->useMag);
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   offsetcal <x> Start offset calibration; h=height, g=gyro, a=acc - require horizontal), m=magdata.\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   imu <x>      Get IMU data SI-units, float. <x>: a=acc, g=gyro, h=height, m=magnetometer.\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   imucal <x>   Get IMU calibration: a=acc, g=gyro, m=magnetometer, A,G ox, oy, oz use these offsets.\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   imuzero <x>  Clear IMU calibration: a=acc, g=gyro, m=magnetometer.\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   alt m        Set offset to 'm' (meter) (alt: %.2f, ofs: %.2f).\r\n", imu->altitude, imu->altOffset);
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   test         test function.\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   altsetup     Call altimeter setup (debug)\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL,  "#   board rX rY rZ  Set board orientation angles (radians) is %.3f %.3f %.3f.\r\n", rX, rY, rZ);
  command->usb_send_str(reply);
  command->usb_send_str("#   brd        Get board orientation\r\n");
}


bool UPropShield::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "imuraw ", 6) == 0)
  {
    sendImuRaw();
  }
  else if (strncmp(buf, "imucal ", 6) == 0)
  {
    char * p1 = (char *)&buf[6];
    float xyz[3];
    while (isspace(*p1)) p1++;
    switch (*p1)
    {
      case 'a': sendImuCalibAcc(); break;
      case 'm': sendImuCalibMag(); break;
      case 'g': sendImuCalibGyro(); break;
      case 'A': 
        p1++;
        xyz[0] = strtof(p1, &p1);
        xyz[1] = strtof(p1, &p1);
        xyz[2] = strtof(p1, &p1);
        setAccOfs(xyz); 
        break;
      case 'G':
        p1++;
        xyz[0] = strtof(p1, &p1);
        xyz[1] = strtof(p1, &p1);
        xyz[2] = strtof(p1, &p1);
        setGyroOfs(xyz); 
        break;
      default: command->usb_send_str("# unknown 'imucal' option (a,g,m,A,G only)\r\n"); break;
    }
  }
  else if (strncmp(buf, "imuzero ", 7) == 0)
  {
    char * p1 = (char *)&buf[7];
    while (isspace(*p1)) p1++;
    switch (*p1)
    {
      case 'a': clearAccOffset(); break;
      case 'm': command->usb_send_str("# use magnetometer calibration utility\r\n"); break;
      case 'g': clearGyroOffset(); break;
      default: command->usb_send_str("# unknown 'imuzero' option (a,g only)\r\n"); break;
    }
  }
  else if (strncmp(buf, "imu ", 3) == 0)
  {
    char * p1 = (char *)&buf[3];
    while (isspace(*p1)) p1++;
    switch (*p1)
    {
      case 'a': sendAccSI(); break;
      case 'g': sendGyroSI(); break;
      case 'm': sendMagSI(); break;
      case 'h': sendAltitude(); break;
      default: command->usb_send_str("# unknown 'imu' option (a,g,m,h only)\r\n"); break;
    }
  }
  else if (strncmp(buf, "offsetcal ", 9) == 0)
  {
    const char * p1 = &buf[9];
    bool a = false;
    bool m = false;
    bool g = false;
    bool h = false;
    while (isspace(*p1)) p1++;
    while (*p1 > ' ')
    {
      switch (*p1++)
      {
        case 'a': a = true; break;
        case 'm': m = true; break;
        case 'g': g = true; break;
        case 'h': h = true; break;
        default: command->usb_send_str("# unknown 'offset' option (a,g,m,h only)\r\n"); break;
      }
    }
    if (not (a or m or g or h))
    {
      command->usb_send_str("# stopping calibration, use option a,g,m and/or h to start\r\n");
      sensorOffsetDone = true;
      accCalib = false;
      gyroCalib = false;
      altCalib = false;
      magCalib = false;
    }
    else if (m)
      // only the flag is needed - sending raw mag data to external app until stopped
      // not really compatible with normal offset
      magCalib = true;
    else
    { // start collecting data (stops after 1 second)
      accCalib = a;
      gyroCalib = g;
      altCalib = h;
      restartOffset();
      command->usb_send_str("# got offset start calibration\r\n");
    }
  }
  else if (strncmp(buf, "test ", 4) == 0)
  {
    char * p1 = (char*)&buf[4];
    float r, p, y;
    r = strtof(p1, &p1);
    p = strtof(p1, &p1);
    y = strtof(p1, &p1);
    test(r, p, y);
  }
  else if (strncmp(buf, "alt ", 3) == 0)
  {
    char * p1 = (char*)&buf[3], *p2;
    if (strlen(p1) > 1)
    {
      float a = strtof(p1, &p2);
      if (p2 > p1)
        altOffset = a;
    }
    sendAltitude();
  }
  else if (strncmp(buf, "usemag ", 6) == 0)
  {
    char * p1 = (char*)&buf[6];
    if (strlen(p1) > 1)
      useMag = strtol(p1, &p1, 10) != 0;
    else
      useMag = true;
    if (useMag)
      command->usb_send_str("# using compass\r\n");
    else
      command->usb_send_str("# ignoring compass\r\n");
  }
  else if (strncmp(buf, "brd ", 3) == 0)
  {
    sendBoardOrientation();
  }
  else if (strncmp(buf, "board ", 5) == 0)
  {
    char * p1 = (char*)&buf[5];
    rX = strtof(p1, &p1);
    rY = strtof(p1, &p1);
    rZ = strtof(p1, &p1);
    boardOrientation.set(rX, rY, rZ);
  }
  else
    used = false;
  return used;
}

void UPropShield::asyncRead()
{ // read sensors, if available
  // gyro - no oversampling on chip
  int g = FXAS21002_read(asyncGyroBuffer[0]);
  if (g == 0 and asyncGyroCnt < MBS - 1)
    asyncGyroCnt++;
}

void UPropShield::getSensorDataSync(float * acc, float * gyro, float * mag)
{ // gyro
//   uint16_t gs[3] = {0};
//   if (asyncGyroCnt > 1)
//   { // average over measurements
//     for (int i = 0; i < asyncGyroCnt; i++)
//     {
//       gs[0] += asyncGyroBuffer[i][0];
//       gs[1] += asyncGyroBuffer[i][1];
//       gs[2] += asyncGyroBuffer[i][2];
//     }
//     gyro[0] = float(gs[0]/asyncGyroCnt) * DEG_PER_SEC_PER_COUNT - cal[3];
//     gyro[1] = float(gs[1]/asyncGyroCnt) * DEG_PER_SEC_PER_COUNT - cal[4];
//     gyro[2] = float(gs[2]/asyncGyroCnt) * DEG_PER_SEC_PER_COUNT - cal[5];
//   }
//   else if (asyncGyroCnt == 1)
  { // one reading only
    gyro[0] = (float)asyncGyroBuffer[0][0] * DEG_PER_SEC_PER_COUNT - cal[3];
    gyro[1] = (float)asyncGyroBuffer[0][1] * DEG_PER_SEC_PER_COUNT - cal[4];
    gyro[2] = (float)asyncGyroBuffer[0][2] * DEG_PER_SEC_PER_COUNT - cal[5];
  }
  //
//   sensor->controlUsedTime[9] = ARM_DWT_CYCCNT;
  // acc-mag - sync read
  bool amOK;
  if (imu->boardOK)
    amOK = FXOS8700_read(accel_mag_raw);  // accel + mag
  else
    amOK = false;  
  if (amOK)
    sensor->controlUsedTime[10] = ARM_DWT_CYCCNT;
  else
  { // not valid timing
    sensor->controlUsedTime[10] = sensor->controlUsedTime[9];
  }
  if (amOK)
  { // acc calibration
    asyncAccMagCnt = 1;
    acc[0] = (float)accel_mag_raw[0] * G_PER_COUNT - cal[0];
    acc[1] = (float)accel_mag_raw[1] * G_PER_COUNT - cal[1];
    acc[2] = (float)accel_mag_raw[2] * G_PER_COUNT - cal[2];
    // mag calibration
    float x = (float)accel_mag_raw[3] * UT_PER_COUNT - cal[6];
    float y = (float)accel_mag_raw[4] * UT_PER_COUNT - cal[7];
    float z = (float)accel_mag_raw[5] * UT_PER_COUNT - cal[8];
    // 
    mag[0] = x * cal[10] + y * cal[13] + z * cal[14];
    mag[1] = x * cal[13] + y * cal[11] + z * cal[15];
    mag[2] = x * cal[14] + y * cal[15] + z * cal[12];
  }
  // 
  // pressure sync read
  if (boardOK)
  {
    bool al = MPL3115_read(&altitude_raw, &temperature_raw);
    if (al)
    { // alt
      MPL3115_meas_restart();
      asyncAirPressureCnt = 1;
    }
  }
}


void UPropShield::tick(bool raw, int calLoop)
{ // sync read
  if (sensorOffsetDone)
  {
//     uint8_t readDataOK = 0;
    if (raw)
    { // NB! this is used in board coordinates
      // and is different if board in not upright with
      // USB plug pointing backwards.
      readMotionSensor(axw, ayw, azw, gxw, gyw, gzw, mxw, myw, mzw);
      sensor->controlUsedTime[6] = ARM_DWT_CYCCNT;
    }
    else
    { // gyro is in deg/sec
      // readDataOK = readMotionSensor(accBoard, gyroBoard, magBoard);
      getSensorDataSync(accBoard, gyroBoard, magBoard);
      sensor->controlUsedTime[8] = ARM_DWT_CYCCNT;
      boardOrientation.rotateD2W(accBoard, acc);
      boardOrientation.rotateD2W(gyroBoard, gyro);
      boardOrientation.rotateD2W(magBoard, mag);
      sensor->controlUsedTime[6] = ARM_DWT_CYCCNT;
      // filter takes gyro data in deg/s
      if (useMag)
        filter.update(gyro[0], gyro[1], gyro[2], 
                    acc[0],  acc[1],  acc[2], 
                    mag[0],  mag[1],  mag[2]);
      else
        filter.updateIMU(gyro[0], gyro[1], gyro[2], 
                         acc[0],  acc[1],  acc[2]);
    }
    // convert to euler
    if (not raw)
    { // NB! pitch is in the range +/- pi/2 (90 deg) deg 
      // more than 90 pitch changes roll by 180 deg
      roll = filter.getRollRadians();
      rollDeg = roll * 180.0/M_PI;
      pitch = filter.getPitchRadians();
      pitchDeg = pitch * 180.0/M_PI;
      yaw = filter.getYawRadians();
      yawDeg = yaw * 180.0/M_PI;
      // make new rotation matrix - for height calculation
      mrotd2w.set(roll, pitch, yaw);
    }
    sensor->controlUsedTime[7] = ARM_DWT_CYCCNT;
  }
  else
  { // perform calibration
    getSensorDataSync(accBoard, gyroBoard, magBoard);
//     readMotionSensor(accBoard, gyroBoard);
    if (asyncAccMagCnt >= 1 and asyncGyroCnt >= 1)
    { // there is fresh data to use
      boardOrientation.rotateD2W(accBoard, acc);
      boardOrientation.rotateD2W(gyroBoard, gyro);
      sensorOffset();
    }
  }
  // pressure altitude
  altitude = getAltitude(); // wait with the offset - altOffset;
  altTemp = getTemperature();
}

void UPropShield::restartOffset()
{
  sensorOffsetDone = false;
  offsetSummationCnt = hbTimerCnt + 2;
}

void UPropShield::sensorOffset()
{ // do gyro offset calculation
  const int samples = 500;
  if (hbTimerCnt < offsetSummationCnt)
  { // zero offset before 500 summations
    // remove current offset
    float zero[3] = {0,0,0};
    if (gyroCalib)
    {
      setGyroOfs(zero);
  //     setAccOfs(zero);
      offsetGyro[0] = gyroBoard[0];
      offsetGyro[1] = gyroBoard[1];
      offsetGyro[2] = gyroBoard[2];
    }
    if (accCalib)
    {
      setAccOfs(zero);
      offsetAcc[0] = acc[0];
      offsetAcc[1] = acc[1];
      offsetAcc[2] = acc[2];
    }
    if (altCalib)
      altCalibSum = hgt->altitudef; // getAltitude();
    offsetSampleCnt = 1;
    offsetSummationCnt = hbTimerCnt;
    command->usb_send_str("# gyro/acc offset calibrate started (remember to save (eew))\r\n");
  }
  else if (hbTimerCnt < offsetSummationCnt + samples)
  { // not finished offset calculation
    // summation over 500 samples (
    if (gyroCalib)
    {
      offsetGyro[0] += gyroBoard[0];
      offsetGyro[1] += gyroBoard[1];
      offsetGyro[2] += gyroBoard[2];
    }
    if (accCalib)
    {
      offsetAcc[0] += acc[0];
      offsetAcc[1] += acc[1];
      offsetAcc[2] += acc[2];
    }
    if (altCalib)
    { // calibrate the filtered value
      altCalibSum += hgt->altitudef;
//       altCalibSum += getAltitude();
    }
    offsetSampleCnt++;
//     const int MSL = 120;
//     char s[MSL];
//     snprintf(s, MSL, "# progress: ACC: %f, %f, %f, (az=%.2f) GYRO: %f %f %f (of %d)\r\n",
//              offsetAcc[0], offsetAcc[1], offsetAcc[2], az, offsetGyro[0], offsetGyro[1], offsetGyro[2], offsetGyroCnt);
//     command->usb_send_str(s);
  }
  else if (hbTimerCnt == offsetSummationCnt + samples)
  { // set average offset
    if (gyroCalib)
    {
      offsetGyro[0] /= float(offsetSampleCnt);
      offsetGyro[1] /= float(offsetSampleCnt);
      offsetGyro[2] /= float(offsetSampleCnt);
      setGyroOfs(offsetGyro);
    }
    if (accCalib)
    {
      float g;
      offsetAcc[0] /= float(offsetSampleCnt); // to exact zero
      offsetAcc[1] /= float(offsetSampleCnt); // to exact zero
      offsetAcc[2] /= float(offsetSampleCnt);
      g = offsetAcc[2];
      offsetAcc[2] -= 1.0;
      float o[3];
      boardOrientation.rotateW2D(offsetAcc, o);
      setAccOfs(o);
      const int MSL = 120;
      char s[MSL];
      snprintf(s, MSL, "# finished: ACC: %f %f %f (of %d) 1G measured = %f\r\n",
               o[0], o[1], o[2], offsetSampleCnt, 
               g);
      command->usb_send_str(s);
    }
    if (altCalib)
    {
      altOffset = altCalibSum / float(offsetSampleCnt); // height in m
    }
    //
    const int MSL = 120;
    char s[MSL];
    snprintf(s, MSL, "# finished: GYRO: %f %f %f (of %d) ALT: %f, ACC: %f %f %f\r\n",
             offsetGyro[0], offsetGyro[1], offsetGyro[2], offsetSampleCnt, 
             altOffset,
             offsetAcc[0], offsetAcc[1], offsetAcc[2]
            );
    command->usb_send_str(s);
    sensorOffsetDone  = true;
    altCalib = false;
    gyroCalib = false;
    accCalib = false;
  }
  else
    // redo of calibrate requested - timer overflow
    offsetSummationCnt = hbTimerCnt + 2;
}

void UPropShield::sendImuRaw()
{
  const int MSL = 120;
  char s[MSL];
  tick(true, 1); // get raw data
  snprintf(s, MSL, "imuw %d %d %d  %d %d %d  %d %d %d\n",
           axw, ayw, azw, gxw, gyw,gzw, mxw, myw, mzw);
  command->usb_send_str(s);
}

void UPropShield::sendAccSI()
{
  const int MSL = 220;
  char s[MSL];
//   snprintf(s, MSL, "acc %f %f %f  %f %f %f %d\n", acc[0], acc[1], acc[2], accw[0], accw[1], accw[2], txmsg++ );
  snprintf(s, MSL, "acc %f %f %f %d\n", acc[0], acc[1], acc[2], txmsg++ );
  command->usb_send_str(s, false);
}

void UPropShield::sendGyroSI()
{
  const int MSL = 220;
  char s[MSL];
  snprintf(s, MSL, "gyro %f %f %f %d\n", gyro[0], gyro[1],gyro[2], txmsg++);
  command->usb_send_str(s, false);
}

void UPropShield::sendMagSI()
{
  const int MSL = 220;
  char s[MSL];
  snprintf(s, MSL, "mag %f %f %f %d\n", mag[0], mag[1], mag[2], txmsg++);
  command->usb_send_str(s, false);
}


void UPropShield::sendImuCalibMag()
{
  const int MCD = 9;
  float off[MCD];
  float soft[MCD];
  float mstrength;
  //
  const int MSL = 120;
  char s[MSL];
  getCalibration(off, soft, &mstrength);
  snprintf(s, MSL, "magoff ");
  int n = strlen(s);
  char * p1 = &s[n];
  for (int i = 6; i < 9; i++)
  {
    snprintf(p1, MSL - n, " %g", off[i]);
    n += strlen(p1);
    p1 = &s[n];
  }
  snprintf(p1, MSL - n, "\n");
  command->usb_send_str(s);
  //
  snprintf(s, MSL, "imumagc ");
  n = strlen(s);
  p1 = &s[n];
  for (int i = 0; i < 9; i++)
  {
    snprintf(p1, MSL - n, " %g", soft[i]);
    n += strlen(p1);
    p1 = &s[n];
  }
  snprintf(p1, MSL - n, " %g\n", mstrength);
  command->usb_send_str(s);
}

void UPropShield::sendImuCalibAcc()
{
  const int MCD = 9;
  float off[MCD];
  const int MSL = 120;
  char s[MSL];
  getCalibration(off, NULL, NULL);
  snprintf(s, MSL, "accoff ");
  int n = strlen(s);
  char * p1 = &s[n];
  for (int i = 0; i < 3; i++)
  {
    snprintf(p1, MSL - n, " %g", off[i]);
    n += strlen(p1);
    p1 = &s[n];
  }
  snprintf(p1, MSL - n, "\n");
  command->usb_send_str(s);
}

void UPropShield::sendImuCalibGyro()
{
  const int MCD = 9;
  float off[MCD];
  const int MSL = 120;
  char s[MSL];
  getCalibration(off, NULL, NULL);
  snprintf(s, MSL, "gyrooff ");
  int n = strlen(s);
  char * p1 = &s[n];
  for (int i = 3; i < 6; i++)
  {
    snprintf(p1, MSL - n, " %g", off[i]);
    n += strlen(p1);
    p1 = &s[n];
  }
  snprintf(p1, MSL - n, "\n");
  command->usb_send_str(s);
}

void UPropShield::sendAltitude()
{
  const int MSL = 120;
  char s[MSL];
//   snprintf(s, MSL, "alti %.2f %.1f %.2f (%02x %02x %02x %02x %02x)\n", altitude, altTemp, altOffset, 
//            alt_ctrl_1[0], alt_ctrl_1[1], alt_ctrl_1[2], alt_ctrl_1[3], alt_ctrl_1[4]);
  if (not altCalib)
  {
    snprintf(s, MSL, "alti %g %g %g %g %g\n", 
            altitude, hgt->height, altTemp, altOffset,
            hgt->heightVelocity);
    command->usb_send_str(s);
  }
}


void UPropShield::sendBoardOrientation()
{
  const int MSL = 120;
  char s[MSL];
  snprintf(s, MSL, "brdr %g %g %g\n", rX, rY, rZ);
  command->usb_send_str(s);
}


void UPropShield::eePromLoad()
{
  const int MSL = 100;
  char s[MSL];
  // load gyro and accelerometer offset
  float ofss[3];
  for (int i = 0; i < 3; i++)
  {
    ofss[i] = eeConfig->readFloat();
    snprintf(s, MSL, "# loaded to cal%d: %f - to address %d\n", i, ofss[i], eeConfig->getAddr() - 4);
    command->usb_send_str(s);
  }
  setAccOfs(ofss);
  for (int i = 0; i < 3; i++)
  {
    ofss[i] = eeConfig->readFloat();
    snprintf(s, MSL, "# loaded to cal%d: %f - to address %d\n", i, ofss[i], eeConfig->getAddr() - 4);
    command->usb_send_str(s);
  }
  setGyroOfs(ofss);
  float a = eeConfig->readFloat();
  if (abs(a) < 500)
    altOffset = a;
  else
    altOffset = 0;
  /*float tau =*/ eeConfig->readFloat();
  /*float beta =*/ eeConfig->readFloat();
  snprintf(s, MSL, "# loaded altOffset = %g\n", altOffset);
  command->usb_send_str(s);
  rX = eeConfig->readFloat();
  rY = eeConfig->readFloat();
  rZ = eeConfig->readFloat();
  boardOrientation.set(rX, rY, rZ);
  // values not used
//   altComplFiltSetup(tau, beta);
}

void UPropShield::eePromSave()
{
//   const int MSL = 100;
//   char s[MSL];
  // save gyro and accelerometer offset
  float ofss[9];
  getCalibration(ofss, NULL, NULL);
  for (int i = 0; i < 6; i++)
  {
    eeConfig->pushFloat(ofss[i]);
//     snprintf(s, MSL, "# save cal%d: %f - to address %d\n", i, ofss[i], eeConfig->getAddr() - 4);
//     command->usb_send_str(s);
  }
  eeConfig->pushFloat(altOffset);
  eeConfig->pushFloat(0); // filter tau - may be reused
  eeConfig->pushFloat(0); // filter beta - may be reused
  eeConfig->pushFloat(rX);
  eeConfig->pushFloat(rY);
  eeConfig->pushFloat(rZ);
  {
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# saved altOffset = %g\n", altOffset);
    command->usb_send_str(s);
  }
}

void UPropShield::altSetKnownHeight(float knownHeight)
{ // make current estimate to the known height
  altOffset += altitude - knownHeight;
}

