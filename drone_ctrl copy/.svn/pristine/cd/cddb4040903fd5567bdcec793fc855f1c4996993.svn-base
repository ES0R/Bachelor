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

#ifndef PROP_SHIELD_H
#define PROP_SHIELD_H

#include "upropsense.h"
#include "main.h"
#include "MadgwickAHRS.h"
#include "command.h"
#include "controlbase.h"
#include "umat.h"


class UPropShield : public UPropSense
{
protected:
  /*Prop shield varibles*/
  /**
   * Raw data  */
  int axw, ayw, azw;
  int gxw, gyw, gzw;
  int mxw, myw, mzw;
  int txmsg = 0;
public:
  /**
   * calibrated data */
  /// in Drone coordinates
  float accBoard[3];
  float gyroBoard[3];
  float magBoard[3];
  // after board orientation correction
  float acc[3];
  float gyro[3];
  float mag[3];
  // acceleration in world coordinates
  float accw[3];
  // rotation matrix from drone rotation to ground orientation
  UMatRot3x3 mrotd2w;
  //
  int getInt2Cnt();
  //
  bool boardOK = false;
  
protected:
  Madgwick filter;
  // sample rate for Madgwick filter [Hz]
private:
//   int imu4calCnt = 0;
  uint32_t offsetSummationCnt;
  float offsetGyro[3];
  float offsetAcc[3];
  //
  float altTs = SAMPLETIME;
  /// board orientation relative
  /// to drone forward and up
  UMatRot3x3 boardOrientation;
  float rX = 0, rY = 0, rZ = 0; //radians
  static const int MBS = 4;
  int16_t asyncGyroBuffer[MBS][3];
  
public:
  int sampleRate = 400;
  float roll; // radians
  float pitch;
  float yaw;
  float rollDeg; // degrees
  float pitchDeg;
  float yawDeg;
  float altitude = 0; // m - pressure altitude
  float altTemp = 0;  // deg C
  float altOffset = 0;
  double altCalibSum = 0;
  int offsetSampleCnt = 0;
  bool useMag = true;
  bool inGyroCal = false;
  bool sensorOffsetDone = true;
  bool altCalib = false;
  bool gyroCalib = false;
  bool accCalib = false;
  bool magCalib = false;
  uint8_t readDataOK = 0;
  /** async data read - accelerometer and magnetometer
   * reads per sample */
  int asyncAccMagCnt = 0;
  /** async data read - Altitude meter
   * reads per sample */
  int asyncAirPressureCnt = 0;
  /** async data read - Gyro
   * reads per sample */
  int asyncGyroCnt = 0;
  

public:
  void setup();
  /**
   * send command help */
  void sendHelp();
  /**
   * decode data commands */
  bool decode(const char * buf);
  /**
   * Get and filter data from sensors */
  void tick(bool raw, int mainLoop);
  /** 
   * Send raw measurements to client */
  void sendImuRaw();
  /** 
   * Send filtered measurements to client */
  void sendAccSI();
  void sendGyroSI();
  void sendMagSI();
  /** 
   * Send altitude data to client to client */
  void sendAltitude();
  /** 
   * Send all calibration values to client */
  void sendImuCalibAcc();
  void sendImuCalibGyro();
  void sendImuCalibMag();
  /**
   * Board orientation in radians */
  void sendBoardOrientation();
  /**
   * save steering configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  /**
   * setup altimeter - debug */
  void altSetup(uint8_t osr)
  {
    bool ok = MPL3115_begin(osr);
    if (ok)
      command->usb_send_str("# alt setup ended OK\r\n");
    else
      command->usb_send_str("# alt setup ended NOT OK\r\n");
  }
  /**
   * setup of altitude complementary filter 
   * Sample time is the same as system sample time
   * \param tau is time constant for crossover frequency (1/tau) */
//   void altComplFiltSetup(float tau, float beta);
  /**
   * setup of altitude complementary filter 
   * Sample time is the same as system sample time
   * \param tau is time constant for crossover frequency (1/tau) */
//   void altComplFiltTick();
  /**
   * Set known altitude */
  void altSetKnownHeight(float knownHeight);
  /**
   * restart calibration */
  void restartOffset();
  /**
   * test function */
  void test(float r, float p, float y)
  {
    UMatRot3x3 a;
    float d[3] = {r,p,y};
    a.set(r, p, y);
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "#  roll=%f, pitch=%f yaw=%f\n", r,p,y);
    command->usb_send_str(s);
    command->usb_send_str("# forward matrix:\n");
    for (int i = 0; i < 3; i++)
    {
      snprintf(s, MSL, "#  [ %5.4f %5.4f %5.4f ] \n", a.d2w[i][0], a.d2w[i][1], a.d2w[i][2]);
      command->usb_send_str(s);
    }
    command->usb_send_str("# Inverse matrix:\n");
    for (int i = 0; i < 3; i++)
    {
      snprintf(s, MSL, "#  [ %5.4f %5.4f %5.4f ] \n", a.w2d[i][0], a.w2d[i][1], a.w2d[i][2]);
      command->usb_send_str(s);
    }
    command->usb_send_str("# vector rotated:\n");
    float w[3];
    a.rotateD2W(d, w);
    snprintf(s, MSL, "#  [ %5.4f %5.4f %5.4f ] \n", w[0], w[1], w[2]);
    command->usb_send_str(s);
    command->usb_send_str("# and back:\n");
    float u[3];
    a.rotateW2D(w, u);
    snprintf(s, MSL, "#  [ %5.4f %5.4f %5.4f ] \n", u[0], u[1], u[2]);
    command->usb_send_str(s);
    
  }
  
  void clearAccOffset()
  {
    float a[3] = {0};
    setAccOfs(a);
    sendImuCalibAcc();
  }
  
  void clearGyroOffset()
  {
    float a[3] = {0};
    setGyroOfs(a);
  }
  /**
   * read async (mostly gyro) */
  void asyncRead();
  
protected:
  void sensorOffset();
  void getSensorDataSync(float * acc, float * gyro, float * mag);
  
};

#endif
