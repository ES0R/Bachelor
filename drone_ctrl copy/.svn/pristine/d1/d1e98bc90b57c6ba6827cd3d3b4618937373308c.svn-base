/**
 * Copy and slightly modified version of NXPMotionSense/NXPMotionSense.h
 * from TeensyDuino version 1.53 (oktober 2020)
 * 
 * Modified to get more data and better calibration use
 * jca@elektro.dtu.dk
 * */


#ifndef _NXP_Motion_Sensors_
#define _NXP_Motion_Sensors_

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

// TODO: move these inside class namespace
// #define G_PER_COUNT            (1.0/8096.0) /* = 0.488 mg/LSB 4G mode */
// #define DEG_PER_SEC_PER_COUNT  0.0625f  // = 1/16
// #define UT_PER_COUNT           0.1f

class UPropSense {
public:
  const float G_PER_COUNT = 1.0/8096.0; /* = 0.488 mg/LSB 4G mode */
  const float DEG_PER_SEC_PER_COUNT = 0.0625f;  // = 1/16
  const float UT_PER_COUNT = 0.1f;
  //
  bool begin();
  bool available() {
    update();
    if (newdata) 
      return true;
    return false;
  }
  int readMotionSensor(int& ax, int& ay, int& az, int& gx, int& gy, int& gz) 
  {
    int isOK = update(); // should be 0x7 if data from all 3 sensors
    //if (!newdata) update();
    newdata = 0;
    ax = accel_mag_raw[0];
    ay = accel_mag_raw[1];
    az = accel_mag_raw[2];
    gx = gyro_raw[0];
    gy = gyro_raw[1];
    gz = gyro_raw[2];
    return isOK;
  }
  int readMotionSensor(int& ax, int& ay, int& az, int& gx, int& gy, int& gz, int& mx, int& my, int& mz) 
  {
    int isOK = update(); // should be 0x7 if data from all 3 sensors
//     if (!newdata) update();
    newdata = 0;
    ax = accel_mag_raw[0];
    ay = accel_mag_raw[1];
    az = accel_mag_raw[2];
    mx = accel_mag_raw[3];
    my = accel_mag_raw[4];
    mz = accel_mag_raw[5];
    gx = gyro_raw[0];
    gy = gyro_raw[1];
    gz = gyro_raw[2];
    return isOK;
  }
  int readMotionSensor(float * a, float * g) 
  {
    int isOK = update(); // should be 0x7 if data from all 3 sensors
//     if (!newdata) update();
    newdata = 0;
    a[0] = (float)accel_mag_raw[0] * G_PER_COUNT - cal[0];
    a[1] = (float)accel_mag_raw[1] * G_PER_COUNT - cal[1];
    a[2] = (float)accel_mag_raw[2] * G_PER_COUNT - cal[2];
    g[0] = (float)gyro_raw[0] * DEG_PER_SEC_PER_COUNT - cal[3];
    g[1] = (float)gyro_raw[1] * DEG_PER_SEC_PER_COUNT - cal[4];
    g[2] = (float)gyro_raw[2] * DEG_PER_SEC_PER_COUNT - cal[5];
    return isOK;
  }
  int readMotionSensor(float * a, float * g, float * m) 
  {
    int isOK = update(); // should be 0x7 if data from all 3 sensors
//     if (!newdata) update();
    newdata = 0;
    a[0] = (float)accel_mag_raw[0] * G_PER_COUNT - cal[0];
    a[1] = (float)accel_mag_raw[1] * G_PER_COUNT - cal[1];
    a[2] = (float)accel_mag_raw[2] * G_PER_COUNT - cal[2];
    g[0] = (float)gyro_raw[0] * DEG_PER_SEC_PER_COUNT - cal[3];
    g[1] = (float)gyro_raw[1] * DEG_PER_SEC_PER_COUNT - cal[4];
    g[2] = (float)gyro_raw[2] * DEG_PER_SEC_PER_COUNT - cal[5];
    float x = (float)accel_mag_raw[3] * UT_PER_COUNT - cal[6];
    float y = (float)accel_mag_raw[4] * UT_PER_COUNT - cal[7];
    float z = (float)accel_mag_raw[5] * UT_PER_COUNT - cal[8];
    m[0] = x * cal[10] + y * cal[13] + z * cal[14];
    m[1] = x * cal[13] + y * cal[11] + z * cal[15];
    m[2] = x * cal[14] + y * cal[15] + z * cal[12];
    return isOK;
  }

  bool writeCalibration(const void *data);
  void getCalibration(float *offsets, float *softiron=NULL, float *fieldstrength=NULL) {
    if (offsets != NULL) {
      memcpy(offsets, cal, 36);
    }
    if (softiron != NULL) {
      *softiron++ = cal[10];
      *softiron++ = cal[13];
      *softiron++ = cal[14];
      *softiron++ = cal[13];
      *softiron++ = cal[11];
      *softiron++ = cal[15];
      *softiron++ = cal[14];
      *softiron++ = cal[15];
      *softiron++ = cal[12];
    }
    if (fieldstrength != NULL) *fieldstrength = cal[9];
  }
  /**
   * Implement offset in degrees per second.
   * ofss is an array (size 3) of float.
   */
	void setGyroOfs(float * ofss)
  { // in degrees/s
    cal[3] = ofss[0];
    cal[4] = ofss[1];
    cal[5] = ofss[2];
  }
  /**
   * Implement acceleration offset
   * assuming horizontal, prop-shield facing up 
   * ofss is float array with 3 elements (x,y,z) 
   */
  void setAccOfs(float * ofss)
  { // offset in g - z is positive up (when shield is facing up)
    cal[0] = ofss[0];
    cal[1] = ofss[1];
    cal[2] = ofss[2];
  }
  
  /**
   * Get altitude in meters (at standard pressure) */
  float getAltitude()
  {
    return altitude_raw / 16.0;
  }
  /**
   * Get temperature in pressure sensor in degrees celcius */
  float getTemperature()
  {
    return temperature_raw / 16.0;
  }

protected:
  // debug data
  uint8_t alt_ctrl_1[5] = {0};
public:
  bool err_FXOS8700 = false;
  bool err_FXAS21002 = false;
  bool err_MPL3115 = false;
  
protected:
	int update();
	bool FXOS8700_begin();
	bool FXAS21002_begin();
  bool MPL3115_begin(uint8_t osr);
  bool MPL3115_meas_restart();
	bool FXOS8700_read(int16_t *data);
	int FXAS21002_read(int16_t *data);
    /**
     * Read altitude in meter * 16
     * Temperature in deg C, last 4 bit is fraction
     */
	bool MPL3115_read(int32_t *altitude, int16_t *temperature);
	float cal[16]; // 0-8=offsets, 9=field strength, 10-15=soft iron map
	int16_t accel_mag_raw[6];
	int16_t gyro_raw[3];
	int16_t temperature_raw; // from MPL3115
	int32_t altitude_raw; // from MPL3115
	uint8_t newdata;
  /**
   * timing flag */
  bool timingFlag = false;
};

// #undef G_PER_COUNT
// #undef DEG_PER_SEC_PER_COUNT
// #undef UT_PER_COUNT

#endif
