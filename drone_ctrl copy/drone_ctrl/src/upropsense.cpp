/**
 * Copy and slightly modified version of NXPMotionSense/NXPMotionSense.cpp
 * from TeensyDuino version 1.53 (oktober 2020)
 * 
 * Modified to get more data and better calibration use
 * jca@elektro.dtu.dk
 * */

#include "upropsense.h"
#include <utility/NXPSensorRegisters.h>
#include <util/crc16.h>
#include <elapsedMillis.h>
#include "main.h"
#include "sensor.h"
// for KDevelop 
//#include </media/chr/2T/data/Downloads/arduino-1.8.13/hardware/teensy/avr/libraries/NXPMotionSense/utility/NXPSensorRegisters.h>
#include "../libraries/NXPMotionSense/utility/NXPSensorRegisters.h"

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

bool UPropSense::begin()
{
	unsigned char buf[NXP_MOTION_CAL_SIZE];
	uint8_t i;
	uint16_t crc;
	Wire.begin();
	Wire.setClock(1000000);

	memset(accel_mag_raw, 0, sizeof(accel_mag_raw));
	memset(gyro_raw, 0, sizeof(gyro_raw));

	//Serial.println("init hardware");
  // ACC/compass
  for (int i = 0; i < 10; i++)
  {
    if (FXOS8700_begin())
    { // all fine
      err_FXOS8700 = false;
      break;
    }
		delay(100);
    err_FXOS8700 = true;
  }
	if (err_FXOS8700)
    Serial.println("config error acc/compas FXOS8700");
  // GYRO
  for (int i = 0; i < 10; i++)
  {
    if (FXAS21002_begin())
    { // all fine
      err_FXAS21002 = false;
      break;
    }
		delay(100);
    err_FXAS21002 = true;
  }
  if (err_FXAS21002)
    Serial.println("config error gyro FXAS21002");
  //
  // digitalWriteFast(PIN_LED_1, 0);
  for (int i = 0; i < 1; i++)
  {
	  if (MPL3115_begin(1)) 
    { // all OK
      err_MPL3115 = false;
      break;
    }
    delay(100);
    err_MPL3115 = true;
  }
  if (err_MPL3115)
    Serial.println("config error Altitude sensor MPL3115");
  
	//Serial.println("init done");

	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		buf[i] = EEPROM.read(NXP_MOTION_CAL_EEADDR + i);
	}
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, buf[i]);
	}
	if (crc == 0 && buf[0] == 117 && buf[1] == 84) {
		memcpy(cal, buf+2, sizeof(cal));
	} else {
		memset(cal, 0, sizeof(cal));
		cal[9] = 50.0f;
	}
	return true;

}

/**
 * fetch data from all sensors - if available
 * \return flags for valid data */
int UPropSense::update()
{
// 	static elapsedMillis msec;
//     int32_t altitude;
  int isOK = 0;
	if (FXOS8700_read(accel_mag_raw)) { // accel + mag
		//Serial.println("accel+mag");
    isOK = 1;
	}
	if (MPL3115_read(&altitude_raw, &temperature_raw)) 
  { // alt
    MPL3115_meas_restart();
    isOK = +2;
	}
	int g = FXAS21002_read(gyro_raw);
  if (g == 0)
  {  // gyro
		//Serial.println("gyro");
		newdata = 1;
    isOK += 4;
	}
	else
    isOK += (g << 4);
	return isOK;
}


static bool write_reg(uint8_t i2c, uint8_t addr, uint8_t val)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	Wire.write(val);
	return Wire.endTransmission() == 0;
}

static bool read_regs(uint8_t i2c, uint8_t addr, uint8_t *data, uint8_t num)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	if (Wire.endTransmission(false) != 0) 
    return false;
	Wire.requestFrom(i2c, num);
  if (Wire.available() != num) 
    return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
	return true;
}

static bool read_regs(uint8_t i2c, uint8_t *data, uint8_t num)
{
	Wire.requestFrom(i2c, num);
	if (Wire.available() != num) return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
	return true;
}

bool UPropSense::FXOS8700_begin() // acc + mag
{
	const uint8_t i2c_addr=FXOS8700_I2C_ADDR0;
	uint8_t b;

	//Serial.println("FXOS8700_begin");
	// detect if chip is present
	if (!read_regs(i2c_addr, FXOS8700_WHO_AM_I, &b, 1)) return false;
	//Serial.printf("FXOS8700 ID = %02X\n", b);
	if (b != 0xC7) return false;
	// place into standby mode
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0)) return false;
	// configure magnetometer
  // M_CTRL_REG1
  // bit 7: 0=autocalib disabled
  // bit 6: 0= mag reset disabled
  // bit 4,3,2: oversamle rate 7=max
  // bit 1,0: 3=hybrid
	if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG1, 0x1F)) return false; // max over-sample, hybrid mode
	// M_CTRL_REG2
	// bit 5: 1=hybrid data register auto-increase
	// bit 4: 0=no auto calibrate
	// bit 1,0: 0=degauss at every cycle 
	if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG2, 0x20)) return false;
	// configure accelerometer
	if (!write_reg(i2c_addr, FXOS8700_XYZ_DATA_CFG, 0x01)) return false; // 4G range
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG2, 0x02)) return false; // hires (not active in 400/800Hz mode)
	// CTRL_REG1
	// bit 5,4,3: 0=800Hz (400Hz hybrid)
	// bit 2: 1=reduced noise
	// bit 0: 1=active
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0x05)) return false; // 100Hz A+M=0x1D, 200Hz a+m=0x0D, 400Hz A+M=0x05
	//Serial.println("FXOS8700 Configured");
	return true;
}

bool UPropSense::FXOS8700_read(int16_t *data)  // accel + mag
{
	static elapsedMicros usec_since;
// 	static int32_t usec_history=5000;
	const uint8_t i2c_addr=FXOS8700_I2C_ADDR0;
	uint8_t buf[13];

// 	int32_t usec = usec_since;
// 	if (usec + 100 < usec_history) return false;

	if (!read_regs(i2c_addr, FXOS8700_STATUS, buf, 1)) 
    // read failed
    return false;
	if (buf[0] == 0) 
    // data not valid
    return false;

// 	usec_since -= usec;
// 	int diff = (usec - usec_history) >> 3;
// 	if (diff < -15) diff = -15;
// 	else if (diff > 15) diff = 15;
// 	usec_history += diff;
  // debug timing
  sensor->controlUsedTime[9] = ARM_DWT_CYCCNT;
  // debug end
	if (!read_regs(i2c_addr, FXOS8700_OUT_X_MSB, buf+1, 12)) return false;
	//if (!read_regs(i2c_addr, buf, 13)) return false;
  
	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	data[3] = (int16_t)((buf[7] << 8) | buf[8]);
	data[4] = (int16_t)((buf[9] << 8) | buf[10]);
	data[5] = (int16_t)((buf[11] << 8) | buf[12]);
	return true;
}

// /////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////

bool UPropSense::FXAS21002_begin() // gyro
{
        const uint8_t i2c_addr=FXAS21002_I2C_ADDR0;
        uint8_t b;

	if (!read_regs(i2c_addr, FXAS21002_WHO_AM_I, &b, 1)) return false;
	//Serial.printf("FXAS21002 ID = %02X\n", b);
	if (b != 0xD7) 
    return false; // not this device

	// place into standby mode
	if (!write_reg(i2c_addr, FXAS21002_CTRL_REG1, 0)) return false;
	// switch to active mode, 100/400/800 Hz output rate
	if (!write_reg(i2c_addr, FXAS21002_CTRL_REG0, 0x00)) return false;
  // CTRL_REG1
  // bit 6 reset, 0=not
  // bit 5 self test, 0=not
  // bit 4,3,2 data rate  000 = 800 Hz, 001=400Hz, 011=100Hz
  // bit 1 active, 1=active
  // bot 0 ready, don't care if bit 1 is set to active 
	if (!write_reg(i2c_addr, FXAS21002_CTRL_REG1, 0x06)) 
    return false;
  // CTRL_REG2 - interrupt
  // bit 3, 0 = INT2 pin
  // bit 2, 1 = interrupt enabled
  // bit 1, 0=active low, 1=active high
  // bit 0, 1 = open drain, when bit1 is 0 (else open source when bit 1 is 1)
  // reg=0x05 is int2 enabled as active low (open drain)
  // reg=0x04 is int2 enabled as active low (totem output)
  // reg=0x06 is int2 enabled as active high (totem output)
  if (!write_reg(i2c_addr, FXAS21002_CTRL_REG2, 0x05)) 
    return false;
  
	//Serial.println("FXAS21002 Configured");
	return true;
}

int UPropSense::FXAS21002_read(int16_t *data) // gyro
{
	static elapsedMicros usec_since;
	static int32_t usec_history=10000;
	const uint8_t i2c_addr=FXAS21002_I2C_ADDR0;
	uint8_t buf[7];

	int32_t usec = usec_since;
//  	if (usec + 100 < usec_history) return 1;

	if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 1)) 
    return 2; // i2c error
	if (buf[0] == 0) 
    return 3; // no data ready

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -15) diff = -15;
	else if (diff > 15) diff = 15;
	usec_history += diff;
	//Serial.println(usec);

	if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 7)) 
    return 4; // i2c error
	//if (!read_regs(i2c_addr, buf, 7)) return false;

	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	return 0;
}


// /////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////

bool UPropSense::MPL3115_meas_restart() // pressure
{
  const uint8_t i2c_addr=MPL3115_I2C_ADDR;
  uint8_t b;
  uint8_t osrReg;
  
  if (!read_regs(i2c_addr, MPL3115_CTRL_REG1, &b, 1)) return false;
  // clear OST and not active
  osrReg = b & 0xfd;
  if (!write_reg(i2c_addr, MPL3115_CTRL_REG1, osrReg)) return false;
  // start new measurement
  osrReg = b | 0x02;
  if (!write_reg(i2c_addr, MPL3115_CTRL_REG1, osrReg)) return false;
  return true;
}


bool UPropSense::MPL3115_begin(uint8_t osr) // pressure
{
  const uint8_t i2c_addr=MPL3115_I2C_ADDR;
  uint8_t b;
  uint8_t osrReg = (osr << 3) | 0x80;

  if (!read_regs(i2c_addr, MPL3115_WHO_AM_I, &b, 1)) return false;
  //Serial.printf("MPL3115 ID = %02X\n", b);
  if (b != 0xC4) return false;

  if (!read_regs(i2c_addr, MPL3115_CTRL_REG1, &b, 1)) return false;
  alt_ctrl_1[0] = b;
  // set OSR and not active
  if (!write_reg(i2c_addr, MPL3115_CTRL_REG1, osrReg)) return false;
  // place into standby mode by reset
  // if (!write_reg(i2c_addr, MPL3115_CTRL_REG1, 0)) return false;
//   delay(10);
  if (!read_regs(i2c_addr, MPL3115_CTRL_REG1, &b, 1)) return false;
  alt_ctrl_1[1] = b;
  delay(2);
  if (!read_regs(i2c_addr, MPL3115_SYSMOD, &b, 1)) return false;
  // debug (LED on = standby)
  alt_ctrl_1[2] = b;
  if (false)
  {
    digitalWriteFast ( PIN_LED_ARMED, LED_ON);
    for (int i = 1; i < 10; i++)
    { // waiting for standby
      if (!read_regs(i2c_addr, MPL3115_SYSMOD, &b, 1)) return false;
      // debug (LED on = standby)
      alt_ctrl_1[1] = b;
      alt_ctrl_1[2] = i;
      if (b == 0)
      { // in stb
        digitalWriteFast ( PIN_LED_ARMED, LED_OFF);
        break;
      }
      digitalWriteFast ( PIN_LED_ARMED, LED_OFF);
      delay(200);
      digitalWriteFast ( PIN_LED_ARMED, LED_ON);
      delay(200);
  //     else
  //       // place into standby mode
  //       if (!write_reg(i2c_addr, MPL3115_CTRL_REG1, 0)) return false;
    }
  }
  // enable events
  if (!write_reg(i2c_addr, MPL3115_PT_DATA_CFG, 0x07)) return false;
  // set active
  // 0xB9 =  switch to active, altimeter mode, 512 ms measurement, polling mode
  // 0x89 =  switch to active, altimeter mode,  10 ms measurement, polling mode
  // 0x81 =  switch to active, altimeter mode,   6 ms measurement, polling mode
  if (!write_reg(i2c_addr, MPL3115_CTRL_REG1, osrReg | 0x01)) return false; // including activate

  if (!read_regs(i2c_addr, MPL3115_CTRL_REG1, &b, 1)) return false;
  alt_ctrl_1[3] = b;
  if (!read_regs(i2c_addr, MPL3115_SYSMOD, &b, 1)) return false;
  // debug (LED on = standby)
  alt_ctrl_1[4] = b;
  //Serial.println("MPL3115 Configured");
  return true;
}

bool UPropSense::MPL3115_read(int32_t *altitude, int16_t *temperature)
{
// 	static elapsedMicros usec_since;
// 	static int32_t usec_history=980000;
	const uint8_t i2c_addr=MPL3115_I2C_ADDR;
	uint8_t buf[6];

// 	int32_t usec = usec_since;
// 	if (usec + 500 < usec_history) return false;

	if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 1)) return false;
	if (buf[0] == 0) return false;
    // new data available
	if (!read_regs(i2c_addr, buf, 6)) return false;

// 	usec_since -= usec;
// 	int diff = (usec - usec_history) >> 3;
// 	if (diff < -1000) diff = -1000;
// 	else if (diff > 1000) diff = 1000;
// 	usec_history += diff;

	int32_t a = ((uint32_t)buf[1] << 12) | ((uint16_t)buf[2] << 4) | (buf[3] >> 4);
  // sign bit expansion
	if (a & 0x00080000) 
    a |= 0xFFF00000;
  // return result
	*altitude = a; // altitude in m, but last 4 bit are the fraction part
	*temperature = (int16_t)((buf[4] << 4) | buf[5] >> 4); // temperature in deg C, last 4 bit is fraction

	//Serial.printf("%02X %d %d: ", buf[0], usec, usec_history);
	//Serial.printf("%6d,%6d", a, *temperature);
	//Serial.println();
	return true;
}

bool UPropSense::writeCalibration(const void *data)
{
	const uint8_t *p = (const uint8_t *)data;
	uint16_t crc;
	uint8_t i;

	if (p[0] != 117 || p[1] != 84) return false;
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, p[i]);
	}
	if (crc != 0) return false;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		EEPROM.write(NXP_MOTION_CAL_EEADDR + i, p[i]);
	}
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		if (EEPROM.read(NXP_MOTION_CAL_EEADDR + i) != p[i]) return false;
	}
	memcpy(cal, ((const uint8_t *)data)+2, sizeof(cal));
	return true;
}

