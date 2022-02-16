/***************************************************************************
*   Copyright (C) 2019 by DTU                             *
*   jca@elektro.dtu.dk                                                    *
*
*   Main function for small regulation control object (regbot)
*   build on Teensy 3.1 or higher,
*   intended for 31300/1 Linear control
*   has an IMU and a dual motor controller with current feedback.
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

// #define REV_ID "$Id: main.cpp 1055 2019-05-25 17:10:51Z jcan $"

#include <malloc.h>
#include <ADC.h>
#include "src/pins.h"
#include "IntervalTimer.h"
#include "src/main.h"
#include "src/eeconfig.h"
#include "src/command.h"
#include "src/uesc.h"
#include "src/sensor.h"

#ifndef REGBOT_HW4
#pragma message "REGBOT v3 and older COMPILE"
#endif

// main heartbeat timer to service source data and control loop interval
IntervalTimer hbTimer;
//
int16_t robotId = 1;
/// has positive reply been received frrom IMU
bool imuAvailable = false;
// battery low filter
uint16_t batVoltInt = 0;
bool batteryUse = false;
bool batteryHalt = false;
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period)
volatile uint32_t hb10us = 0;     /// heart beat timer count (2 us)
unsigned int cycleTimerInUs = 10; /// heartbeat timer interrupt every (us)
// flag for start of new control period
volatile bool startNewCycle = false;
uint32_t startCycleCPU;
//
//uint32_t controlUsedTime[3] = { 0, 0, 0 }; // third item is acceleration limit is reached (active)
float steerWheelAngle = 0;
// Heart beat interrupt service routine
void hbIsr ( void );
///
UEsc * esc = new UEsc();
USensor * sensor = new USensor();
/// main loop counter
uint32_t mainLoop = 0;
const int EEPROM_SIZE = 2024;

//const bool useADCInterrupt = true;
// const int useADCresolution = 12;
//const float lpFilteredMaxADC = 4095*2;	// ADC returns 0->4095
// AD conversion
//void hbIsr ( void );
///
// ADC * adc = new ADC();
// uint16_t adcInt0Cnt = 0;
// uint16_t adcInt1Cnt = 0;
// uint16_t adcStartCnt = 0;
// uint16_t adcValue[ADC_NUM_ALL];
// uint32_t adcStartTime, adcConvertTime;
// int adcPin[ADC_NUM_ALL] =
// {
//   PIN_AD_SENSOR_0,
//   PIN_AD_SENSOR_1,
//   PIN_BATTERY_VOLTAGE,
//   PIN_LEFT_MOTOR_CURRENT,
//   PIN_RIGHT_MOTOR_CURRENT
// };
// int adcSeq = 0;


// ////////////////////////////////////////

void setup()   // INITIALIZATION
{
  pinMode ( PIN_LED_DEBUG, OUTPUT );
  pinMode ( PIN_ADC_CONV_OUT, OUTPUT );
  // init USB connection (parameter is not used - always 12MB/s)
  Serial.begin ( 115200 ); // USB init serial
  analogWriteResolution ( 12 ); // set PWM resolution
  sensor->InitSensor();
  esc->initEsc();     // set PWM for available servo pins
  // start 10us timer (heartbeat timer)
  hbTimer.begin ( hbIsr, cycleTimerInUs ); // heartbeat timer, value in usec
  // data logger init
  // I2C init
  // Setup for Master mode, pins 18/19 (V4, older versions use pins 16/17) external pullups, 400kHz
//  Wire.begin ( I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400 );
  // Initialization of MPU9150
  digitalWriteFast ( PIN_LED_DEBUG, 1 );
  imuAvailable = true; // MPU9150_init();
  digitalWriteFast ( PIN_LED_DEBUG, 0 );
  if ( not imuAvailable )
    usb_send_str ( "# failed to find IMU\r\n" );
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  if ( true )
    eeConfig.eePromLoadStatus ( NULL );
  //
  // init CPU cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

// void batteryMonitoring()
// {
// };

/**
* Main loop
* primarily for initialization,
* non-real time services and
* synchronisation with heartbeat.*/
void loop(void)
{
  uint32_t thisCycleStartTime = 0;
  uint32_t thisADCStartTime = 0;
  // run initialization
  setup();
  bool cycleStarted = false;
  uint32_t tickCnt = 0;
  // set to start ADC conversion
  sensor->adcSeq = ADC_NUM_ALL;
  //sensor->adcTick();
  while ( true ) 
  { // main loop
    // start ADC loop
    if (sensor->adcSeq >= ADC_NUM_ALL)
    {
      sensor->controlUsedTime[3] = ARM_DWT_CYCCNT - thisADCStartTime;
      thisADCStartTime = ARM_DWT_CYCCNT;
//       usb_send_str("# new ADC cycle\r\n");
      sensor->adcTick();
    }
    // listen to USB for commands
    handleIncoming(mainLoop);
    //
    if ( startNewCycle ) // start of new control cycle
    { // control and sensor cycle
      sensor->controlUsedTime[4] = ARM_DWT_CYCCNT - thisCycleStartTime;
      thisCycleStartTime = startCycleCPU;
      startNewCycle = false;
      cycleStarted = true;
      // read new sensor values
      sensor->readSensorTick();
      // record read sensor time
      sensor->controlUsedTime[0] = ARM_DWT_CYCCNT - thisCycleStartTime;
      // Implement on actuators
      esc->escTick();
      // record read sensor time + control time
      sensor->controlUsedTime[1] = ARM_DWT_CYCCNT - thisCycleStartTime;
      tickCnt++;
      if (tickCnt %100 == 0)
        digitalWriteFast(PIN_LED_DEBUG, 1);
      else if (tickCnt %100 == 20)
        digitalWriteFast(PIN_LED_DEBUG, 0);
      // filter timing
      sensor->adc_us = sensor->controlUsedTime[3] / sensor->c_us;
      sensor->adc_per_cycle = float(CONTROL_PERIOD_10us * cycleTimerInUs) / sensor->adc_us;
    }
    mainLoop++;
    // loop end time
    if (cycleStarted)
    {
      sensor->controlUsedTime[2] = ARM_DWT_CYCCNT - thisCycleStartTime;
      cycleStarted = false;
    }
  }
  //return 0;
}

// extern "C" int main ( void )
// {
//   setup();
//   while (true)
//     loop();
// }

/**
* Heartbeat interrupt routine
* schedules data collection and control loop timing.
* */
void hbIsr ( void ) // called every 10 microsecond
{
  // as basis for all timing
  hb10us++;
  if ( hb10us % CONTROL_PERIOD_10us == 0 ) // 1ms timing - main control period start
  {
    missionTime += 1e-5 * CONTROL_PERIOD_10us;
    hbTimerCnt++;
    startNewCycle = true;
    startCycleCPU = ARM_DWT_CYCCNT;
  }
}

/////////////////////////////////////////////////////////////////

// // If you enable interrupts make sure to call readSingle() to clear the interrupt.
// void adc0_isr()
// {
//   uint16_t v = adc->readSingle ( ADC_0 );
//   if ( adcSeq < ADC_NUM_ALL )
//   { // save value
//     adcValue[adcSeq] = v;
//   }
//   adcSeq++;
//   if ( adcSeq < ADC_NUM_ALL ) // start new and re-enable interrupt
//   { // start next conversion
//     adc->startSingleRead ( adcPin[adcSeq] );
//   }
//   else     // finished
//   {
//     adcConvertTime = hb10us - adcStartTime;
//     digitalWriteFast ( PIN_ADC_CONV_OUT, LOW );
//   }
//   adcInt0Cnt++;
// }
// // 
// // //////////////////////////////////////////////////////////
// // 
// void adc1_isr()
// {
//   uint16_t v = adc->readSingle ( ADC_1 );
//   if ( adcSeq < ADC_NUM_ALL )
//   { // IR sensor use RAW AD - averaged later
//     adcValue[adcSeq] = v;
//   }
//   adcSeq++;
//   if ( adcSeq < ADC_NUM_ALL ) // start new and re-enable interrupt
//   { // start conversion
//     adc->startSingleRead ( adcPin[adcSeq] );
//   }
//   else     // finished
//   { // save convert time
//     adcConvertTime = hb10us - adcStartTime;
//     digitalWriteFast ( PIN_ADC_CONV_OUT, LOW );
//   }
//   adcInt1Cnt++;
// }


