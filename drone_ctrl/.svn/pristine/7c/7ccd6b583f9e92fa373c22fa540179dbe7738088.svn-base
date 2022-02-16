/***************************************************************************
*   Copyright (C) 2021 by DTU                             *
*   jca@elektro.dtu.dk                                                    *
*
*   Main file for drone control (has the main loop, arduino style)
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


#include <core_pins.h>
#include <malloc.h>
#include <Wire.h>
#include <IntervalTimer.h>
#include <ADC.h>
#include "src/pins.h"
#include "src/main.h"
#include "src/eeconfig.h"
#include "src/command.h"
#include "src/uesc.h"
#include "src/sensor.h"
#include "src/upropshield.h"
#include "src/control.h"
#include "src/mixer.h"
#include "src/logger.h"
#include "src/usbus.h"
#include "src/usubscribe.h"
#include "src/ustate.h"
#include "src/uled.h"
#include "src/ultrasound.h"
#include "uheight.h"
#include "ulaser.h"

// #define FASTLED_TEENSY3
// typedef volatile uint32_t RwReg;
// typedef uint32_t RoReg;
// #include <SPI.h>
// #include <FastLED.h>


// main heartbeat timer to service source data and control loop interval
IntervalTimer hbTimer;
//
int16_t robotId = 1;
/// has positive reply been received frrom IMU
bool imuAvailable = false;
// battery low filter
uint16_t batVoltInt = 0;
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period)
volatile uint32_t hb10us = 0;     /// heart beat timer count (10us)
unsigned int cycleTimerInUs = 10; /// heartbeat timer interrupt every (us)
// flag for start of new control period
volatile bool startNewCycle = false;
// volatile bool startADC = false;
int adcFullCycles = 0;
int adcFullCyclesMax = 4;
uint32_t startCycleCPU;
//
//uint32_t controlUsedTime[3] = { 0, 0, 0 }; // third item is acceleration limit is reached (active)
float steerWheelAngle = 0;
// Heart beat interrupt service routine
void hbIsr ( void );
///
UCommand * command = new UCommand();
EEConfig * eeConfig = new EEConfig();
UEsc * esc = new UEsc();
USensor * sensor = new USensor();
UPropShield * imu = new UPropShield();
UControl * control = new UControl();
UMixer * mixer = new UMixer();
ULogger * logger = new ULogger();
USbus * rc = new USbus();
USubscribe * subscribe = new USubscribe();
UState * state = new UState();
ULEDs * leds = new ULEDs();
// height
UUltrasound * uhgt = new UUltrasound();
ULaser * uhlas = new ULaser();
UHeight * hgt = new UHeight();
/// main loop counter
const int EEPROM_SIZE = 2024;

/*
#define NUM_LEDS 50
CRGB leds[NUM_LEDS];
*/

// ////////////////////////////////////////

void setup()   // INITIALIZATION
{
  bool isOK = true;
  pinMode ( PIN_LED_DEBUG, OUTPUT );
  pinMode ( PIN_LED_ARMED, OUTPUT );
  pinMode ( PIN_ADC_CONV_OUT, OUTPUT );
  pinMode(PIN_TX5, OUTPUT);
  // RED LED until setup
  digitalWriteFast ( PIN_LED_ARMED, LED_ON);
  // init USB connection (parameter is not used - always 12MB/s)
  Serial.begin ( 115200 ); // USB init serial
  analogWriteResolution ( 12 ); // set PWM resolution
  sensor->InitSensor();
  esc->initEsc();     // set PWM for available servo pins
  imu->setup();
  if (not imu->boardOK)
  { // then motor test stand, that require more cycles 
    // to count RPM
    adcFullCyclesMax = 10;
  }
  mixer->setup();
  control->setup();
  logger->setup();
  rc->setup();
  subscribe->setup();
  state->setup();
  leds->setup();
  uhgt->setup();
  uhlas->setup();
  hgt->setup();
  isOK = eeConfig->eePromLoadStatus();
  // start 10us timer (heartbeat timer)
  hbTimer.begin ( hbIsr, cycleTimerInUs ); // heartbeat timer, value in usec
  // on-board LEDs
  digitalWriteFast ( PIN_LED_DEBUG, LED_ON );
  if (isOK) // and not (imu->err_FXAS21002 or imu->err_FXOS8700 or imu->err_MPL3115))
    digitalWriteFast ( PIN_LED_ARMED, LED_OFF);
  //
  // init CPU cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

/**
 * Blink status LED on board */
void statusBlink(int mainLoop)
{
  if (mainLoop % (100000/CONTROL_PERIOD_10us) == 0)
    digitalWriteFast(PIN_LED_DEBUG, LED_ON);
  else if (mainLoop % (100000/CONTROL_PERIOD_10us) == 20)
    digitalWriteFast(PIN_LED_DEBUG, LED_OFF);
}


/**
* Main loop
* input-output using USB 
* synchronization with heartbeat for
* sensor processing and actuator control.
*/
void loop(void)
{
  uint32_t thisCycleStartTime = 0;
  uint32_t thisADCStartTime = 0;
  bool cycleStarted = false;
  uint32_t mainLoop = 0; // sample time (2.5ms) loop count
  // set to start ADC conversion
  sensor->adcSeq = ADC_NUM_ALL;
  // activate control (just control loops, not output to esc)
  control->resetControl();
  control->controlActive = true;
  int skipped_ticks = 0;
  uint32_t usb_log_send_time = 0;
  bool logGetStarted = false;
  int logLines = 0;
  while ( true ) 
  { // main loop
    if (sensor->adcSeq >= ADC_NUM_ALL)
    {
      if (adcFullCycles < adcFullCyclesMax)
      { // ADC sample as fast as possible for filtering (current, battery, RPM, temperature, ...)
        sensor->adcTick();
        // save ADC sample time
        sensor->controlUsedTime[3] = ARM_DWT_CYCCNT - thisADCStartTime;
        thisADCStartTime = ARM_DWT_CYCCNT;
  //       startADC = false;
                adcFullCycles++;
      }
    }
//     else if (sensor->adcFinished)
//     { // there is more to ADC before an ADC-tick is needed
// //       delayMicroseconds(10);
//       sensor->adcFinished = false;
//       digitalWriteFast ( PIN_ADC_CONV_OUT, HIGH );
//       adc->startSingleRead ( adcPin[sensor->adcSeq] );
//     }
    // async read from IMU sensors
    if (imu->boardOK)
      imu->asyncRead();
    // listen to USB for commands
    //if (not logGetStarted)
    {  // wait with incoming while sending log
      command->handleIncoming();
    }
    // get data from RC (using FUTUBA SBUS)
    if (not startNewCycle)
      rc->tick(hbTimerCnt);
    //
    if ( startNewCycle ) // start of new control cycle
    { // control cycle sample time (in CPU clocks)
      digitalWriteFast ( PIN_ADC_CONV_OUT, HIGH );
      uint32_t tCPU = ARM_DWT_CYCCNT;
      sensor->controlUsedTime[4] = tCPU - thisCycleStartTime;
      thisCycleStartTime = tCPU;
      sensor->controlUsedTime[5] = thisCycleStartTime;
      // sample flags
      startNewCycle = false;
      cycleStarted = true;
      // state control - init, disarmed, armed etc.
      state->tick();
      // Prop shield (IMU) sensor process
      if (not (imu->err_FXAS21002 or imu->err_FXOS8700 or imu->err_MPL3115))
      { // no error, so get sensor data
        imu->tick(false, mainLoop);
        // if not - probably just testing software in blank Teensy
      }
      if (true)
      { // read new AD sensor values (bat voltage, RPM sensor, current, temperature)
        sensor->readSensorTick(mainLoop);
        // read height sensor(s)
        uhgt->readHeight(); // ultrasound
        uhlas->readHeight(); // laser
        // combine height measurements
        hgt->combineHeight(mainLoop);
        // record read sensor time (CPU clocks)
        sensor->controlUsedTime[0] = ARM_DWT_CYCCNT - thisCycleStartTime;
        // run controller
        control->controlTick();
        // convert to motor values
        mixer->tick();
        // Implement on actuators
        esc->escTick();
        // record read sensor time + control time + implement
        sensor->controlUsedTime[1] = ARM_DWT_CYCCNT - thisCycleStartTime;
        // data logging
        logger->tick(mainLoop);
        // subscription data service
        subscribe->tick(mainLoop, logGetStarted);
        // leds may have changed
        leds->tick(mainLoop);
      }
      // blink once a second
      statusBlink(mainLoop);
      // end tick loop timing
      // save ADC sample time in this cycle
      sensor->adc_us = sensor->controlUsedTime[3] / sensor->c_us;
//       sensor->adc_per_cycle = float(CONTROL_PERIOD_10us * cycleTimerInUs) / sensor->adc_us;
      mainLoop++;
      digitalWriteFast ( PIN_ADC_CONV_OUT, LOW );
    }
    // loop end time
    if (cycleStarted)
    { // time spend in full control cycle (CPU clocks)
      sensor->controlUsedTime[2] = ARM_DWT_CYCCNT - thisCycleStartTime;
      cycleStarted = false;
//       startADC = true;
            adcFullCycles = 0;
      imu->asyncGyroCnt =0;
      imu->asyncAccMagCnt =0;
      imu->asyncAirPressureCnt =0;     
      // send log data if there is space in USB write buffer, else wait.
      int bf = usb_serial_write_buffer_free();
      if (logger->dataLogOut /*and bf >= 5*/) //           and mainLoop % 10 == 0)
      {
        uint32_t t1 = ARM_DWT_CYCCNT;
        //bool sendOK = 
        logger->sendLogToClient(skipped_ticks, usb_log_send_time / (F_CPU / 1000000), bf);
//         if (sendOK)
        {
          usb_log_send_time = ARM_DWT_CYCCNT - t1;
          skipped_ticks = 0;
          logGetStarted = true;
          logLines++;
        }
//         else
//         {
//           skipped_ticks++;
//           if (skipped_ticks == 1)
//             command->usb_send_str("# logger flow control skip started\n");
//           if (skipped_ticks > 400)
//           {
//             logger->flowCtrl = false;
//             command->usb_send_str("# logger flow control resumed (tick > 400)\n");
//           }
//         }
      }
      else
        skipped_ticks++;
      if (logGetStarted and not logger->dataLogOut)
      { // finished sending log
        const int MSL = 300;
        char s[MSL];
        snprintf(s, MSL, "# log send %d lines\n", logLines);
        command->usb_send_str(s);
        logGetStarted = false;
        logLines = 0;
      }
    }
  }
}

// This part is handled by Arduino main()
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
//   if ( hb10us % (CONTROL_PERIOD_10us/4) == 1 )
//   { // time for ADC
//     startADC = true;
//   }
  if ( hb10us % CONTROL_PERIOD_10us == 0 ) 
  { // main control period start
    hbTimerCnt++;
    startNewCycle = true;
    startCycleCPU = ARM_DWT_CYCCNT;
  }
}
