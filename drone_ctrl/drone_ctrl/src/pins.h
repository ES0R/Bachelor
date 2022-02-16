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

#ifndef PINS_H
#define PINS_H

// Main pins
#define PIN_LED_DEBUG           17 /* (LED_BUILTIN) moved from 13 with shield */
#define PIN_LED_ARMED           8  /* red */
#define LED_ON                  0 /* low value is on, to supply LED from 5V */
#define LED_OFF                 1 /* low value is on, to supply LED from 5V */
// ADC pins
// convert in progress output (starts with tick time, ends with ADC end)
#define PIN_ADC_CONV_OUT        35  /* debug signal out */
// number of ADC pins used
#define ADC_NUM_ALL             7
// which pins to do ADC on
#define PIN_AD_SENSOR_0         A0  /* 14  RPM sensor 1 */
#define PIN_AD_SENSOR_1         A1  /* 15  RPM sensor 2 */
#define PIN_TEMP_1              A19  /* pin 38 */
#define PIN_TEMP_2              A20  /* pin 39 */
#define PIN_CURRENT_1           A17  /* pin 36 */
#define PIN_CURRENT_2           A18  /* pin 37 */
#define PIN_BATT_VOLT           A2   /* pin 16 */
// ESC/Servo pins
#define PIN_ESC_01      3 /* Teensy pin for ESC 1 */
#define PIN_ESC_02      4 /* Teensy pin for ESC 2 */
#define PIN_ESC_03      9 /* Teensy pin for ESC 3 */
#define PIN_ESC_04      10 /* Teensy pin for ESC 4 */
#define PIN_ESC_05      20 /* Teensy pin for ESC 5 (also A6) */
#define PIN_ESC_06      21 /* Teensy pin for ESC 6  (also A7) */
#define PIN_ESC_07      22 /* Teensy pin for ESC 7  (also A8) */
#define PIN_ESC_08      23 /* Teensy pin for ESC 8  (also A9) */

#define PIN_SENSOR_INT  2 /* prop shield - active low (open drain) */

#define PIN_RX5         34 /* extra  io  for laser height meter */
#define PIN_TX5         34 /* */

#endif
