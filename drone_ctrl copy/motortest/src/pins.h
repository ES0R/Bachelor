#ifndef PINS_H
#define PINS_H

// Main pins
#define PIN_LED_DEBUG           13 /* (LED_BUILTIN) move to 7 with shield */
#define PIN_POWER_ROBOT         33
// ADC pins
// convert in progress output (starts with tick time, ends with ADC end)
#define PIN_ADC_CONV_OUT        8
// number of ADC pins used
#define ADC_NUM_ALL             7
// which pins to do ADC on
#define PIN_AD_SENSOR_0         A0  /* 14 */
#define PIN_AD_SENSOR_1         A1  /* 15 */
#define PIN_TEMP_1              A19  /* pin 38 */
#define PIN_TEMP_2              A20  /* pin 39 */
#define PIN_CURRENT_1           A17  /* pin 36 */
#define PIN_CURRENT_2           A18  /* pin 37 */
#define PIN_BATT_VOLT           A6  /* pin 37 */
// Servo pins
#define PIN_ESC_01      3 /* Teensy pin for ESC 1 */
#define PIN_ESC_02      4 /* Teensy pin for ESC 2 */
#define PIN_ESC_03      9 /* Teensy pin for ESC 3 */
#define PIN_ESC_04      10 /* Teensy pin for ESC 4 */
#define PIN_ESC_05      22 /* Teensy pin for ESC 5 (also A8) */
#define PIN_ESC_06      23 /* Teensy pin for ESC 6  (also A9) */

#endif
