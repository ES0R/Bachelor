/* main global variables 
 * */

#ifndef MAIN_H
#define MAIN_H

#include <Servo.h>
#include <NXPMotionSense.h>
#include "MadgwickAHRS.h"
#include <Wire.h>
#include <EEPROM.h>


extern NXPMotionSense imu;
extern Madgwick filter; // using the madgwick algo - change to mahony if you have problems with drift

/* functions */
void Transmitter();
float Yaw_counter(float yaw_difference);
void PC_input();
void Dof3PID();
int flightmodes();

void MotorMixHex();
// void MotorMixQuad();

/* Add differet fligth modes */
void flightMode0(); // dis-armed
void flightMode1(); // armed
void failsafe(); // fligthmode 2 = failsafe
void stopAll();
int anti_windup(float a, float b, float c);
void PC_input(int & a, int & b, int & c, int & d);
void blink(); // in Transmitter  
int flightmodes(int a, int input1, int input2, int input3, int input4);
void data_vector();

/******************** Values to edit ************************
 * 
 *      Edite the values below with your Kp, tau_d, tau_i 
 *      And the K from the discrete controller with pre-warping
 * 
 ***********************************************************/
#define loop_time 450000 //45000=2.5ms @ 180Mhz // 36000=2ms
#define pinCount 6
// total number of motors
extern Servo Propeller[pinCount];
extern float pwm_[pinCount];

extern bool mag; // dafault state for magnetometer On/Off = true/false
extern bool dataOn; // dafault state for data On/Off = true/false

/* set your values [roll, pitch, yaw] */
extern float alpha[3];
extern float tau_D[3];
extern float tau_I[3];
extern float kp[3];
extern float T;
extern float K[3]; // calculate this value from the pre-warping ... or if you belive in GOD the use 2/T



/******************************************************
 *                       *
 *      Edit there paremeters if you change 
 *      the inputs from the rc transmitter     
 *                       *
 ******************************************************/
extern float f1[3], f2[3], K1[3], K2[3], K3[3];

extern unsigned long counter[6];
extern byte last_CH_state[5];
extern int input_pin[5];

/******************************************************
 *                       *
 *       inizilazation   
 *                       *
 ******************************************************/
extern float PID_output[3];
extern float error[3];
extern float old_error[3];
extern float Iterm[3];
extern float Dterm[3];
extern float old_I[3];
extern float old_D[3];

extern int throttle;
extern int flightflag;

/*Controller inputs*/
extern float desired_angle[3];
extern float yaw_desired_angle_set;
extern float total_yaw;

/*PC input with serial check PC_input for the commands*/
extern int Serial_input[4];
extern int data_flag;
extern float roll, pitch, yaw, yaw_previous, yaw_difference, last_yaw;
extern float roll_val, pitch_val, yaw_val;
extern float old_roll, old_pitch, old_yaw;
extern float old_Lead[3];

/*Prop shield varibles*/
extern float ax, ay, az;
extern float gx, gy, gz;
extern float mx, my, mz;

// timing the code
extern volatile int cycles;

#endif
