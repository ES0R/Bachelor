/***************************************************************************
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
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

#ifndef CONTROL_BASE_H
#define CONTROL_BASE_H

#include <string.h>
#include <stdlib.h>
#include "WProgram.h"
#include "logger.h"

// whenever mission control type is changed
// then old values may be invalid
//extern bool historyInvalid;

class UTransferFunctionPZ
{
public:
  /**
   * constructor */
  UTransferFunctionPZ();
  /**
   * Init discrete control parameters */
  void initControl();
  /**
   * Reset control - i.e. set all old values to zero */
  void resetControl();
  /**
   * Set parameters */
  void setParamsTauTau(float tau_numerator, float tau_denominator, float limit);
  /**
   * decode from 3 values in a string 
   * \param line format is "1 0.22 0.022" for "use=true tauNumerator=0.22 tauDenominator=0.022"
   * \returns pointer to next unused character on line, or NULL if format error */
  const char * decodeFromString(const char * line);
  /**
   * Set parameters */
//   void setParamsTauAlpha(float tau, float alpha, float limit);
  /**
  * Get parametres to string */
  int getToString(char * buf, int bufCnt);
  /**
   * Do calculate control */
  void controlTick();
  /**
   * save to ee-prom (or string) the configuration
   * of this controller part */
  void eePromSave();
  /**
   * read configuration from ee-prom (or string)
   * to this controller part */
  void eePromLoad();
  
public:
  /** 
   * Should this regulator be used at all */
  bool inUse;
  /** differentiator time constant bay be used direct, if a gyro (differentiated) input exist.*/
  float tau_num;
protected:
  /**
   * Pole time constant for filter */
  float tau_den;
  /**
   * Sample time - is constant in this case */
  float sampleTime;
//   float limit;
//   bool filter_use;
//   bool output_limit;
private:
  /** control factor for numerator */
  float ze[2];
  /** control factor for denominator */
  float zu[2];
public:
  /** current control input values [0]=current [1]=last value */
  float x[2];
  /** current control output values [0]=current [1]=last value */
  float y[2];
};

class UTransferFunctionI
{
public:
  /**
   * constructor */
  UTransferFunctionI();
  /**
   * Init discrete control parameters */
  void initControl();
  /**
   * Reset control - i.e. set all old values to zero */
  void resetControl();
  /**
   * Set parameters */
  void setParamsIntegrator(float tau, float limit);
  /**
   * decode from 3 values in a string 
   * \param line format is "1 0.22 4.5" for "use=true tau=0.22 limit=4.5"
   * \returns pointer to next unused character on line, or NULL if format error */
  const char * decodeFromString(const char * line);
  /**
   * Get parametres to string */
  int getToString(char * buf, int bufCnt);
  /**
   * Do calculate control */
  void controlTick();
  /**
   * save tau and limit to e-prom - if in use */
  void eePromSave();
  /**
   * read configuration from ee-prom (or string)
   * to this controller part */
  void eePromLoad();
  
public:
  bool inUse;
  // should input to integrator be added to outpot - generation a zero
  bool andZero;
protected:
//   float tau_num;
  float tau_den;
  float sampleTime;
  float limit;
//   bool filter_use;
  bool limit_use;
private:
  /** control factor for numerator */
  float ze[2];
  /** control factor for denominator */
  float zu[2];
public:
  /** current control input values [0]=current [1]=last value */
  float x[2];
  /** current control output values [0]=current [1]=last value */
  float y[2];
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

class UControlBase
{
public:
  /**
   * constructor */
  UControlBase(const char * idKey);
  /**
  * init regulator from string 
  * \param line is a formatted string:
  *     "rxx use kp  ui ti iLimit  u1 t1n t1d  u2 t2n t2d   u3 t3n t3d u3i t3i lim3  ...
  *          ffuse kp4 u4 t4n t4d  lim_use limit rate_use rate_limit"
  * where the part in quotes are the line parameter, and:
  * use is controller active, else factor one from in to out
  * kp is proportional gain
  * ui ti iLimit is integrator use, time constant, and i limit (1e10 is no limit)
  * u1 t1n, t1d is Lead fwd: use, numerator,denominator time constants in  (t1n*s + 1)/(t1d*s + 1)
  * u2 t2n, t2d is Lead back use, numerator,denominator time constants in  (t2n*s + 1)/(t2d*s + 1) 
  * u3 t3n, t3d is pre-filter use, numerator,denominator time constants in  (t3n*s + 1)/(t3d*s + 1)
  * u3i t3i, lim3 is pre-filter i_term: use, tau_i, and limit
  * ffuse kp4 is feed forward use and proportional gain 
  * u4 t4n, t4d is feed forward use, numerator,denominator time constants in  (t4n*s + 1)/(t4d*s + 1)
  * lim_use is output limiting active
  * u_limit is output limit value
  * rate_use is rate limit active
  * rate_limit for reference value >1e10 marks do not use
  * \returns true initial key matches this regulator */
   bool setRegulator(const char * line);
   /**
    * Set pointer to input and output of controller
    * \param referenceInput reference value in same scaling as measurement input
    * \param measurementInput is measured value that controller should try to keep be equal to reference
    * \param outputValue is output of the controller - affecting the measurement
    * \param gyro is differentiated measurement input, that is used by lead in back-branch, (if not NULL) in place of the pole-zero Lead filter
    * */
   void setInputOutput(float * referenceInput, float * measurementInput, float * outputValue, float * gyroInput = NULL);
   /**
    * Get regulator parameters (as set earlier)
    * \param buf is string buffer to fill 
    * \param bufCnt is buffer length
    * \return used characters */
   int getRegulator(char * buf, int bufCnt);
   /**
    * reset controller, i.e. set all old values to zero */
   void resetControl();
   /**
    * control tick 
    * This will calculate the full controller. */
   void controlTick(bool logExtra = false);
   /**
    * save configiration to ee-prom (or string) */
   void eePromSave();
   /**
    * load configiration from ee-prom (or string) */
   void eePromLoad();
   /** is this key me? */
   bool isMe(const char * id);
   /**
    * get key */
   const char * getKey()
   {
     return key;
   }
   /**
    * set output limit to enabled with this value */
//    void setOutputLimit(float min, float max)
//    {
//      outLimitUse = true;
//      outLimitMin = min;
//      outLimitMax = max;
//    }
   /**
    * get limit value for controller output */
   float getOutLimitMin()
   {
     return outLimitMin;
   }
   /**
    * get limit value for controller output */
   float getOutLimitMax()
   {
     return outLimitMax;
   }
   /**
    * add control values to log
    * \param item is is for log set of values */
   void toLog(CtrlLogData * entry);
   /**
    * send log to USB client
    * \param data is array of data 
    * \param cnt is number of entries
    * \param dataInterval is data interval in units of 10 us */
   void getCtrlLog(CtrlLogData * data, int cnt, int dataInterval);
   
public:
   /** use regulator, if false then no connection from input to output */
   bool use;
   /** is the output limit in use, this can flag that 
    * integrator should not integrate - as in balance velosity */
   bool outLimitUsed;
   
protected:
  static const int MKL = 8;
  /** key when receiving or sending settings about this controller */
  char key[MKL];
  /// proportional gain
  float kp;
  /// feed forward
  bool ffUse;
  float ffKp;
  /// output limit
  bool outLimitUse;
  float outLimitMax;
  float outLimitMin;
  /// rate limiter
//   bool rateLimitUse;
//   float rateLimit;
  /** input output */
  float * input; /// input reference value
  float * measurement; /// measurement value
  float * gyro; /// extra input for balance controller (uses tau_num only then)
  float * output; /// actuator output
  bool plusMinusPiCheck; // is reference an angle, there may be need of a folding check
  // adjust angle to remove discontinuity at +/- PI
  // bool angleAdjust;
public:
  /** controller elements */
  UTransferFunctionPZ preFilt;
//   UTransferFunctionI postFiltI;
//   UTransferFunctionPZ ffFilt;
  UTransferFunctionI integrator;
  UTransferFunctionPZ leadFwd;
  UTransferFunctionPZ leadBack;
  UTransferFunctionPZ ffFilt;
protected:
  /// control varaibles (interim values used when logging control data
  float preOut;  /// pre filter output
  float backEst; /// estimated measurement (after potential lead)
  float eu;       /// error (ref - measurement)
  float u1, u;    /// control signal before and after post-integrator and limit
};

#endif
