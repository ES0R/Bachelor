
/***************************************************************************
 *   Copyright (C) 2019-2021 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#ifndef UACCGYRO_H_INCLUDED
#define UACCGYRO_H_INCLUDED

#include <udata.h>


class UBridge;

/**
 * base class for data items from Accelerometer and Gyro */
class UAccGyro : public UData
{
public:
  float acc[3] = {0};  // filtered drone coordinates (SI, m/s^2)
  float accW[3] = {0}; // world coordinates (using Madge filter)
  float gyro[3] = {0}; // drone coordinates (SI rad/s)
  float mag[3] = {0}; // magnetometer 
  // constructor
  UAccGyro(UDataBase * datasource);
  /**
   * \return false if message keyword is unknown */
  bool decode(const char * msg, int source, const char * key, bool isCheckedOK) override;
  //
  /**
   * subscribe to source data */
  virtual void subscribeFromHW(bool fromTeensy) override;
  // open logfile for all updates
  void openLog();
  /**
   * Print status for bridge and all data elements */
  void printStatus(int client) override;
  /**
   * get turnrate */
  float turnrate();
  /**
   * send specific help */
  void sendHelp(const int client) override;
  
};


#endif
