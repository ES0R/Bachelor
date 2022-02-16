 /***************************************************************************
 *   Copyright (C) 2016-2020 by DTU (Christian Andersen)                        *
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

#ifndef UMISSION_H
#define UMISSION_H


#include <sys/time.h>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "urun.h"
#include "ucamera.h"
#include "ubridge.h"
#include "ujoy.h"
#include "uplay.h"
#include <udatabase.h>


class UDataBase;
class UBridge;
class UCamera;

/**
 * Base class, that makes it easier to starta thread
 * from the method runObj
 * The run method should be overwritten in the real class */
class UMission : public UData
{
public:
  /// flag to finish all (pending) missions and RC
  bool finished = false;
private:
  /**
   * Pointer to communication and data part of this mission application */
  UBridge * bridge;
  UDataBase * db;
  /**
   * Pointer to camera (class) of this mission application */
  UCamera * cam;
  /** is thread active */
  bool active = false;
  // thread on Regbot
  int threadActive = false;
  /// space for fabricated lines
  const static int MAX_LINES = 100;
  const static int MAX_LEN = 100;
  const static int missionLineMax = 20;
  /** definition of array with c-strings for the mission snippets */
  char lineBuffer[missionLineMax][MAX_LEN];
  /** an array of pointers to mission lines */
  char * lines[missionLineMax];
  /** logfile for mission state */
  FILE * logMission = NULL;
  
public:
  /**
   * Constructor */
  UMission(UDataBase * db);
  /**
   * Destructor */
  ~UMission();
  /**
   * open logfile and write prolog */
  void openLog();
  /**
   * Stop all missions */
  void stop()
  {
    finished = true;
    th1stop = true;
    usleep(11000);
    if (th1 != NULL)
      th1->join();
  }
  /**
   * Print status for mission */
  void printStatus(int client) override;
  /**
   * time to update mission state */
  void tick() override;
  
  
private:
  /**
   * Object to play a soundfile as we go */
  USay play;
  /**
   * save mission state to log */
  void saveDataToLog();
  /**
   * mission states */
  int mission;
  int missionState;
  
};


#endif
