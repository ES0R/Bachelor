/***************************************************************************
 *   Copyright (C) 2016-2021 by DTU (Christian Andersen)                        *
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

#ifndef UUNHANDLED_H_INCLUDED
#define UUNHANDLED_H_INCLUDED

#include <udata.h>

class UDataBase;
/**
 * class for any additional messages from Teensy. these may be requested by a client (e.g. the GUI) */
class UUnhandled : public UData
{
public:
  /// constructor
  UUnhandled(UDataBase * data);
  /** handle message
   * return true if used */
  bool decodeUnhandled(const char * msg, int source);
  // open logfile
  void openLog();
//   void saveToLog();
  /**
   * Print status for bridge and all data elements */
  void printStatus(int client) override;
  /**
   * send on-line help */
  void sendHelp(const int client) override;
};

#endif
