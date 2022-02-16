/***************************************************************************
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * read and save configuration as string
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

#ifndef REGBOT_EESTRING_H
#define REGBOT_EESTRING_H

#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "command.h"

class EEConfig
{
public:
  // constructor
  EEConfig();
  /**
   * Send command help */
  void sendHelp();
  /**
   * decode commands */
  bool decode(const char * buf);
  /**
   * Load configuration from eeProm (flash) 
   * \returns true if variables read count matches the ones saved,
   * if not, then configuration has changed and system
   * should be rebooted (flash set as invalid)
   * NB! No influence on magnetometer calibration
   * */
  bool eePromLoadStatus();
  /**
   * Save configuration to eeProm (flash).
   *  */
  void eePromSaveStatus();
  //
public:
  // global support functions
  /** save a 32 bit value */
  void push32(uint32_t value);
  /** save a byte */
  void pushByte(uint8_t value);
  /** save a word in configuration stack */
  void pushWord(uint16_t value);
  /** get a 32 bit integer from configuration stack */
  uint32_t read32();
  /** get a byte from configuration stack */
  uint8_t readByte();
  /** get a 16 bit integer from configuration stack */
  uint16_t readWord();
  /**
   * Add a block of data to ee-Prom area 
   * \param data is the byte data block,
   * \param dataCnt is the number of bytes to write 
   * \returns true if space to write all. */
  bool pushBlock(const char * data, int dataCnt);
  /**
   * Read a number of bytes to a string 
   * \param data is a pointer to a byte array with space for at least dataCnt bytes.
   * \param dataCnt is number of bytes to read
   * \returns true if data is added to data array and false if 
   * requested number of bytes is not available */
  bool readBlock(char * data, int dataCnt);
  
  /** save a 32 bit float to configuration stack */
  inline void pushFloat(float value)
  {
    union {float f; uint32_t u32;} u;
    u.f = value;
    push32(u.u32);
  }
  // read 32 bit as float from configuration stack
  inline float readFloat()
  {
    union {float f; uint32_t u32;} u;
    u.u32 = read32();
    return u.f;  
  }
  /** write a word to a specific place in configuration stack
   * typically a size that is not known before pushing all the data */
  inline void write_word(int adr, uint16_t v)
  {
//     if (not stringConfig)
      eeprom_write_word((uint16_t*)adr, v);
//     else if (config != NULL)
//     {
//       memcpy(&config[adr], &v, 2);
//     }
//     else
      command->usb_send_str("# failed to save word\n");
    if (adr > configAddr - 2)
      configAddr = adr + 2;
  }
  /**
   * a busy wit if the flash write system is busy */
  inline void busy_wait()
  {
//     if (not stringConfig)
    {
      eeprom_busy_wait();
    }
  }
  /** push a block of data to the configuration stack */
  inline void write_block(const char * data, int n)
  {
//     if (not stringConfig)
//     {
      eeprom_write_block(data, (void*)configAddr, n);
//     }
//     else
//     {
//       memcpy(&config[configAddr], data, n);
//     }
    configAddr += n;
  }
  /** set the adress for the next push or read operation on the configuration stack */
  void setAddr(int newAddr)
  {
    configAddr = newAddr;
  }
  /** skip some bytes from the configuration stack
   * \param bytes is the number of bytes to skib. */
  void skipAddr(int bytes)
  {
    configAddr+=bytes;
    // debug
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "# skipped %d bytes\n", bytes);
//     usb_send_str(s);
    // debug end
  }
  /** get the address of the next push or read operation on the configuration stack */
  int getAddr()
  {
    return configAddr;
  }
  /**
   * Implement one of the hard-coded configurations 
   * \param hardConfigIdx is index to the hardConfig array, as defined in eeconfig.h and set in the constructor.
   * \param andToUsb is a debug flag, that also will return the just loaded configuration to the USB
   * */
//   bool hardConfigLoad(int hardConfigIdx, bool andToUsb);
  
protected:
  /**
   * Get hard coded configuration string */
//   int getHardConfigString(uint8_t * buffer, int configIdx);
  
// #ifdef TEENSY35
//   const int maxEESize = 4096;
// #else // Teensy 3.2
//   const int maxEESize = 2048;
// #endif
  /**
   * Prop shield has own EE-config for magnetometer calibration
   * starting at 60 and of size 68 bytes */
  static const int EE_CONFIG_ADDR_START = 130;
  
private:
  /** current read/write adress in config array (flash) */
  int configAddr;
  /** highest number used in configuration in config array */
//   int configAddrMax;
};



#endif
