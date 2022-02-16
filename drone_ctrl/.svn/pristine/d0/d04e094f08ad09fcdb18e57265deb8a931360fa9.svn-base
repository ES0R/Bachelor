/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
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
  /** send configuration to USB
   * \param configBuffer set to NULL to use the just saved configuration or to another configuration to fetch.
   * Sends the configuration as byte hex code  */
  void stringConfigToUSB(const uint8_t * configBuffer, int configBufferLength);
  /**
   * is stringbuffer in use, i.e. loaded from a hard-coaded configuration (non-robot specific) */
  bool isStringConfig()
  {
    return stringConfig;
  }
  /** set in use flag and clear buffer */
  inline void setStringBuffer(uint8_t * string2kBuffer,  bool initializeEEprom)
  {
    config = string2kBuffer;
    configAddr = 0;
    configAddrMax = 0;
    if (initializeEEprom)
      eeprom_initialize();
  }
  inline void clearStringBuffer()
  {
    config = NULL;
  }
  /* get 2k config buffer pointer */
//   uint8_t * get2KConfigBuffer()
//   {
//     return config;
//   }
  /**
   * Load configuration from either a config string or from eeProm (flash) 
   * dependent on the "stringConfig" flag
   * \param from2Kbuffer if true, then load from string buffer (must be loaded first), if false, then read for eeprom (flask).
   * */
  void eePromLoadStatus(bool from2Kbuffer);
  /**
   * Save configuration to eeProm (flash) or sent configuration to USB in hex format.
   * \param toUSB set to true to read to USB, or false to save to eeProm */
  void eePromSaveStatus(bool toUSB);
  
public:
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
    if (not stringConfig)
      eeprom_write_word((uint16_t*)adr, v);
    else if (config != NULL)
    {
      memcpy(&config[adr], &v, 2);
    }
    else
      usb_send_str("# failed to save word\n");
    if (adr > configAddr - 2)
      configAddr = adr + 2;
  }
  /**
   * a busy wit if the flash write system is busy */
  inline void busy_wait()
  {
    if (not stringConfig)
    {
      eeprom_busy_wait();
    }
  }
  /** push a block of data to the configuration stack */
  inline void write_block(const char * data, int n)
  {
    if (not stringConfig)
    {
      eeprom_write_block(data, (void*)configAddr, n);
    }
    else
    {
      memcpy(&config[configAddr], data, n);
    }
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
  bool hardConfigLoad(int hardConfigIdx, bool andToUsb);
  
protected:
  /**
   * Get hard coded configuration string */
  int getHardConfigString(uint8_t * buffer, int configIdx);
  
#ifdef TEENSY35
  const int maxEESize = 4096;
#else
  const int maxEESize = 2048;
#endif
// public:
//   /**
//    * use hard-coded string values flag - should be false for configurations related to a specific robot
//    * if false, then the flash-version is maintained while this flag is false. */
//   bool eeFromStringuse;
  
private:
  /** full configuration buffer, as real eeProm 
   * is either NULL or points to a 2048 byte array */
  uint8_t * config;
  /** max number of bytes in string buffer */
//   static const int sbufMaxCnt = 64;
//   /** string buffer for data reply to client */
//   uint8_t sbuf[sbufMaxCnt];
  /** number of bytes written to string buffer */
  int sbufCnt;
  /** is string buffer in use - else flash is in use */
  bool stringConfig;
  /** current read/write adress in config array */
  int configAddr;
  /** highest number used in configuration in config array */
  int configAddrMax;
  /** hard coded configuration 
   * for configuration as of 25 dec 2018, NB! must be changed if configurattion layout changes
   */
  const char * hardConfigBalanceOnSpot2 = 
  "#cfg00:1e 02 00 00 d7 24 00 00 1a 00 05 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c e1 7a 14\
  #cfg01:3e 01 8e 13 cb ff ff ff fe ff ff ff 1c 00 00 00 03 45 00 05 28 00 00 00 00 41 03 00 00 70 41 00\
  #cfg02:00 90 40 00 00 10 41 41 03 00 00 70 41 00 00 90 40 00 00 10 41 1b 03 00 00 80 3f 00 00 80 3e cd\
  #cfg03:cc cc 3d 9a 99 19 3e 0a d7 a3 3c 00 00 00 00 0a d7 23 3c 00 00 00 3f 01 02 00 00 c0 3f 01 03 66\
  #cfg04:66 46 40 cd cc cc 3d 00 02 05 02 0a d7 a3 3d cd cc cc 3e 0a d7 23 3d 29 02 00 00 00 c0 ae 47 61\
  #cfg05:3d 00 00 80 3f 29 5c 8f 3d 00 00 a0 40 1f 03 cd cc 4c 3d 33 33 b3 3e cd cc 4c 3e 66 66 06 40 3d\
  #cfg06:0a d7 3e 6f 12 83 3a 0a d7 23 3c 0a d7 23 3c 9a 99 19 3e cd cc cc 3e 0d 01 66 66 e6 3f 00 00 80\
  #cfg07:3f 00 00 00 40 00 00 00 3f cd cc 4c 3d 00 00 c0 3f 2a 00 6f 31 0a 61 30 2e 32 66 31 6b 30 3a 42\
  #cfg08:3d 31 30 0a 61 2d 30 2e 35 66 31 3a 42 3d 30 2e 31 0a 61 30 66 30 3a 42 3d 31 0a 0e 37 01 00 00\
  #cfg09:71 3d 0a 3e 00 00 b4 42 03 0a 3e 00 00 b4 42 65 34 09 61 6c 66 61 72 6f 6d 65 6f c1 5d 02 34 21\
  #cfg10:b7 20 b8 0b b8 0b 01 06 64 65 76 69 63 65 00 c1 5d 01 00 00 71 3d 0a 3e 00 00 b4 42 05 30 d6 03\
  #cfg11:d8 03 ef 03 ef 03 d8 03 d2 03 e8 03 44 04 e1 03 df 03 ea 03 25 04 d8 03 ae 03 e5 03 0c 04 de 03\
  #cfg12:c9 03 ef 03 0a 04 e3 03 d6 03 ed 03 00 04 dd 03 e1 03 e8 03 c2 03 d9 03 41 04 e9 03 99 03 d4 03\
  #cfg13:d5 03 f0 03 2b 04 d5 03 c5 03 e9 03 1e 04 d8 03 ec 03 e6 03 d4 03 cf 03 01 04 e6 03 03 04 ea 03\
  #cfg14:e3 03 ed 03 fc 03 e6 03 7b 03 ec 03 d1 03 ec 03 0b 04 ee 03 16 04 ea 03 ee 03 eb 03 9a 03 e9 03\
  #cfg15:2c 04 eb 03 ae 03 e9 03 20 04 f1 03 d4 03 ec 03 07 04 e9 03 e4 03 e9 03 14 04 ef 03 d5 03 e9 03\
  #cfg16:d8 03 ef 03 d4 03 e7 03 f7 03 ec 03 d0 03 e8 03 16 04 ea 03 ad 03 e8 03 f6 03 ec 03 df 03"; 
  
  const char * hardConfigBalanceSquare = 
  "#cfg00:3c 02 00 00 d7 24 00 00 1a 00 05 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c e1 7a 14\
  #cfg01:3e 01 8e 13 cb ff ff ff fe ff ff ff 1c 00 00 00 03 45 00 05 28 00 00 00 00 41 03 00 00 70 41 00\
  #cfg02:00 90 40 00 00 10 41 41 03 00 00 70 41 00 00 90 40 00 00 10 41 13 03 00 00 80 3f 00 00 80 3e cd\
  #cfg03:cc cc 3d 00 00 00 00 0a d7 23 3c 00 00 00 3f 01 02 00 00 c0 3f 01 03 66 66 46 40 cd cc cc 3d 00\
  #cfg04:02 05 02 0a d7 a3 3d cd cc cc 3e 0a d7 23 3d 29 02 00 00 00 c0 ae 47 61 3d 00 00 80 3f 29 5c 8f\
  #cfg05:3d 00 00 a0 40 1f 03 cd cc 4c 3d 33 33 b3 3e cd cc 4c 3e 66 66 06 40 3d 0a d7 3e 6f 12 83 3a 0a\
  #cfg06:d7 23 3c 0a d7 23 3c 9a 99 19 3e cd cc cc 3e 0d 01 66 66 e6 3f 00 00 80 3f 00 00 00 40 00 00 00\
  #cfg07:3f cd cc 4c 3d 00 00 c0 3f 50 00 6f 31 0a 61 30 2e 32 66 31 6b 30 3a 42 3d 32 0a 63 30 3a 43 3d\
  #cfg08:30 0a 61 30 2e 32 6b 30 3a 42 3d 33 0a 61 30 2e 34 64 34 30 6a 31 3a 41 3d 30 2e 34 0a 63 30 2e\
  #cfg09:32 3a 43 3d 39 30 0a 6e 31 3a 44 3d 33 0a 61 30 66 31 6b 30 3a 42 3d 35 0a 0e 37 01 00 00 71 3d\
  #cfg10:0a 3e 00 00 b4 42 03 0a 3e 00 00 b4 42 65 34 09 61 6c 66 61 72 6f 6d 65 6f c1 5d 02 34 21 b7 20\
  #cfg11:b8 0b b8 0b 01 06 64 65 76 69 63 65 00 c1 5d 01 00 00 71 3d 0a 3e 00 00 b4 42 05 30 d6 03 d8 03\
  #cfg12:ef 03 ef 03 d8 03 d2 03 e8 03 44 04 e1 03 df 03 ea 03 25 04 d8 03 ae 03 e5 03 0c 04 de 03 c9 03\
  #cfg13:ef 03 0a 04 e3 03 d6 03 ed 03 00 04 dd 03 e1 03 e8 03 c2 03 d9 03 41 04 e9 03 99 03 d4 03 d5 03\
  #cfg14:f0 03 2b 04 d5 03 c5 03 e9 03 1e 04 d8 03 ec 03 e6 03 d4 03 cf 03 01 04 e6 03 03 04 ea 03 e3 03\
  #cfg15:ed 03 fc 03 e6 03 7b 03 ec 03 d1 03 ec 03 0b 04 ee 03 16 04 ea 03 ee 03 eb 03 9a 03 e9 03 2c 04\
  #cfg16:eb 03 ae 03 e9 03 20 04 f1 03 d4 03 ec 03 07 04 e9 03 e4 03 e9 03 14 04 ef 03 d5 03 e9 03 d8 03\
  #cfg17:ef 03 d4 03 e7 03 f7 03 ec 03 d0 03 e8 03 16 04 ea 03 ad 03 e8 03 f6 03 ec 03 df 03";
  const char * hardConfigFollowWall = 
  "#cfg00:0d 02 00 00 d7 24 00 00 1a 00 05 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c e1 7a 14\
  #cfg01:3e 01 8e 13 cb ff ff ff fe ff ff ff 1c 00 00 00 03 45 00 05 28 00 00 00 00 41 03 00 00 70 41 00\
  #cfg02:00 90 40 00 00 10 41 41 03 00 00 70 41 00 00 90 40 00 00 10 41 13 03 00 00 80 3f 00 00 80 3e cd\
  #cfg03:cc cc 3d 00 00 00 00 0a d7 23 3c 00 00 00 3f 01 02 00 00 c0 3f 01 03 66 66 46 40 cd cc cc 3d 00\
  #cfg04:02 05 02 0a d7 a3 3d cd cc cc 3e 0a d7 23 3d 29 02 00 00 00 c0 ae 47 61 3d 00 00 80 3f 29 5c 8f\
  #cfg05:3d 00 00 a0 40 1f 03 cd cc 4c 3d 33 33 b3 3e cd cc 4c 3e 66 66 06 40 3d 0a d7 3e 6f 12 83 3a 0a\
  #cfg06:d7 23 3c 0a d7 23 3c 9a 99 19 3e cd cc cc 3e 0d 01 66 66 e6 3f 00 00 80 3f 00 00 00 40 00 00 00\
  #cfg07:3f cd cc 4c 3d 00 00 c0 3f 21 00 6f 32 0a 61 30 66 31 3a 42 3d 32 0a 61 30 2e 33 35 6c 31 6d 30\
  #cfg08:2e 31 38 3a 42 3d 31 32 30 0a 0e 37 01 00 00 71 3d 0a 3e 00 00 b4 42 03 0a 3e 00 00 b4 42 65 34\
  #cfg09:09 61 6c 66 61 72 6f 6d 65 6f c1 5d 02 34 21 b7 20 b8 0b b8 0b 01 06 64 65 76 69 63 65 00 c1 5d\
  #cfg10:01 00 00 71 3d 0a 3e 00 00 b4 42 05 30 d6 03 d8 03 ef 03 ef 03 d8 03 d2 03 e8 03 44 04 e1 03 df\
  #cfg11:03 ea 03 25 04 d8 03 ae 03 e5 03 0c 04 de 03 c9 03 ef 03 0a 04 e3 03 d6 03 ed 03 00 04 dd 03 e1\
  #cfg12:03 e8 03 c2 03 d9 03 41 04 e9 03 99 03 d4 03 d5 03 f0 03 2b 04 d5 03 c5 03 e9 03 1e 04 d8 03 ec\
  #cfg13:03 e6 03 d4 03 cf 03 01 04 e6 03 03 04 ea 03 e3 03 ed 03 fc 03 e6 03 7b 03 ec 03 d1 03 ec 03 0b\
  #cfg14:04 ee 03 16 04 ea 03 ee 03 eb 03 9a 03 e9 03 2c 04 eb 03 ae 03 e9 03 20 04 f1 03 d4 03 ec 03 07\
  #cfg15:04 e9 03 e4 03 e9 03 14 04 ef 03 d5 03 e9 03 d8 03 ef 03 d4 03 e7 03 f7 03 ec 03 d0 03 e8 03 16\
  #cfg16:04 ea 03 ad 03 e8 03 f6 03 ec 03 df 03";
  const char * hardConfigHighSpeedBalance = 
  "#cfg00:25 02 00 00 01 1e 00 00 0b 00 03 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c 8f c2 75\
  #cfg01:3c 01 8e 13 ec ff ff ff 10 00 00 00 00 00 00 00 03 45 00 05 28 00 00 00 00 41 03 00 00 70 41 00\
  #cfg02:00 90 40 00 00 10 41 41 03 00 00 70 41 00 00 90 40 00 00 10 41 1b 03 00 00 80 3f 00 00 80 3e cd\
  #cfg03:cc cc 3d 9a 99 19 3e 0a d7 a3 3c 00 00 00 00 0a d7 23 3c 00 00 00 3f 01 02 00 00 c0 3f 01 03 66\
  #cfg04:66 46 40 cd cc cc 3d 00 02 05 02 0a d7 a3 3d cd cc cc 3e 0a d7 23 3d 29 02 00 00 00 c0 ae 47 61\
  #cfg05:3d 00 00 80 3f 29 5c 8f 3d 00 00 a0 40 1f 03 cd cc 4c 3d 33 33 b3 3e cd cc 4c 3e 66 66 06 40 3d\
  #cfg06:0a d7 3e 6f 12 83 3a 0a d7 23 3c 0a d7 23 3c 9a 99 19 3e cd cc cc 3e 0d 01 66 66 e6 3f 00 00 80\
  #cfg07:3f 00 00 00 40 00 00 00 3f cd cc 4c 3d 00 00 c0 3f 31 00 6f 31 0a 61 30 66 31 3a 42 3d 32 0a 61\
  #cfg08:31 2e 34 64 34 30 6b 32 6a 31 3a 42 3d 36 0a 61 31 2e 34 6b 2d 32 3a 42 3d 36 0a 6e 31 3a 44 3d\
  #cfg09:31 0a 0e 37 e4 02 fc 0b 03 04 82 0d 70 04 a7 0d cc 04 c0 0d db 04 cb 0d 9b 04 c2 0d 2b 04 92 0d\
  #cfg10:04 03 f6 0b 02 e4 0c d6 0b bc 02 20 03 01 06 64 65 76 69 63 65 00 c1 5d 01 00 00 71 3d 0a 3e 00\
  #cfg11:00 b4 42 07 30 24 04 f3 03 9b 03 02 04 f5 03 f0 03 e0 03 00 04 c9 03 ec 03 36 04 06 04 cd 03 ec\
  #cfg12:03 c8 03 fc 03 d2 03 ec 03 02 04 04 04 9a 03 eb 03 e5 03 fd 03 ad 03 e7 03 2a 04 fe 03 a7 03 e0\
  #cfg13:03 06 04 f6 03 8f 03 e4 03 fc 03 01 04 a7 03 e8 03 fb 03 00 04 bb 03 eb 03 09 04 01 04 ba 03 e6\
  #cfg14:03 c1 03 fb 03 e6 03 ff 03 bd 03 f7 03 d4 03 fe 03 b1 03 f9 03 46 04 fc 03 b4 03 ef 03 16 04 f5\
  #cfg15:03 8d 03 ee 03 07 04 f3 03 a9 03 e8 03 fd 03 f5 03 ca 03 ee 03 f3 03 f8 03 e0 03 ec 03 b2 03 f4\
  #cfg16:03 44 04 f2 03 8e 03 f4 03 d1 03 f0 03 38 04 f4 03 c3 03 f0 03 08 04 fb 03 d0 03 ee 03 b7 03 fc\
  #cfg17:03 01 04 f5 03";
  static const int hardConfigCnt = 4;
  const char * hardConfig[hardConfigCnt];
};

/**
 * Instans af ee og string config */
extern EEConfig eeConfig;


#endif
