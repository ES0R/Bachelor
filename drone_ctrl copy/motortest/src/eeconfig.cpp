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

#include "main.h"
#include "eeconfig.h"
//#include "avr_functions.h"
#include "uesc.h"
/**
 * Global configuration */
EEConfig eeConfig;


/** initialize */
EEConfig::EEConfig()
{
  sbufCnt = 0;
  stringConfig = false;
  config = NULL;
  hardConfig[0] = hardConfigBalanceOnSpot2;
  hardConfig[1] = hardConfigBalanceSquare;
  hardConfig[2] = hardConfigFollowWall;
  hardConfig[3] = hardConfigHighSpeedBalance;
}

void EEConfig::stringConfigToUSB(const uint8_t * configBuffer, int configBufferLength)
{
  int length = configBufferLength;
  const uint8_t * cfg = configBuffer;
  if (cfg == NULL)
  {
    cfg = config;
    length = configAddrMax;
  }
  if (cfg == NULL)
  {
    usb_send_str("# error: configuration not generated as string\n");
  }
  else
  {
    const int MSL = 110;
    char s[MSL];
    char * p1 = s;
    int n = 0;
    int line = 0;
    int i = 0;
    while (i < length)
    {
      snprintf(s, MSL, "#cfg%02d:", line++);
      n += strlen(p1);
      p1 = &s[n];
      for (int j = 0; j < 32; j++)
      {
        snprintf(p1, MSL-n, "%02x ", cfg[i]);
        n += strlen(p1);
        p1 = &s[n];
        i++;
        if (i >= length)
          break;
      }
      p1--; // skip last space
      if (i < length)
        // not finished, so add an (linefeed) escape character
        *p1++ = '\\';
      *p1++ = '\n';
      *p1++ = '\0';
      usb_send_str(s);
      if (n > MSL - 4)
        usb_send_str("# stringConfigToUSB error\n");
      p1 = s;
      n = 0;
    }
  }
}
  
void EEConfig::push32(uint32_t value)
{
  //   const int MSL = 100;
  //   char s[MSL];
  //   snprintf(s, MSL, "# ee saved: at %lu, value %lu\r\n", eePushAdr, value);
  //   usb_send_str(s);
  //
  if (stringConfig)
  {
    if (config != NULL)
      memcpy(&config[configAddr], &value, 4);
  }
  else
  {
    eeprom_busy_wait();
    eeprom_write_dword((uint32_t*)configAddr, value);
  }
  configAddr += 4;
  if (configAddr > configAddrMax)
    configAddrMax = configAddr;
}

////////////////////////////////////////////////

void EEConfig::pushByte(uint8_t value)
{ // save one byte
  if (stringConfig)
  {
    if (config != NULL)
      config[configAddr] = value;
  }
  else
  {
    eeprom_busy_wait();
    eeprom_write_byte((uint8_t*)configAddr, value);
  }
  configAddr++;
  if (configAddr > configAddrMax)
    configAddrMax = configAddr;
}

////////////////////////////////////////////////

void EEConfig::pushWord(uint16_t value)
{ // save one byte
  if (stringConfig)
  {
    if (config != NULL)
      memcpy(&config[configAddr], &value, 4);
  }
  else
  {
    eeprom_busy_wait();
    eeprom_write_word((uint16_t*)configAddr, value);
  }
  configAddr += 2;
  if (configAddr > configAddrMax)
    configAddrMax = configAddr;
}

//////////////////////////////////////////////

uint32_t EEConfig::read32()
{
  uint32_t b;
  if (stringConfig)
  {
    if (config != NULL)
      b = *(uint32_t *)&config[configAddr];
    else
      b = 0;
  }
  else
  {
    b = eeprom_read_dword((uint32_t*)configAddr);
  }
  configAddr += 4;
  return b;
}

/////////////////////////////////////////////////

uint8_t EEConfig::readByte()
{
  uint8_t b;
  if (stringConfig)
  {
    if (config != NULL)
      b = config[configAddr];
    else
      b = 0;
  }
  else
  {
    b = eeprom_read_byte((uint8_t*)configAddr);
  }
//   { // debug
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "# read byte %d (%c) at address %d (%x), max=%d\n", b, b, configAddr, configAddr, configAddrMax);
//     usb_send_str(s);
//   }
  configAddr++;
  return b;
}

/////////////////////////////////////////////////

uint16_t EEConfig::readWord()
{
  uint16_t b;
  if (stringConfig)
  {
    if (config != NULL)
      b = *(uint16_t *)&config[configAddr];
    else
      b = 0;
  }
  else
  {
    b = eeprom_read_word((uint16_t*)configAddr);
  }
  configAddr += 2;
  return b;
}
  
///////////////////////////////////////////////////

void EEConfig::eePromSaveStatus(bool toUSB)
{ // reserve first 4 bytes for dword count
  const int MSL = 100;
  char s[MSL];
  // debug
  // debug end
  stringConfig = toUSB;
  // save space for used bytes in configuration
  configAddr = 4;
  configAddrMax = 4;
  // save revision number
  push32(getRevisionNumber());
  // save servo configuration
  esc->eePromSave();
  // then save length
  uint32_t cnt = configAddr;
  configAddr = 0;
  if (robotId <= 0)
  {
    // ignore ee-prom at next reboot
    push32(0);
    snprintf(s, MSL, "# EE-prom D set to default values at next reboot\r\n");
  }
  else
  {
    push32(cnt);
    if (toUSB)
      snprintf(s, MSL, "# Send %lu config bytes (of %d) to USB\r\n", cnt, EEPROM_SIZE);
    else
      snprintf(s, MSL, "# Saved %lu bytes (of %d) to EE-prom D\r\n", cnt, EEPROM_SIZE);
  }
  configAddr = cnt;
  // tell user
  usb_send_str(s);
}

//////////////////////////////////////////////////

void EEConfig::eePromLoadStatus(bool from2Kbuffer)
{ 
  const int MSL = 100;
  char s[MSL]; 
  //eePushAdr = 0;
  stringConfig = from2Kbuffer;  
  configAddr = 0;
  uint32_t cnt = read32();
  uint32_t rev = read32();
  snprintf(s, MSL, "# Reading configuration - in flash cnt=%lu, rev=%lu, this is rev=%d\r\n", cnt, rev, getRevisionNumber());
  usb_send_str(s);
  if (cnt == 0 or cnt >= uint32_t(maxEESize) or rev == 0)
  {
    snprintf(s, MSL, "# No saved configuration - save a configuration first (cnt=%lu, rev=%lu)\r\n", cnt, rev);
    usb_send_str(s);
    return;
  }
  if (rev != getRevisionNumber())
  {
    snprintf(s, MSL, "# configuration from old SW version now:%g != ee:%g - continues\r\n", getRevisionNumber()/100.0, rev/100.0);
    usb_send_str(s);
  }
  // load servo settings (mostly steering parameters)
  esc->initEsc();
  esc->eePromLoad();
  // note changes in ee-prom size
  if (cnt != (uint32_t)configAddr)
  {
    snprintf(s, MSL, "# configuration size has changed! %lu != %d bytes\r\n", cnt, configAddr);
    usb_send_str(s);
  }
}
  
  /////////////////////////////////////////////////
  
  
int EEConfig::getHardConfigString(uint8_t* buffer, int configIdx)
{
  int n = 0;
  int line = 0;
  const int MSL = 60;
  char s[MSL];
  if (buffer == NULL or configIdx < 0 or configIdx >= hardConfigCnt)
  {
    usb_send_str("# error in get config from string, no buffer ir index out of range\n");
  }
  else
  { // buffer string assumed to be 2kbyte
    const char * p1 = hardConfig[configIdx];
    int nl = strlen(p1);
    while (p1 != NULL and n < 2048 and (p1 - hardConfig[configIdx]) < nl)
    {
      if (*p1 != '#')
      {
        snprintf(s, MSL, "# error in hard config found '%c' at n=%d\n", *p1, n); 
        usb_send_str(s);
        p1++;
      }
      else
      { // skip "cfg" - assumed to be OK
        p1 += 4;
        line = strtol(p1, (char**)&p1, 10);
        if (line > 64)
        { // more than 2kB configuration string
          snprintf(s, MSL, "#too many hard lines: %d (n=%d)\n", line, n);
          usb_send_str(s);
          break;
        }
        // read data
        p1++;
        for (int i = 0; i < 32; i++)
        {
          buffer[n] = strtol(p1, (char**)&p1, 16);
          n++;
          if (*p1 == '\0' or (p1 - hardConfig[configIdx]) > nl)
            break;
        }
        while (*p1 == ' ')
          p1++;
        if (n >= 2048)
          break;
      }
    }
    snprintf(s, MSL, "# loaded %d byte sized values\n", n); 
    usb_send_str(s);
  }
  return n;
}
  
//////////////////////////////////////////////////////

bool EEConfig::hardConfigLoad(int hardConfigIdx, bool andToUsb)
{
  bool isOK = false;
  uint8_t buffer2k[2048];
  // set stringConfig flag and set 2k buffer pointer
  eeConfig.setStringBuffer(buffer2k, false);
  // convert hard coded string configuration to 2k buffer
  // returns number of used values in the 2k buffer
  int n = eeConfig.getHardConfigString(buffer2k, hardConfigIdx);
  if (n > 100)
  { // string config is now in buffer2k, ready to be used
    // and set flag to use this rather than the real 2k flash
    eeConfig.eePromLoadStatus(true);
    // debug
    if (andToUsb)
      eeConfig.stringConfigToUSB(buffer2k, n);
    // debug end
    isOK = true;
  }
  else
    usb_send_str("#config string too short\n");
  // clear stringConfig flag - and reset 2k buffer pointer
  eeConfig.clearStringBuffer();
  return isOK;
}


bool EEConfig::pushBlock(const char * data, int dataCnt)
{
  if (getAddr() + dataCnt < 2048 - 2)
  {
    busy_wait();
    write_block(data, dataCnt);
    return true;
  }
  else
    return false;
}

bool EEConfig::readBlock(char * data, int dataCnt)
{
  if (getAddr() + dataCnt < 2048 - 2)
  {
    busy_wait();
    for (int n = 0; n < dataCnt; n++)
    {
      data[n] = readByte();
    }
    return true;
  }
  else
    return false;
}
