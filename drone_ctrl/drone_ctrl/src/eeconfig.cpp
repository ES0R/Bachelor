/***************************************************************************
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * read and save configuration as string to/from uP flash
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
#include "uesc.h"
#include "mixer.h"
#include "upropshield.h"
#include "control.h"
#include "ustate.h"
#include "uled.h"
#include "uheight.h"
#include "ultrasound.h"
#include "sensor.h"


/** initialize */
EEConfig::EEConfig()
{
}

void EEConfig::sendHelp()
{
  const int MRL = 250;
  char reply[MRL];
  command->usb_send_str("# ---- EE (configuration flash) --------\r\n");
  snprintf(reply, MRL, "#   eew          save configuration to EE-Prom (flash)\r\n");
  command->usb_send_str(reply);
  snprintf(reply, MRL, "#   eer          Read configuration from EE-Prom\r\n");
  command->usb_send_str(reply);
}


bool EEConfig::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "eer", 3) == 0)
  {  // load from flash to ccurrent configuration
    eePromLoadStatus();
  }
  else if (strncmp(buf, "eew", 3) == 0)
  {  // load from flash to ccurrent configuration
    eePromSaveStatus();
  }
  else
    used = false;
  return used;
}

  
void EEConfig::push32(uint32_t value)
{
  eeprom_busy_wait();
  if (configAddr < EEPROM.length())
    eeprom_write_dword((uint32_t*)configAddr, value);
  configAddr += 4;
}

////////////////////////////////////////////////

void EEConfig::pushByte(uint8_t value)
{ // save one byte
  eeprom_busy_wait();
  if (configAddr < EEPROM.length())
    eeprom_write_byte((uint8_t*)configAddr, value);
  configAddr++;
}

////////////////////////////////////////////////

void EEConfig::pushWord(uint16_t value)
{ // save one byte
  eeprom_busy_wait();
  if (configAddr < EEPROM.length())
    eeprom_write_word((uint16_t*)configAddr, value);
  configAddr += 2;
}

//////////////////////////////////////////////

uint32_t EEConfig::read32()
{
  uint32_t b;
  b = eeprom_read_dword((uint32_t*)configAddr);
  configAddr += 4;
  return b;
}

/////////////////////////////////////////////////

uint8_t EEConfig::readByte()
{
  uint8_t b;
  b = eeprom_read_byte((uint8_t*)configAddr);
  configAddr++;
  return b;
}

/////////////////////////////////////////////////

uint16_t EEConfig::readWord()
{
  uint16_t b;
  b = eeprom_read_word((uint16_t*)configAddr);
  configAddr += 2;
  return b;
}
  
///////////////////////////////////////////////////

void EEConfig::eePromSaveStatus()
{ // reserve first 4 bytes for dword count
  const int MSL = 200;
  char s[MSL];
  uint32_t cnt;
  // save space for used bytes in configuration
  configAddr = EE_CONFIG_ADDR_START + 4;
  // save revision number
  push32(command->getRevisionNumber());
  // save all configuration
  esc->eePromSave();
  command->usb_send_str("# saving IMU offset\n");
  imu->eePromSave();
  command->usb_send_str("# saving Control\n");
  control->eePromSaveCtrl();
  rc->eePromSave();
  leds->eePromSave();
  state->eePromSave();
  hgt->eePromSave();
  uhgt->eePromSave();
  mixer->eePromSave();
  sensor->eePromSave();
  // then save length
  cnt = configAddr;
  configAddr = EE_CONFIG_ADDR_START;
  if (state->deviceID <= 0)
  { // ignore ee-prom at next reboot
    push32(0);
    snprintf(s, MSL, "# EE-prom D set to default values at next reboot\r\n");
  }
  else
  {
    push32(cnt - EE_CONFIG_ADDR_START);
    snprintf(s, MSL, "# Saved %lu bytes (of %d) to EE-prom D\r\n", cnt-EE_CONFIG_ADDR_START, EEPROM.length()); //EEPROM_SIZE);
  }
  configAddr = cnt;
  // tell user
  command->usb_send_str(s);
}

//////////////////////////////////////////////////

bool EEConfig::eePromLoadStatus()
{ 
  const int MSL = 200;
  char s[MSL]; 
  //eePushAdr = 0;
//  stringConfig = from2Kbuffer;  
  bool isOK = true;
  configAddr = EE_CONFIG_ADDR_START;
  uint32_t cnt = read32();
  uint32_t rev = read32();
  snprintf(s, MSL, "# Reading configuration - in flash cnt=%lu, rev=%lu, this is rev=%d\r\n", cnt, rev, command->getRevisionNumber());
  command->usb_send_str(s);
  if (cnt == 0 or cnt >= uint32_t(EEPROM.length()) or rev == 0)
  {
    snprintf(s, MSL, "# No saved configuration - save a configuration first (cnt=%lu, rev=%lu)\r\n", cnt, rev);
    command->usb_send_str(s);
    // loaded nothing just fine
    return true;
  }
  if (rev != command->getRevisionNumber())
  {
    snprintf(s, MSL, "# configuration from old SW version now:%g != ee:%g - continues\r\n", command->getRevisionNumber()/10.0, rev/10.0);
    command->usb_send_str(s);
  }
  // load settings for these modules
  esc->eePromLoad();
  imu->eePromLoad();
  control->eePromLoadCtrl();
  rc->eePromLoad();
  leds->eePromLoad();
  state->eePromLoad();
  hgt->eePromLoad();
  uhgt->eePromLoad();
  mixer->eePromLoad();
  sensor->eePromLoad();
  // note changes in ee-prom size
  if (cnt != (uint32_t)configAddr - EE_CONFIG_ADDR_START)
  { // mark configuration as invalid
    configAddr = EE_CONFIG_ADDR_START;
    push32(0);
    isOK = false;
    snprintf(s, MSL, "# configuration size has changed! %lu != %d bytes (data may be corrupted)\r\n", cnt, configAddr);
    command->usb_send_str(s);
  }
  return isOK;
}
  
  /////////////////////////////////////////////////

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
