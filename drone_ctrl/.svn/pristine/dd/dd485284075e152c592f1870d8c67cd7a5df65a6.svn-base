/***************************************************************************
 *   Copyright (C) 2019 by DTU (Christian Andersen)                        *
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

#include <string.h>
#include <udatabase.h>
#include "ustate.h"

UState::UState(UDataBase * data)
: UData(data)
{
  gettimeofday(&bootTime, NULL);
  // default
  strncpy(robotname, "no name", MAX_NAME_LENGTH);
  gettimeofday(&lastHbtTime, NULL);
  gettimeofday(&lastFakeHbtTime, NULL);
  // add supported keys
  addKey("hbt");
  addKey("dname");
}

bool UState::decode(const char* message, int source, const char * key, bool isCheckedOK)
{
  bool used = true;
  if (strcmp(key, "hbt") == 0)
    decodeHbt(message); // it is a heartbeat message (time and battry voltage)
  else if (strcmp(key, "dname") == 0)
    decodeId(message); // mission status skipped
  else if (strcmp(key, "temp") == 0)
  {
    float pit = measureCPUtemp();
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "tmppi %g\r\n", pit);
    db->sendReply(s, source);
  }
  else 
    used = false;
  if (used)
  {
    updated(message, source, key);
  }
  return used;
}


void UState::openLog()
{ // open pose log
  UData::createLogfile("log_hbt_mission");
  if (logIsOpen())
  {
    fprintf(logfile, "%% robobot mission heartbeat logfile\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 REGBOT time\n");
    fprintf(logfile, "%% 3 battery voltage [V]\n");
    fprintf(logfile, "%% 4 armState (Init=0, Disarmed=1, Armed=2, Fail=3)\n");
    fprintf(logfile, "%% 5 flight state (OnGround, Starting, InFlight, Landing)\n");
    fprintf(logfile, "%% 6 Raspberry CPU temperature (C)\n");
  }
}


void UState::decodeHbt(const char * msg)
{ //   snprintf(reply, MRL,  "hbt %.4f %d %d %.2f %d %d %d\r\n",  time, deviceID,
  //   command->getRevisionNumber()/10, sensor->batteryVoltage, armState, flightState, bypassRC);
  const char * p1 = &msg[3];
  regbotTime = strtof(p1, (char **)&p1);
  robotId = strtol(p1, (char **)&p1, 10);
  robotSWversion = strtol(p1, (char **)&p1, 10);
  batteryVoltage = strtof(p1, (char**)&p1);
  armState = (ArmState)strtof(p1, (char**)&p1);
  flightState = (FlightState)strtof(p1, (char**)&p1);
  timeval t;
  gettimeofday(&t, NULL);
  float dt = getTimeDiff(t, lastHbtTime);
  if (dt < 36000 and dt > 2)
  {
    printf("# heartbeat time too slow (%.3fsec) - lost Teensy connection?\n", dt);
    lastHbtOnTime = false;
  }
  else
  {
    if (not lastHbtOnTime)
      printf("# heartbeat back on time (%.3fsec) - Teensy now online\n", dt);
    lastHbtOnTime = true;
  }
  lastHbtTime = t;
  saveDataToLog();
}

void UState::saveDataToLog()
{
  if (logfile != NULL)
  {
    timeval t;
    gettimeofday(&t, NULL);
    fprintf(logfile, "%ld.%03ld %.3f %.1f %d %d %.1f\n", t.tv_sec, t.tv_usec / 1000, 
            regbotTime,
            batteryVoltage,
            armState,
            flightState,
            measureCPUtemp()
    );
    fflush(logfile);
  }
}

bool UState::isHeartbeatOK()
{
  timeval t;
  gettimeofday(&t, NULL);
  float dt = getTimeDiff(t, lastHbtTime);
  bool isOK = dt < 2.0;
  //  printf("# heartbeat time not OK (%.3fsec)\n", dt);
  return isOK;
}

void UState::decodeId(const char * msg)
{ // 
  const char * p1 = &msg[3];
  while (isblank(*p1))
    p1++;
  const char * p2 = p1;
  while (isalnum(*p2))
    p2++;
  int chars = p2-p1;
  if (chars > MAX_NAME_LENGTH)
    chars = MAX_NAME_LENGTH - 1;
  strncpy(robotname, p1, chars);
  robotname[chars] = '\0'; // terminate string
}


float UState::getTime()
{
  timeval t;
  gettimeofday(&t, NULL);
  return getTimeDiff(t, bootTime);
}

void UState::subscribeFromHW(bool fromTeensy)
{
  if (fromTeensy)
  {
    db->sendToDataSource("robot id\n", UDataBase::Teensy); // request ID values from robot (once should be enough)
    db->sendToDataSource("robot sub state 100\n", UDataBase::Teensy); // request regular hbt data
  }
  else
  {
    db->sendToDataSource("mis subscribe 2\n", UDataBase::Bridge); // mission data
    db->sendToDataSource("hbt subscribe 1\n", UDataBase::Bridge); // time and mission info  
    db->sendToDataSource("rid subscribe 3\n", UDataBase::Bridge); // get robot ID values
  }
}


void UState::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];

  db->sendReply("# ------- State ----------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# %s (%d), FW=%d\n", robotname, robotId, robotFWversion);
  db->sendReply(s, client);
  snprintf(s, MSL, "# REGBOT time %.3fs, batteryVoltage %.1f\n", regbotTime, batteryVoltage);
  db->sendReply(s, client);
  snprintf(s, MSL, "# Arm %d, flight %d\n", armState, flightState);
  db->sendReply(s, client);
  //printf("# data age %.3fs\n", getTimeSinceUpdate());
  printDataStatus(client);
}

void UState::tick()
{
  timeval t;
  gettimeofday(&t, NULL);
  float dt = getTimeDiff(t, lastHbtTime);
  if (dt > 2)
  { // time to make a fake hbt
    dt = getTimeDiff(t, lastFakeHbtTime);
    if (dt > 1.5)
    { // update time
      lastFakeHbtTime = t;
      const int MSL = 100;
      char s[MSL];
      // hbt 9267.6582 1 1240 11.92 1 0 0
      // hbt time id, hw, V, state, flight, bypass, fake
      double dt2 = getTimeDiff(t, bootTime);
      snprintf(s, MSL, "hbt %.03f %d %d %.2f %d %d %d 0 fake\n", dt2, robotId, robotSWversion, batteryVoltage, armState, flightState, 0);
      // make it a fake update
      updated(s, dataID, "hbt");
      // printf("# UState::tick: made fake hbt\n");
    }
  }
  UData::tick();
}

float UState::measureCPUtemp()
{
  FILE * temp;
  const char * tf = "/sys/class/thermal/thermal_zone0/temp";
  temp = fopen(tf, "r");
  float t = 0;
  if (temp != NULL)
  {
    const int MSL = 20;
    char s[MSL];
    char * p1 = s;
    int n = fread(p1, 1, 1, temp);
    int m = n;
    while (n > 0)
    {
      n = fread(++p1, 1, 1, temp);
      m += n;
    }
    s[m] = '\0';
    if (m > 3)
    {
      t = strtof(s, &p1);
    }
    //     printf("#got %d bytes (%g deg) as:%s\n", m, t/1000.0, s); 
    fclose(temp);
  }
  return t/1000.0;
}

void UState::sendHelp(const int client)
{
  db->sendReply("# ------- State ----------\r\n", client);
  sendKeyList(client);
}
