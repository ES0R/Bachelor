/**
 * Builds on top of NatNet example, as per the statement below 
 * Modified by DTU jca@elektro.dtu.dk 2021 for use in ASTA Optitrack system
 * 
 * Receive data from Optitrack Motive (v1.5-2.1)
 * by directly reading NatNet (2.5-3.0) UDP stream;
 * works under Ubuntu 14.04 and 16.04 with Python 3 or C++11.
 * This is a direct translation of the official Packet Client example from
 * [NatNet SDK](http://optitrack.com/downloads/developer-tools.html#natnet-sdk),
 * that originally works only on Windows.
 * 
 */


//=============================================================================
// Copyright � 2014 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================


#include <cinttypes>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <cmath>

#include <netdb.h>


//#include "main.h"
#include "uoptidata.h"



/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////

void URigid::printRigid(int client)
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL,"# ID : %d (valid=%d) %s\n", ID, valid, name);
  parent->db->sendReply(s, client);
  snprintf(s, MSL, "# pos: [%3.3f,%3.3f,%3.3f]\n", pos[0], pos[1], pos[2]);
  parent->db->sendReply(s, client);
  snprintf(s, MSL, "# ori: [%3.4f,%3.4f,%3.4f,%3.4f]\n", rotq[0], rotq[1], rotq[2], rotq[3]);
  parent->db->sendReply(s, client);
}

// void URigid::updated()
// {
//   fresh = true;
//   t.now();
//   
// }


void URigid::saveToLog()
{
  if (lf != NULL)
  {
    if (lf->logIsOpen())
    {
      lf->logLock();
      fprintf(lf->logfile, "%ld.%03ld %d %.3f %d %d %.4f %.4f %.4f %.5f %.5f %.5f %.5f %g\n", dataTime.getSec(), dataTime.getMilisec(), 
              frameCnt, frametime,
              ID, valid, pos[0], pos[1], pos[2], rotq[0], rotq[1], rotq[2], rotq[3], posErr 
      );
      lf->doFlush();
      lf->logUnlock();
    }
  }
}

bool URigid::sendToClient(int client, double frametime)
{
  const int MSL = 200;
  char s[MSL];
  toEuler();
  snprintf(s, MSL, "optip %ld.%03ld %d %.3f, %d %d %d %.4f %.4f %.4f %.5f %.5f %.5f %g\n", dataTime.getSec(), dataTime.getMilisec(), frameCnt, frametime,
           ID, totalRigidCnt, valid, pos[2], pos[0], pos[1], euler[0], euler[1], euler[2], posErr);
  bool sendOK = parent->forwardReply(s, client);
  return sendOK;
}

void URigid::clear()
{
  for (int i = 0; i < 3; i++)
    pos[i] = 0;    
  for (int i = 0; i < 4; i++)
    rotq[i] = 0;
  ID = -1;
  strncpy(name, "(not valid)", UOptitrack::MAX_RIGID_NAME_SIZE);
  valid = false; // not tracked
  dataTime.now();
}

void URigid::toEuler()
{
  if (eulerFrame != frameCnt)
  { // convert Quaternion to Euler angles. - from wikipedia https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float w = rotq[0]; // vector rotation
    float x = rotq[3]; // East in ASTA (z in optitrack)
    float y = rotq[1]; // North in ASTA (x in optitrack)
    float z = rotq[2]; // up (y in optitrack)
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);
    float pitch;
    float yaw;
    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
        pitch = std::asin(sinp);      
      // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
    euler[0] = roll;
    euler[1] = pitch;
    euler[2] = yaw;
    eulerFrame = frameCnt;
  }
}


/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////

UOptiData::UOptiData(UDataBase * database, int id)
: UData(database)
{
  sourceID = id;
  addKey("opti");
  dummyMarker = new URigid(this, this); 
  dummyMarker->clear();
}

UOptiData::~UOptiData()
{
  UData::stop();
  closeLog();
  setupLog.closeLog();
}

void UOptiData::printStatus(int client)
{
  const int MSL = 200;
  char s[MSL];
  db->sendReply("# ------ Optitrack interface ------------\n", client);
  sendKeyList(client);
  snprintf(s, MSL, "# connected %d, frame %d\n", connected, frameNumber);
  db->sendReply(s, client);
  snprintf(s, MSL, "# --- %d rigid bodies:\n", markersCnt);
  db->sendReply(s, client);
  for (int i = 0; i < markersCnt; i++)
  {
      markers[i]->printRigid(client);
  }
  snprintf(s, MSL, "# Timestamp     : %3.3f s\n", timestamp);
  db->sendReply(s, client);
  printDataStatus(client);
}
  
URigid * UOptiData::findMarker(int id)
{
    URigid * a = markers[0];
    bool found = false;
    for (int i = 0; i < markersCnt; i++)
    {
        a = markers[i];
        if (a == nullptr)
        {
        break;
        }
        if (a->ID == id)
        {
        found = true;
        break;
        }
    }
    if (not found and (markersCnt < (MAX_MARKERS - 1)))
    {
        printf("# creating object for ID=%d\n", id);
        a = new URigid(this, this);
        markers[markersCnt++] = a;
        a->ID = id;
        // total number needed for message reply
        for (int i=0; i < markersCnt; i++)
          markers[i]->totalRigidCnt = markersCnt;
        // update marker status with list of IDs
        updateOptiData();
    }
    return a;
}

void UOptiData::updateOptiData()
{
  char * p1 = raw[0]->raw;
  int n = 0;
  const int max = UPublish::MAX_MESSAGE_LENGTH;
  snprintf(p1, max, "optis %d %d", connected, markersCnt);
  for (int i =0; i < markersCnt; i++)
  {
    n = strlen(p1);
    p1 += n;
    snprintf(p1, max - n, " %d", markers[i]->ID);
  }
  n = strlen(p1);
  p1 += n;
  snprintf(p1, max - n, "\r\n");
  // mark data as fresh
  raw[0]->updated();
}



void UOptiData::setup(const char* argv0)
{
  UOptitrack::setup(argv0);
  updateOptiData();
}

// URigid * UOptiData::isUpdated(int id)
// {
//   URigid * r = findMarker(id);
//   if (r != nullptr)
//     if (not r->isUpdated())
//       r = nullptr;
//   return r;
// }

void UOptiData::sendHelp(const int client)
{
  db->sendReply("# ------- Optitrack ----------\r\n", client);
  sendKeyList(client);
//   db->sendReply("# keywords: opti    Hold: rigid body tracking data\r\n", client);
  db->sendReply("# sub opti [A [B]]      Subscription connection status\r\n", client);
  db->sendReply("# sub rigid N [A [B]]   Subscription to individual rigid body IDs\r\n", client);
  db->sendReply("#                       N=rigid body ID; A=-1 for continuous, 0=stop; B=update interval in units of 10ms\r\n", client);
}

bool UOptiData::subscribeFromThis(const char* params, const char* key, int client)
{
  bool founda = strcmp(key, "rigid") == 0;
  bool foundb = strcmp(key, "opti") == 0;
  if (founda or foundb)
  { // implement as relevant
    const char * p1 = params;
    if (founda)
    { // rigid subscription
      // transfer to subscription to specific rigid ID
      int id = strtol(p1, (char **)&p1, 10);
      printf("# opti subscribe: id=%d, a=%d, b=%d, client=%d, param=%s", id, founda, foundb, client, p1);
      if (markersCnt > 0)
      {
        for (int i =0; i < markersCnt; i++)
        {
          if (markers[i]->ID == id)
            markers[i]->subscribe(p1, client);
        }
      }
      else
      { // empty marker, just to get some data
        dummyMarker->subscribe(p1, client);
        printf("#    dummy\n");
      }
    }
    else
    { // opti track status subscription
//       printf("# opti subscribe: a=%d, b=%d, client=%d, param=%s\n", founda, foundb, client, p1);
      raw[0]->subscribe(p1, client);
    }
  }
  return founda or foundb;
}

void UOptiData::tick()
{
  if (markersCnt > 0)
  {
    for (int i = 0; i < markersCnt; i++)
      markers[i]->tick();
  }
  else
    dummyMarker->tick();
  // also status
  raw[0]->tick();
}

void UOptiData::openLog()
{
  createLogfile("log_opti_rigid");
  if (logfile != NULL)
  {
    fprintf(logfile, "%% log of all rigid body data from optitrack\n");
    // fprintf(lf->logfile, "%ld.%03ld %d %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", t.getSec(), t.getMilisec(), 
    //        ID, pos[0], pos[1], pos[2], rotq[0], rotq[1], rotq[2], rotq[3] 
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 frameTime (seconds from OptiTrack)\n");
    fprintf(logfile, "%% 3 ID\n");
    fprintf(logfile, "%% 4 valid\n");
    fprintf(logfile, "%% 5 position x (north)\n");
    fprintf(logfile, "%% 6 position y (up)\n");
    fprintf(logfile, "%% 7 position z (east)\n");
    fprintf(logfile, "%% 8 orientation q1 (quertonions (w))\n");
    fprintf(logfile, "%% 9 orientation q2 (x->)\n");
    fprintf(logfile, "%% 10 orientation q3 (y->)\n");
    fprintf(logfile, "%% 11 orientation q4 (z->)\n");
    fprintf(logfile, "%% 12 position error (m)\n");
  }
  setupLog.createLogfile("log_optitrack");
  if (setupLog.logIsOpen())
  {
    fprintf(setupLog.getF(), "%% log of all rigid body data from optitrack\n");
    fprintf(setupLog.getF(), "%% 1 timestamp (sec)\n");
    fprintf(setupLog.getF(), "%% 2 message\n");
  }
}

void UOptiData::frameUpdate(double time, int rigidCnt)
{
  frameTime = time;
  frameCnt++;
  rigidCount = rigidCnt;
  for (int i = 0; i < markersCnt; i++)
  {
    if (markers[i]->fresh and markers[i]->valid)
    { // add common data for all rigid bodies
      markers[i]->frametime = frameTime;
      markers[i]->frameCnt = frameCnt;
      markers[i]->fresh = false;
      // now coordinates is fully upodated
      // mark OK for subscribers
      markers[i]->updated();
      // printf("# UOptiData::frameUpdate: marker fresh and valid %d\n", markers[i]->ID);
      if (logIsOpen())
        markers[i]->saveToLog();
    }
  }
}
