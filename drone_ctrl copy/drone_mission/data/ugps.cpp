/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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

#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/serial.h>

#include <utime.h>
#include "ugps.h"


int setDeviceSpeed(const char * devName, int devSpeed);

bool latlon2UTM(double latDeg, double longDeg, int zone, double * easting, double *  northing);
bool utm2latlon(double easting, double northing, int zone,
                double * latitude, double * longitude);
void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
             double * UTMNorthing, double * UTMEasting, int zone);
void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting,
             int zone /*const char* UTMZone*/, double * Lat,  double * Long );

/////////////////////////////////////////////

void UGpsStatus::clear()
{
  int i;

   //Internal status struct
  opr_mode = '\0'; //Operational mode
  mode = 0;  //Calculation mode
  for(i = 0; i < GPS_SATELLITES_SUPPORTED; i++)
    satUsed[i] = 0;//Satellites used to make calculation
  for(i = 0; i < GPS_SATELLITES_TOTAL; i++)
  {
    satVisGID[i] = 0; //Number
    satVisElev[i] = 0; //Elevation
    satVisAz[i] = 0; //Azimuth
    satVisSN[i] = 0; //Signal to Noise Ratio
  }
  PDOP = 0.0;  //Positional Dillution of Precision
  HDOP = 0.0;  //Horizontal Dillution of Precision
  VDOP = 0.0;  //Vertical Dillution of precision
  satVisCnt = 0;//Satellites in view
  satVisIdx = 0;
  satUsedCnt = 0;
  // FOM = 0.0; //Precision of the soultion
   //These variables are not updated every second or every 5 seconds.
  EGNOS = 0; //SBAS (EGNOS/WAAS) augmentation indicator
  //zone = 0;  //UTM zone used to specify the base
             //for the coordinate system
}

/////////////////////////////////////////////////

bool UGpsStatus::parseGPGSA(char * inBuf)
{ //   $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
  bool isOK = true;
  char *p1, *p2;
  int param = 0, sat;
  //
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  sat = 0;
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  lock.lock();
  while (*p2 >= ' ')
  { // separate into substring
    p1 = strsep(&p2, ",");
    switch (param)
    {
      case 0: // type, $GPGSA or $GNGSA
      break;
    case 1:  //GPS operation mode "A" Automatic "M" manual
      opr_mode = *p1;
      break;
    case 2: //GPS mode "1" No fix "2" 2D fix "3" 3D fix
      mode = atoi(p1);
      break;
    case 15: //Parse PDOP
      PDOP = atof(p1);
      break;
    case 16: //Parse HDOP
      HDOP = atof(p1);
      break;
    case 17: //Parse VDOP
      VDOP = atof(p1);
      break;
    default: //ID(PRN) numbers of the satellites used in the solution
      if ((strlen(p1) > 0) and (param < 15))
      {
        satUsed[sat] = atoi(p1);
        sat++;
      }
      break;
    }
    if (p2 == NULL)
      break;
    param++;
  }
  satUsedCnt = sat;
  lock.unlock();
  return isOK;
}

/////////////////////////////////////////////////////////////

bool UGpsStatus::parseGPGSV(char * inBuf)
{ //   $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
  bool isOK = true;
  char *p1, *p2;
  int param = 0;
  int msg = 0;
  bool glonass = false;
  //
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  // lock while updating
  lock.lock();
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  while (*p2 >= ' ')
  { // separate into substring
    p1 = strsep(&p2, ",");
    switch (param)
    {
      case 0: // type $GPGSV, $GNGSV or $GLGSV
        glonass = (inBuf[2] == 'L');
        break;
      case 1:  // number of messages of this type
        //msgCnt = strtol(p1, NULL, 10);
        break;
      case 2: // this message is number
        msg = strtol(p1, NULL, 0);
        if ((msg == 1) and not glonass) // first message, assuming GPS satellites comes first
          satVisIdx = 0;
        break;
      case 3: // number of satellites in view
        satVisCnt = strtol(p1, NULL, 10);
        break;
      default: // satelite data
        if (param < 20)
        {
          switch ((param - 4) % 4)
          { // get data for one satellite
            case 0: // satelite ID number
              satVisGID[satVisIdx] = strtol(p1, NULL, 10);
              break;
            case 1: // Elevation in degrees
              satVisElev[satVisIdx] = strtol(p1, NULL, 10);
              break;
            case 2: // azimuth
              satVisAz[satVisIdx] = strtol(p1, NULL, 10);
              break;
            case 3: // SNR (0--99dB, 0=not tracked)
              satVisSN[satVisIdx] = strtol(p1, NULL, 10);
              satVisIdx++;
              break;
            default:;
              break;
          }
        }
    }
    if (p2 == NULL)
      break;
    param++;
  }
  //
  lock.unlock();
  return isOK;
}

/////////////////////////////////////////////////////////////

int UGpsStatus::getTrueVisCnt()
{
  int i;
  int cnt = 0;
  //
  for (i = 0; i < satVisCnt; i++)
  {
    if (satVisSN[i] > 0)
      cnt++;
  }
  return cnt;
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////


void UGpsLatLong::clear()
{
  lock.lock();
  valid = false;
  dop = 99.0;
  gmt.clear(); // time
  latDeg = 0.0;
  north_lat = '?';
  longDeg = 0.0;
  east_lon = '?';
  speed = 0.0;
  heading = 0.0;
  height = 0.0;
  height2 = 0;
  easting = 0;
  northing = 0;
  utmUpdated = false;
  lock.unlock();
  setGroundZero();
}


void UGpsLatLong::printLL(UDataBase * db, int client)
{
//   double easting = 0, northing = 0;
//   latlon2UTM(latDeg, longDeg, 32, &easting, &northing);
  const int MSL = 300;
  char s[MSL];
  snprintf(s, MSL, "# LatLong %.6f%c, %.6f%c, hdop %.2f, sats %d, (E,N)=%.2fm, %.2fm\n", latDeg, 'N', longDeg, 'E', dop, satellites, easting, northing);
  db->sendReply(s, client);
}

///////////////////////////////////////////////////

bool UGpsLatLong::parseGPRMC(char * inBuf, UTime lastTime)
{ // $GPRMC,180432,A,4027.027912,N,08704.857070,W, 000.04,181.9,131000,1.8,W,D*25
  //        hhmmss   ddmm.mmmmmm   dddmm,mmmmmm    knots   deg  ddmmyy, mag  A/D/N
  bool isOK = true;
  //char tmp[20][20]={{0},{0}};
  char parse[3][20]={{0},{0}};
  int len = 0;
  double dsec = 0.0, mm, dd;
  char *p1, *p2;
  int param = 0;
  struct tm t;
  unsigned long sec;
  //
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  lock.lock();
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  while (*p2 >= ' ')
  { // get string to next comma
    p1 = strsep(&p2, ",");
    switch (param)
    { // decode message parameters
      case 0: // just the name $GPRMC, $GNRMC
        break;
      case 1: // time of day
        len = sscanf(p1,"%2s%2s%s",parse[0],parse[1],parse[2]);
        isOK = len == 3;
        if (isOK)
        { // reconstruct date and time (from time only)
          int hour = atoi(parse[0]);
          t = lastTime.getTimeTm(false);
          if (t.tm_hour == 0 and hour > 12)
          { // the last time is tomorrow add a day
            lastTime -= 3600 * 24; // subtract one day
            t = lastTime.getTimeTm(false);
          }
          else if (t.tm_hour == 23 and hour < 12)
          { // last time is yesterday
            lastTime += 3600 * 24;
            t = lastTime.getTimeTm(false);
          }
          t.tm_hour = hour;
          t.tm_min = atoi(parse[1]);
          dsec = strtod(parse[2], NULL);
          t.tm_sec = int(dsec);
          dsec -= t.tm_sec;
          // convert to seconds in this epoc (since 1 jan 1970)
          sec = mktime(&t);
          // set as time type
          gmt.setTime(sec, (unsigned int)(dsec * 1e6));
        }
        break;
      case 2:  //Convert the validity of the measurement
        if(*p1 == 'V') // warning
          valid = false;
        else if(*p1 == 'A') // valid
          valid = true;
        else
          valid = false;
        break;
      case 3: // Parsse latitude (ddmm.mmmmm)
        sscanf(p1, "%2s%lf", parse[0], &mm);
        dd = atof(parse[0]);
        latDeg = dd + mm/60.0;
        break;
      case 4: // N or S
        north_lat = *p1;
        if (north_lat == 'S')
          latDeg = -latDeg;
        break;
      case 5: // Parse lontitude (dddmm.mmmmm)
        sscanf(p1, "%3s%lf", parse[0], &mm);
        dd = atof(parse[0]);
        longDeg = dd + mm/60.0;
        break;
      case 6: // E or W
        east_lon = *p1;
        if (east_lon == 'W')
          longDeg = -longDeg;
        break;
      case 7: // Convert speed
        speed = atof(p1) * KNOTS2M_S;
        break;
      case 8: // Convert heading
        heading = atof(p1);
        break;
      case 9: //Convert date
        len = sscanf(p1,"%2s%2s%2s",parse[0],parse[1],parse[2]);
        if(len == 3)
        {
          t.tm_mday = atoi(parse[0]);
          t.tm_mon = atoi(parse[1]) - 1;
          t.tm_year = atoi(parse[2]);
          if (t.tm_year < 70)
            // should be since year 1900
            t.tm_year += 100;
            // convert to seconds in this epoc (since 1 jan 1970)
          sec = mktime(&t);
          // set as time type
          gmt.setTime(sec, (unsigned int)(dsec * 1e6));
          isOK = (t.tm_mon < 12 and t.tm_mday > 0 and t.tm_hour < 24);
        }
        break;
      default: // ignore the rest
        break;
    }
    if (p2 == NULL)
      break;
    param++;
  }
  // release data
  lock.unlock();
  return isOK;
}

/////////////////////////////////////////////////////////////

bool UGpsLatLong::parseGPGGA(char * inBuf, UTime lastTime)
{ // $GPGGA,180432.00,4027.027912,N,08704.857070, W,2,07,1.0,212.15,M,-33.81,M,4.2,0555*73
  //        hhmmss.ss ddmm.mmmmmm   dddmm.mmmmmm     sat hdop hgt[m]   (DGPS base)
  bool isOK = false;
  char parse[3][20]={{0},{0}};
  int len;
  //static int GSV_old=0;
  int hour;
  double dsec, dd, mm;
  char * p1, *p2;
  int param = 0;
  struct tm t;
  unsigned long sec;
  //
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  lock.lock();
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  while (*p2 >= ' ')
  {
    p1 = strsep(&p2, ",");
    switch(param)
    { // decode parameters
      case 0: // just $GPGGA and $GNGGA
        break;
      case 1: // time hhmmss.ss
        //Convert the time
        len = sscanf(p1,"%2s%2s%s",parse[0],parse[1],parse[2]);
        if (len == 3)
        {
          UTime tn;
          tn.now();
          t = tn.getTimeTm(false);
          t = lastTime.getTimeTm(false);
          hour = atoi(parse[0]);
          if (t.tm_hour == 0 and hour > 12)
          { // the last time is tomorrow add a day
            lastTime -= 3600 * 24; // subtract one day
            t = lastTime.getTimeTm(false);
          }
          else if (t.tm_hour == 23 and hour < 12)
          { // last time is yesterday
            lastTime += 3600 * 24;
            t = lastTime.getTimeTm(false);
          }
          t.tm_hour = hour;
          t.tm_min = atoi(parse[1]);
          dsec = strtod(parse[2], NULL);
          t.tm_sec = int(dsec);
          dsec -= t.tm_sec;
          // convert to seconds in this epoc (since 1 jan 1970)
          sec = mktime(&t);
          // set as time type
          gmt.setTime(sec, (unsigned int)(dsec * 1e6));
        }
        break;
      case 2: // Parsse latitude
        sscanf(p1, "%2s%lf", parse[0], &mm);
        dd = atof(parse[0]);
        latDeg = dd + mm/60.0;
        break;
      case 3: // N or S
        north_lat = *p1;
        if (north_lat == 'S')
          latDeg = -latDeg;
        break;
      case 4: // Parse lontitude
        sscanf(p1, "%3s%lf", parse[0], &mm);
        dd = atof(parse[0]);
        longDeg = dd + mm/60.0;
        break;
      case 5: // E or W
        east_lon = *p1;
        if (east_lon == 'W')
          longDeg = -longDeg;
        break;
      case 6: // Parse quality
        quality = atoi(p1);
        valid = (quality == 1 or quality == 2);
        egnos = (quality == 2);
        break;
      case 7: // Parse number of satellites
        satellites = atoi(p1);
        break;
      case 8: // Parse HDOP
        dop = atof(p1);
        isOK = true;
        break;
      case 9: // Antenna height above mean sea level
        height = atoi(p1);
        break;
      case 10: //height difference to geoid
        height2 = atoi(p1);
        break;
      default:
        ; // ignore the rest
        break;
    }
    if (p2 == NULL)
      break;
    param++;
  }
  lock.unlock();
  return isOK;
}

void UGpsLatLong::toUTM()
{
//   void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
//                double * UTMNorthing, double * UTMEasting, int zone)
  LLtoUTM(23, latDeg, longDeg, &northing, & easting, 32);
  utmUpdated = true;
}

void UGpsLatLong::toLog(FILE * logfile)
{
  if (logfile != NULL)
  {
    UTime t;
    t.now();
    fprintf(logfile, "%ld.%03ld %.5f %.5f %.2f %.2f %.2f %.3f %d\n", t.getSec(), t.getMilisec(), latDeg, longDeg, height, easting, northing, dop, satellites);
  }
}

void UGpsLatLong::getPoseMsg(char* s, int MSL)
{
  snprintf(s, MSL, "gpsp %.5f %.5f %.2f %.2f %.2f %.3f %d\n", latDeg, longDeg, height, easting, northing, dop, satellites);
}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void UGps::clear()
{
  latLong.clear();
  status.clear();
  addKey("gnss");
}

void UGps::sendHelp(const int client)
{
  db->sendReply("# ------- GNSS ----------\r\n", client);
  sendKeyList(client);
}

/////////////////////////////////////////////////////////////

bool UGps::parseNMEA(char* inBuf, UTime tod)
{
  bool isOK;
  char * p1;
   //Find the string format
  isOK = validateNMEA(inBuf);
  //strip off linefeed
  p1 = &inBuf[strlen(inBuf)-1];
  while (*p1 < ' ' and p1 > inBuf)
    *p1-- = '\0';
  //
  if (not isOK)
    printf("Failed to validate the GPS message: '%s'\n", inBuf);
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  if (isOK)
  { //Recommended Minimum Specific GNSS information relayed by all satellites
    if((strncmp(inBuf,"$GPRMC", 6) == 0) or (strncmp(inBuf,"$GNRMC", 6) == 0))
    { // $GPRMC,180432,A,4027.027912,N,08704.857070,W, 000.04,181.9,131000,1.8,W,D*25
      //        hhmmss   ddmm.mmmmmm   dddmm,mmmmmm    knots   deg  ddmmyy, mag  A/D/N
      if (latLong.parseGPRMC(inBuf, lastTime))
      {
        lastTime = latLong.gmt;
        latLongUpdated = true;
        msgTime = tod;
      }
    }
    //GPS Fix data
    else if((strncmp(inBuf,"$GPGGA", 6) == 0) or (strncmp(inBuf,"$GNGGA", 6) == 0))
    { // $GPGGA,180432.00,4027.027912,N,08704.857070, W,2,07,1.0,212.15,M,-33.81,M,4.2,0555*73
      //        hhmmss.ss ddmm.mmmmmm   dddmm.mmmmmm     sat hdop hgt[m]   (DGPS base)
      if (latLong.parseGPGGA(inBuf, lastTime))
      {
        lastTime = latLong.gmt;
        latLongUpdated = true;
        msgTime = tod;
        //latLong.printLL();
      }
    }
    // Geografic position Lat/Lon
    // $GPGLL,8960.0000,N,00000.0000,E,105317.709,V,N*49
    // Redundant information
    else if((strncmp(inBuf,"$GPGLL", 6) == 0) or (strncmp(inBuf,"$GNGLL", 6) == 0))
    {
      //printf("$GPGLL - not supported\n");
      isOK = false;
    }
    //GNSS Dillution Of Presition and Active Satellites
    else if((strncmp(inBuf,"$GPGSA", 6) == 0) or (strncmp(inBuf,"$GNGSA", 6) == 0))
    {
      if (status.parseGPGSA(inBuf))
        statusUpdated = true;
    }
    //GNSS Satellites in view
    //There has not been done any handling on missing GSV messages
    //GSV_new and GSV_old are for this purpose
    else if((strncmp(inBuf,"$GPGSV", 6) == 0) or (strncmp(inBuf,"$GNGSV", 6) == 0) or (strncmp(inBuf,"$GLGSV", 6) == 0))
    {
      if (status.parseGPGSV(inBuf))
        statusUpdated = true;
    }
    //Course over Ground and Ground Speed
    //Redundant information
    else if((strncmp(inBuf,"$GPVTG", 6) == 0) or (strncmp(inBuf,"$GNVTG", 6) == 0))
    {
      //printf("$GPVTG - not supported\n");
      isOK = false;
    }
    else if((strncmp(inBuf,"$GPZDA", 6) == 0) or (strncmp(inBuf,"$GNZDA", 6) == 0))
    {
      //printf("$GPVTG - not supported\n");
      isOK = false;
    }
    //Position figure of merit
    else if(strncmp(inBuf,"$PFST", 5) == 0)
    {
      printf("$PFST - proparity sentence not supported\n");
      isOK = false;
    }
    //Proparitary SiRF message
    else if(strncmp(inBuf,"$PSRF151", 8) == 0)
    {
      if(true)
      {
        printf("%s\n", inBuf);
      }
      p1 = strchr(inBuf, ',');
      status.EGNOS = strtol(p1, NULL, 10);
      //       if(PRINT_INFO)
      printf("EGNOS status %i\n",status.EGNOS);
    }
    else if (strncmp(inBuf, "$GNTXT", 6) == 0)
      ;// e.g. version and model of GNSS device or error messages;
    else if (strncmp(inBuf, "$GNGNS", 6) == 0)
      ;// Lat-Long only;
    else if (strncmp(inBuf, "$GNGST", 6) == 0)
      ; // error estimate
    else if (strncmp(inBuf, "$GBGSV", 6) == 0)
      ;  // don't know
    else if (strncmp(inBuf, "$GAGSV", 6) == 0)
      ;//  don't know
    else
      printf("Data error, tag not known (%s)\n", inBuf);
    //
  }
  //
  return isOK;
}

/////////////////////////////////////////////////////

bool UGps::validateNMEA(char* inBuf)
{
  char tmp = 0;
  int i = 2;
  bool result;
  //
  tmp = inBuf[1];
  if(inBuf[0] == '$' && isalpha(inBuf[1]))
  {
    do
    {
      tmp = tmp ^ inBuf[i];
      i++;
    } while((inBuf[i] != '*') and (inBuf[i] >= ' '));
  }
  //
  if((tmp >> 4) == ascii2hex(inBuf[i + 1]) and (tmp & 0x0f) == ascii2hex(inBuf[i + 2]))
    result = true;
  else
    result = false;

  return result;
}

//////////////////////////////////////////////////

UTime UGps::getTimeLocal()
{
  UTime t;
  struct tm ts;
  // get time assuming lastTime is GMT
  ts = lastTime.getTimeTm(true);
  // set time using this data
  t.setTime(ts.tm_year, ts.tm_mon, ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec, lastTime.getMicrosec());
  return t;
}

/////////////////////////////////////////////////////

char UGps::ascii2hex(char input)
{
  if(isalpha(input))
  {
    if(isupper(input))
      return input - 'A' + 10;
    else
      return input - 'a' + 10;
  }
  else
    return input - '0';
}

/////////////////////////////////////////////////////////////

void UGps::UResGpsInit()
{ // these first two lines is needed
  // to save the ID and version number
  clear();
  latLongUpdated = false;
  utmUpdated = false;
  statusUpdated = false;
  lastTime.Now();
  //
  dataRxMsgCnt = 0;
}

///////////////////////////////////////////

UGps::~UGps()
{
  // stop read thread
  stop();
  closeLog();
}

///////////////////////////////////////////

void UGps::printStatus(int client)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  const int MSL = 1200;
  char s[MSL];
  char s2[100];
  //
  db->sendReply("# ------ GNSS interface ------------\n", client);
  sendKeyList(client);
  snprintf(s, MSL,"# Connected (%d) to %s (at %d baud)\n", deviceOpen, devicename, baudrate);
  db->sendReply(s, client);
  if (deviceOpen)
  {
    lastTime.getTimeAsString(s2, true);
    snprintf(s, MSL,"    Last update at %s was\n       '%s'\n", s2, sentence);
    db->sendReply(s, client);
    //
    latLong.printLL(db, client);
    printDataStatus(client);
  }
}



////////////////////////////////////////////////////////////////////

void UGps::toLog(const char * message)
{
  UTime t;
  //
  if (logGps.logIsOpen())
  {
    t.now();
    if (message[strlen(message) -1] < ' ')
      fprintf(logGps.getF(), "%lu.%06lu %s", t.getSec(), t.getMicrosec(), message);
    else
      fprintf(logGps.getF(), "%lu.%06lu %s\n", t.getSec(), t.getMicrosec(), message);
  }
}

/////////////////////////////////////////////////////////////////////

void UGps::run()
{
  const int MRXB = 1000;
  char rxBuff[MRXB + 1];
  bool isOK;
  UTime tod;
  FILE * gpsDev = NULL;
  int failCnt = 0;
  int rxBuffCnt = 0;
  const double rxTimeout = 0.030;
  //
//   threadRunning = true;
  // wait to allow init script to finish
  sleep(1);
  while (not th1stop)
  {
    if (deviceShouldOpen and not deviceOpen)
    { // device is to be open - try
//       printf("UResGps::run: Trying to open %s\n", devicename);
      if (baudrate > 0)
      {
        setDeviceSpeed(devicename, baudrate);
      }
      gpsDev = fopen(devicename, "r");
      if (gpsDev == NULL)
      { // file not found
        if (failCnt < 5)
        {
          toLog("# Device file not found (readable)");
          printf("UGps::run: GPS device %s could not be opened\n", devicename);
          failCnt++;
        }
      }
      else
      {
        deviceOpen = true;
        failCnt = 0;
        rxBuffCnt = 0;
        rxBuff[0] = '\0';
      }
    }
    else if (not deviceShouldOpen and deviceOpen)
    { // device is to be closed
      printf("UResGps::run: Trying to close %s\n", devicename);
      fclose(gpsDev);
      gpsDev = NULL;
      deviceOpen = false;
    }
    if (deviceOpen)
    { // is still open, so get data
      char * pEnd = rxBuff;
      //printf("UResGps::run: Trying to read from %s for device %s\n", varDevice->getValues(), getResID());
      /// replace with non-blocking - see below
      //p1 = fgets(rxBuff, MRXB, gpsDev);
      //printf("UResGps::run: read from %s: %s", varDevice->getValues(), rxBuff);
      if (rxBuffCnt > 1 and rxBuff[0] != '$')
        // start of buffer is not start of message, so advance
        rxBuffCnt = advanceNmeaMessage(rxBuff, rxBuffCnt, rxBuff);
      if (not hasNmeaMessage(rxBuff, rxBuffCnt, NULL))
      { // get new data from device
        int n;
        n = receiveFromDevice(gpsDev, &rxBuff[rxBuffCnt], MRXB - rxBuffCnt, rxTimeout);
        if (n > 0)
          rxBuffCnt+= n;
      }
      if (hasNmeaMessage(rxBuff, rxBuffCnt, &pEnd))
      { // log raw message
        toLog(rxBuff);
        isOK = parseNMEA(rxBuff, dataRxTime);
        if (isOK and latLongUpdated)
        { // update global variables - etc
          latLong.toUTM();
          latLongUpdated = false;
          updateSubscriptionData();
          if (logfile != NULL)
          {
            latLong.toLog(logfile);
//             UTime t;
//             t.now();
//             fprintf(logfile, "%ld.%03ld %.5f %.5f %.5f %.2f %.2f\n", t.getSec(), t.getMilisec(), latL);
          }
        }
        // advance one message
        rxBuffCnt = advanceNmeaMessage(rxBuff, rxBuffCnt, pEnd);
      }
    }
    if (not deviceOpen)
      usleep(100000);
  }
//   threadRunning = false;
}

///////////////////////////////////////////////////

bool UGps::hasNmeaMessage(char * rxBuff, int rxBuffCnt, char ** pEnd)
{
  char * p1 = rxBuff;
  bool hasNmea = false;
  //
  if (*p1 == '$')
  {
    p1 = strchr(p1, '*');
    if (p1 != NULL)
    {
      int m = p1 - rxBuff;
      if ((rxBuffCnt - m) > 2)
        hasNmea = true;
      p1 += 3;
      *p1 = '\0';
      if (pEnd != NULL)
        *pEnd = p1 + 1;
    }
  }
  return hasNmea;
}

///////////////////////////////////////////////////

int UGps::advanceNmeaMessage(char * rxBuff, int rxBuffCnt, char * pEnd)
{
  char * p1 = pEnd;
  int newBufCnt = 0;
  //
  p1 = strchr(p1, '$');
  if (p1 != NULL)
  { // there is (start of) next message
    newBufCnt = rxBuffCnt - (p1 - rxBuff);
    if (newBufCnt > 0)
      memmove(rxBuff, p1, newBufCnt);
    else
      newBufCnt = 0;
  }
  // make sure buffer is 0 terminated
  rxBuff[newBufCnt] = '\0';
  // return new length
  return newBufCnt;
}


///////////////////////////////////////////////////

int UGps::receiveFromDevice(FILE * device,
                               char * start, // put data here
                               int maxLng,   // max number of characters
                               const double timeoutSec) // timeout in seconds
{ // poll wait time
  int pollTime = int(timeoutSec * 1000.0); // ms
  int n, m = maxLng;
  struct pollfd ps;
  // Set poll structure for test of indata available
  ps.fd = device->_fileno;
  ps.events = POLLIN;
  bool timeout;
  int result = 0; // number of characters received
  char * bp = start; // put new data here
  char * p1;
  //
  // receive a number of data that should include a header
  /* get new data */
  timeout = false;
  while ((m > 0) and not timeout)
  { // Wait for data in up to the timeout period
    if (poll(&ps, 1, pollTime) != POLLIN)
      // timeout or other error - return
      timeout = true;
    else
    { /* data available, read up to a full message */
      n = read(device->_fileno, bp, m);
      if (n == -1)
      { // error
        perror("Error in read from serial line");
        result = -1;
        break;
      }
      else
      { //
        if (result == 0)
        { // first data - timestamp
          dataRxTime.Now();
        }
        bp[n] = '\0';
        p1 = strchr(bp, '\r');
        if (p1 == NULL)
        {
          p1 = bp-1;
          // wait for approx 50 characters
          usleep(600000000/baudrate);
        }
        //printf("len=%d, CR=%d n=%d, m=%d bp='%s'\n", strlen(bp), p1 - bp, n, m, bp);
        // terminate string
        m -= n;
        result += n;
        // stop if a newline is received
        if (strchr(bp, '\n') != NULL)
          // a full message is received (at least one).
          break;
        bp = &start[result];
      }
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////

void UGps::openLog()
{
  logGps.createLogfile("log_gps");
  if (logfile != NULL)
  { //fprintf(logfile, "%ld.%03ld %.5f %.5f %.2f %.2f %.3f %d\n", t.getSec(), t.getMilisec(), latDeg, longDeg, height, easting, northing, dop, satellites);
    fprintf(logfile, "%% GPS lat-long logfile\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 Latitude in decimal degrees\n");
    fprintf(logfile, "%% 3 Longitude in decimal degrees\n");
    fprintf(logfile, "%% 4 Height above sea level in m\n");
    fprintf(logfile, "%% 5 Easing (WGS84, zone 32)\n");
    fprintf(logfile, "%% 6 Northing (WGS84, zone 32)\n");
    fprintf(logfile, "%% 7 dop (horizontal) \n");
    fprintf(logfile, "%% 8 Number of satellites used\n");
  }
}

void UGps::updateSubscriptionData()
{ // send pose data to client
  const int MSL = 300;
  char s[MSL];
  latLong.getPoseMsg(s, MSL);
  raw[0]->save4publish(s);
}


/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

int set_serial(int fd, int speed)
{ // set speed of serial port on this computer
  int result = 1;
  struct termios options;
  
  if (tcgetattr(fd, &options) == -1)
  {
    perror("set_serial: tcgetattr(fd, &options) ");
    //exit(-1);
    result = 0;
  }
  if (result)
  {
    if (speed == 2400)
    {
      cfsetispeed(&options, B2400);  /*/Set input baudrate */
      cfsetospeed(&options, B2400);  /*/Set output baudrate */
    }
    else if (speed == 4800)
    {
      cfsetispeed(&options, B4800);  /*/Set input baudrate */
      cfsetospeed(&options, B4800);  /*/Set output baudrate */
    }
    else if (speed == 9600)
    {
      cfsetispeed(&options, B9600);  /*/Set input baudrate */
      cfsetospeed(&options, B9600);  /*/Set output baudrate */
    }
    else if (speed == 19200)
    {
      cfsetispeed(&options, B19200);  /*/Set input baudrate */
      cfsetospeed(&options, B19200);  /*/Set output baudrate */
    }
    else if (speed == 38400)
    {
      cfsetispeed(&options, B38400);  /*/Set input baudrate */
      cfsetospeed(&options, B38400);  /*/Set output baudrate */
    }
    else if (speed == 57600)
    {
      cfsetispeed(&options, B57600);  /*/Set input baudrate */
      cfsetospeed(&options, B57600);  /*/Set output baudrate */
    }
    else if (speed == 115200)
    {
      cfsetispeed(&options, B115200);  /*/Set input baudrate */
      cfsetospeed(&options, B115200);  /*/Set output baudrate */
    }
    else if (speed == 230400)
    {
      cfsetispeed(&options, B230400);  /*/Set input baudrate */
      cfsetospeed(&options, B230400);  /*/Set output baudrate */
    }
    else if (speed == 500000)
    {
      cfsetispeed(&options, B500000);  /*/Set input baudrate */
      cfsetospeed(&options, B500000);  /*/Set output baudrate */
    }
    else
    {
      fprintf(stderr, "Trying to set speed of serial port to unsupported %d bit/sec\n", speed);
      fprintf(stderr, "- supports 2400,4800,9600,19200,38400,57600,115200,230400,500000 bit/sec\n");
    }
    
    /* Enable the receiver and set local mode... */
    options.c_cflag |= (CLOCAL | CREAD);
    
    /*/ Set to no parity 8 bit,1 stop, 8N1
    //options.c_cflag &= ~PARENB;
    //options.c_cflag &= ~CSTOPB;
    //options.c_cflag &= ~CSIZE; */
    options.c_cflag |= CS8;
    /*
     *    //options.c_cflag = 0;
     *    //options.c_cflag |= (CLOCAL | CREAD | CS8);
     * 
     * 
     *    //Using raw input mode
     *    //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); */
    options.c_lflag = 0;
    /*
     *    //ignore parity errors
     *    //options.c_iflag |= IGNPAR; */
    options.c_iflag =0;  /*/Must be zero, change it at own risk */
    /*
     *    //Set output to raw mode
     *    //options.c_oflag &= ~OPOST; */
    options.c_oflag = 0;
    
    /*/Set the new options for the port... */
    if (tcsetattr(fd, TCSANOW, &options) == -1)
    {
      perror("can not set serial port parameters\n");
      //exit(-1);
      result = 0;
    }
  }
  if (result)
  {  /* flush unread data */
    tcflush(fd, TCIFLUSH);
    // request low-latency from kernel
    struct serial_struct ss;
    ioctl(fd, TIOCGSERIAL, &ss);
    ss.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &ss);
  }  
  //
  return !result;
}

//////////////////////////////////////////

// int gpsPortErrCnt = 0;

int setDeviceSpeed(const char * devName, int devSpeed)
{
  int result;
  int hfd = -1;
  /*  const int MSL = 30;
   *  char s[MSL];*/
  //
  if (hfd < 0)
  {
    /* hfd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY); */
//     printf("open_port: Trying to open port %s\n", devName);
    hfd = open(devName, O_RDWR | O_NOCTTY );
    if (hfd == -1)
    { /* Could not open the port. */
//       if (gpsPortErrCnt < 10)
//         perror("open_port:: Unable to open GPS port: ");
//       gpsPortErrCnt++;
    }
    else
    { /* fcntl(fd, F_SETFL, FNDELAY);   non blocking by using FNDELAY */
      //fcntl(hfd, F_SETFL, 0);      /* Blocking */
      /* setting serial port speed */
      printf("# UGPS::setDeviceSpeed: %d bit/sec\n", devSpeed);
      set_serial(hfd, devSpeed);
      tcflush(hfd, TCIFLUSH); /* flush unread data */
//       gpsPortErrCnt = 0;
      //
    }
  }
  result = hfd >= 0;
  close(hfd);
  return !result;
}


bool latlon2UTM(double latDeg, double longDeg, int zone, double * easting, double *  northing)
{
  // void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
  //             double &UTMNorthing, double &UTMEasting, char* UTMZone)
  LLtoUTM(23, latDeg, longDeg, northing, easting, zone);
  return true;
}

///////////////////////////////////////////////////////////////////////

bool utm2latlon(double easting, double northing, int zone,
                double * latitude, double * longitude)
{ // 23=WGS-84
  UTMtoLL(23, northing, easting, zone, latitude, longitude);
  return true;
}

// /////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////

const double PI = M_PI;
const double FOURTHPI = PI / 4;
const double deg2rad = PI / 180;
const double rad2deg = 180.0 / PI;

class Ellipsoid
{
public:
  Ellipsoid(){};
  Ellipsoid(int Id, const char* name, double radius, double ecc)
  {
    id = Id; ellipsoidName = name;
    EquatorialRadius = radius; eccentricitySquared = ecc;
  }
  
  int id;
  const char * ellipsoidName;
  double EquatorialRadius;
  double eccentricitySquared;
};


static Ellipsoid ellipsoid[] =
{//  id, Ellipsoid name, Equatorial Radius, square of eccentricity
  Ellipsoid( -1, "Placeholder", 0, 0),//placeholder only, To allow array indices to match id numbers
  Ellipsoid( 1, "Airy", 6377563, 0.00667054),
  Ellipsoid( 2, "Australian National", 6378160, 0.006694542),
  Ellipsoid( 3, "Bessel 1841", 6377397, 0.006674372),
  Ellipsoid( 4, "Bessel 1841 (Nambia) ", 6377484, 0.006674372),
  Ellipsoid( 5, "Clarke 1866", 6378206, 0.006768658),
  Ellipsoid( 6, "Clarke 1880", 6378249, 0.006803511),
  Ellipsoid( 7, "Everest", 6377276, 0.006637847),
  Ellipsoid( 8, "Fischer 1960 (Mercury) ", 6378166, 0.006693422),
  Ellipsoid( 9, "Fischer 1968", 6378150, 0.006693422),
  Ellipsoid( 10, "GRS 1967", 6378160, 0.006694605),
  Ellipsoid( 11, "GRS 1980", 6378137, 0.00669438),
  Ellipsoid( 12, "Helmert 1906", 6378200, 0.006693422),
  Ellipsoid( 13, "Hough", 6378270, 0.00672267),
  Ellipsoid( 14, "International", 6378388, 0.00672267),
  Ellipsoid( 15, "Krassovsky", 6378245, 0.006693422),
  Ellipsoid( 16, "Modified Airy", 6377340, 0.00667054),
  Ellipsoid( 17, "Modified Everest", 6377304, 0.006637847),
  Ellipsoid( 18, "Modified Fischer 1960", 6378155, 0.006693422),
  Ellipsoid( 19, "South American 1969", 6378160, 0.006694542),
  Ellipsoid( 20, "WGS 60", 6378165, 0.006693422),
  Ellipsoid( 21, "WGS 66", 6378145, 0.006694542),
  Ellipsoid( 22, "WGS-72", 6378135, 0.006694318),
  Ellipsoid( 23, "WGS-84", 6378137, 0.00669438)
};


void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
             double * UTMNorthing, double * UTMEasting, int zone)
{
  //converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
  //East Longitudes are positive, West longitudes are negative.
  //North latitudes are positive, South latitudes are negative
  //Lat and Long are in decimal degrees
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  // from http://www.gpsy.com/gpsinfo/geotoutm/
  // changed to forced zone number by Christian Andersen DTU
  
  double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
  double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
  double k0 = 0.9996;
  
  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;
  
  //Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long+180)-int((Long+180)/360)*360-180; // -180.00 .. 179.9;
  
  double LatRad = Lat*deg2rad;
  double LongRad = LongTemp*deg2rad;
  double LongOriginRad;
  int    ZoneNumber;
  
  if (true)
  {
    ZoneNumber = int((LongTemp + 180)/6) + 1;
    
    if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
      ZoneNumber = 32;
    // Special zones for Svalbard
    if( Lat >= 72.0 && Lat < 84.0 )
    {
      if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
      else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
      else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
      else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
    }
  }
  else
    ZoneNumber = zone;
  
  LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone
  LongOriginRad = LongOrigin * deg2rad;
  
  // compute the UTM Zone from the latitude and longitude
  // sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
  
  eccPrimeSquared = (eccSquared)/(1-eccSquared);
  
  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);
  
  M = a*((1       - eccSquared/4          - 3*eccSquared*eccSquared/64    - 5*eccSquared*eccSquared*eccSquared/256)*LatRad
  - (3*eccSquared/8       + 3*eccSquared*eccSquared/32    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
  + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
  - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));
  
  *UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
  + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
  + 500000.0);
  
  *UTMNorthing = (double)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
  + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
  if(Lat < 0)
    *UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}

char UTMLetterDesignator(double Lat)
{
  //This routine determines the correct UTM letter designator for the given latitude
  //returns 'Z' if latitude is outside the UTM limits of 84N to 80S
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  char LetterDesignator;
  
  if((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
  else if((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
  else if((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
  else if((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
  else if((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
  else if((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
  else if((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
  else if((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
  else if((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
  else if(( 8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
  else if(( 0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
  else if((-8> Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  else LetterDesignator = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits
  
  return LetterDesignator;
}


void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting,
             int zone /*const char* UTMZone*/, double * Lat,  double * Long )
{
  //converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
  //East Longitudes are positive, West longitudes are negative.
  //North latitudes are positive, South latitudes are negative
  //Lat and Long are in decimal degrees.
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  // from http://www.gpsy.com/gpsinfo/geotoutm/
  
  double k0 = 0.9996;
  double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
  double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
  double eccPrimeSquared;
  double e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1Rad;
  double x, y;
  int ZoneNumber;
  //  char* ZoneLetter;
  //  int NorthernHemisphere; //1 for northern hemispher, 0 for southern
  
  x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
  y = UTMNorthing;
  
  //  ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
  ZoneNumber = zone;
  //  if((*ZoneLetter - 'N') >= 0)
  //  NorthernHemisphere = 1;//point is in northern hemisphere
  //   else
  //     NorthernHemisphere = 0;//point is in southern hemisphere
  
  LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone
  
  eccPrimeSquared = (eccSquared)/(1-eccSquared);
  
  M = y / k0;
  mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64-5*eccSquared*eccSquared*eccSquared/256));
  
  phi1Rad = mu    + (3*e1/2-27*e1*e1*e1/32)*sin(2*mu)
  + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
  +(151*e1*e1*e1/96)*sin(6*mu);
  //phi1 = phi1Rad*rad2deg;
  
  N1 = a/sqrt(1-eccSquared*sin(phi1Rad)*sin(phi1Rad));
  T1 = tan(phi1Rad)*tan(phi1Rad);
  C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
  R1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
  D = x/(N1*k0);
  
  *Lat = phi1Rad - (N1*tan(phi1Rad)/R1)*(D*D/2-(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
  +(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720);
  *Lat = *Lat * rad2deg;
  
  *Long = (D-(1+2*T1+C1)*D*D*D/6+(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
  *D*D*D*D*D/120)/cos(phi1Rad);
  *Long = LongOrigin + *Long * rad2deg;
  
}
