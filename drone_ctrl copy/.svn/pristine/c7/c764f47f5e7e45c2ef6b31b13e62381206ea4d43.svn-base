/***************************************************************************
 *   Copyright (C) 2006 by DTU (by Christian Andersen, Lars m.fl.)         *
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

#ifndef URESGPS_H
#define URESGPS_H

#include <cstdlib>

#include <ulogfile.h>
#include <urun.h>
#include <udata.h>

//gps = new UGps(this, "/dev/GNSS", 9600);
//gps = new UGps(this, "/dev/ttyUSB0", 4800);
// gps = new UGps(this, "/dev/ttyUSB0", 9600);
//   gps = new UGps(this, "/dev/ttyACM0", 9600);


class UDataBase;

/// Conversion factor from knots to m/s
#define KNOTS2M_S (1852.0/3600.0)
//#define KNOTS2M_S 0.5144444444
/// Number of satellites supported by the GPS (12 by the SIRFII chipset)
#define GPS_SATELLITES_SUPPORTED 100
/// Number of satellites in the GPS system
#define GPS_SATELLITES_TOTAL 100
/// GPS max sentence length
#define GPS_SENTENCE_MAX_LENGTH 200

/// GPS status struct to hold the advanced status information about the GPS system
class UGpsStatus
{
public:
  /**
  Constructor */
  UGpsStatus()
  {
    clear();
  };
  /**
  Clear the structure */
  void clear();
  /**
  GSA - GPS DOP and active satellites. This sentence provides details on the nature of the fix.
  It includes the numbers of the satellites being used in the current solution and the DOP.
  DOP (dilution of precision) is an indication of the effect of satellite geometry on the
  accuracy of the fix. It is a unitless number where smaller is better.
  For 3D fixes using 4 satellites a 1.0 would be considered to be a perfect number,
  however for overdetermined solutions it is possible to see numbers below 1.0.

  There are differences in the way the PRN's are presented which can effect the ability
  of some programs to display this data. For example, in the example shown below there
  are 5 satellites in the solution and the null fields are scattered indicating that the
  almanac would show satellites in the null positions that are not being used as part of
  this solution.
  Other receivers might output all of the satellites used at the beginning of the sentence
  with the null field all stacked up at the end.
  This difference accounts for some satellite display programs not always being able to
  display the satellites being tracked. Some units may show all satellites that have
  ephemeris data without regard to their use as part of the solution but this is non-standard.

  $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39

Where:
  GSA      Satellite status
  A        Auto selection of 2D or 3D fix (M = manual)
  3        3D fix - values include: 1 = no fix
  2 = 2D fix
  3 = 3D fix
  04,05... PRNs of satellites used for fix (space for 12)
  2.5      PDOP (dilution of precision)
  1.3      Horizontal dilution of precision (HDOP)
  2.1      Vertical dilution of precision (VDOP)
   *39      the checksum data, always begins with *
   */
  bool parseGPGSA(char * inBuf);
  /**
    GSV - Satellites in View shows data about the satellites that the unit might be able to
    find based on its viewing mask and almanac data. It also shows current ability to track this data.
    Note that one GSV sentence only can provide data for up to 4 satellites and thus there may
    need to be 3 sentences for the full information. It is reasonable for the GSV sentence to contain
    more satellites than GGA might indicate since GSV may include satellites that are not used as part
    of the solution.
    It is not a requirment that the GSV sentences all appear in sequence.
    To avoid overloading the data bandwidth some receivers may place the various sentences in
    totally different samples since each sentence identifies which one it is.

    The field called SNR (Signal to Noise Ratio) in the NMEA standard is often referred to as
    signal strength. SNR is an indirect but more useful value that raw signal strength.
    It can range from 0 to 99 and has units of dB according to the NMEA standard, but the various
    manufacturers send different ranges of numbers with different starting numbers so the values
    themselves cannot necessarily be used to evaluate different units. The range of working values
    in a given gps will usually show a difference of about 25 to 35 between the lowest and
    highest values, however 0 is a special case and may be shown on satellites that are in
    view but not being tracked.

    $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75

  Where:
    GSV          Satellites in view
    2            Number of sentences for full data
    1            sentence 1 of 2
    08           Number of satellites in view

    01           Satellite PRN number
    40           Elevation, degrees
    083          Azimuth, degrees
    46           SNR - higher is better
    for up to 4 satellites per sentence
    *75          the checksum data, always begins with *
  */
  bool parseGPGSV(char * inBuf);
  /**
  Get number of visible satellites with a signal strength obove zero */
  int getTrueVisCnt();


public:
   /// Operational mode
  char opr_mode;
   /// Calculation mode
  int mode;
   /// Number of the satellites used to make calculation (GID)
  int satUsed[GPS_SATELLITES_SUPPORTED];
  int satUsedCnt;
   /// Visible satellites with information: Number (GID), elevation (0-90 deg),
   /// azimuth (0-360 deg), Signal to Noise Ratio (1-99 db-Hz)
  int satVisGID[GPS_SATELLITES_TOTAL];
  /// elevation of satelite 0..90 (0 is hirizon and 90 is zenith)
  int satVisElev[GPS_SATELLITES_TOTAL];
  /// azimuth in compas degrees (0..359)
  int satVisAz[GPS_SATELLITES_TOTAL];
  /// Signal to noice ratio 1--99 db-Hz
  int satVisSN[GPS_SATELLITES_TOTAL];
  /// total number of satellites in view
  int satVisCnt;
  /// index during reception of data
  int satVisIdx;
   /// Positional Dillution of Precision (3D)
  double PDOP;
   /// Horizontal Dillution of Precision (In a plane)
  double HDOP;
   /// Vertical Dillution of precision (Height only)
  double VDOP;
  /** Show if the GPS is in SBAS augmentation mode (EGNOS or WAAS)
      from SiRF message */
  int EGNOS;
  /// last message received
  char sentence[GPS_SENTENCE_MAX_LENGTH];
  
  std::mutex lock;
};


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/// GPS latlon struct to hold the GPS fix information in the latitude/lontitude system
class UGpsLatLong
{
public:
  /**
  Constructor */
  UGpsLatLong()
  {
    clear();
  }
  /**
  Clear the structure */
  void clear();
  /**
   * Parse the GPRMC message
   * \param[in] *inBuf Pointer to the string to be parsed.
   * \param[in] *lastTime is the last time data were received, to reconstruct date (also for replay).
   * \return true on succes
   */
  bool parseGPRMC(char * inBuf, UTime lastTime);
  /**
   * Parse the GPRMC message
   * \param[in] *inBuf Pointer to the string to be parsed.
   * \param[in] *lastTime is the last time data were received, to reconstruct date (also for replay).
   * \return true on succes
   */
  bool parseGPGGA(char * inBuff, UTime lastTime);
  /**
   * print to console */
  void printLL(UDataBase * db, const int client);
  /**
   * convert LL to UTM */
  void toUTM();
  /**
   * save data to logfile, if logfile is open */
  void toLog(FILE * logfile);
  /**
   * format pose data to string */
  void getPoseMsg(char * s, int MSL);
  /**
   * set start UTM - i.e. when not armed */
  void setGroundZero()
  {
    lock.lock();
    eastingBase = easting;
    northingBase = northing;
    lock.unlock();
  }

public:
   /// Show if the fix is valid
  bool valid;
   /// Show the quality of the fix (No gps/gps/dgps)
  int quality;
  ///Has the fix used the EGNOS correction
  bool egnos;
   /// Show the nuber of satellites used in the fix
  int satellites;
   /// Dillution of Precision for the fix
  float dop;
   /// GMT time, as reported by the message (date may be reconstructed)
  UTime gmt;
   /// Latitude coordinate in decimal degrees, with North as positive
  double latDeg;
   /// Direction of the latitude
  char north_lat;
   /// Lontitude coordinate in decimal degrees, with East as positive
  double longDeg;
   /// Direction of the lontitude
  char east_lon;
   /// Speed of the reciever (m/s)
  float speed;
   /// Heading of the reciever (0-360 deg)
  float heading;
   /// Antenna height data (geoid height)
  double height;
   /// Height differential between an ellipsion and geoid
  int height2;
  /// last message received
  char sentence[GPS_SENTENCE_MAX_LENGTH];
  /// converted to UTM (zone 32, WGS84 (23))
  double easting, northing;
  /// set true when updated
  bool utmUpdated;
  /// start position (when disarmed)
  double eastingBase, northingBase;
public:
  std::mutex lock;
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class ULogfile;

/**

@author Christian Andersen
*/
class UGps : public UData
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResGps) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UGps(UDataBase * parent, const char * deviceName)
    : UData(parent)
  { // set name and version
    const char * p1 = strchr(deviceName, ':');
    int n = strlen(deviceName);
    if (p1 != nullptr)
    {
      n = p1 - deviceName;
      p1++;
      baudrate = strtol(p1, nullptr, 10);
    }
    else
      baudrate = 9600;
    strncpy(devicename, deviceName, MFL);
    // terminate in replacement for ':'
    devicename[n] = '\0';
    UResGpsInit();
    addKey("gps");
  };
  /**
  Destructor */
  virtual ~UGps();
  /**
   * Initialize resource */
  void UResGpsInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "gps"; };
  /**
  Set (or remove) ressource (core pointer needed by event handling) */
//   bool setResource(UResBase * resource, bool remove);
  /**
  print status console 
  this method are used be the server core and should be present */
  void printStatus(int client) override;
  /**
   * Update position data message for subscribers */
  void updateSubscriptionData(); // override;
  /**
   * Send help about this data item */
  void sendHelp(const int client) override;
  
public:
  /**
  Open logfile for raw gps data. */
  void openLog() override;
  /**
  Run the receice loop for the GPS device.
  This call do not return until the threadStop flag is set true. */
  void run() override;
  /**
  Get the latest received sentence - may or may not be used. */
  inline const char * getLastSentance()
  { return sentence; };
  /**
  Get the latest lat-long received sentence. */
  inline const char * getLastLatLongSentance()
  { return latLong.sentence; };
  /**
  Get the latest status received sentence. */
  inline const char * getLastStatusSentance()
  { return status.sentence; };
  /**
  Get time of laset received sentence position sentence. */
  inline UTime getLastPositionTime()
  { return getTimeLocal(); };

protected:
  /**
  Save this string to the logfile after a timestamp with current time. */
  void toLog(const char * message);
  /**
  * Receive data from device
  * Returns when a newline '\n' is received, and there may be more data and possibly more messages.
  * in the received data buffer.
  * \param[in] 'buff' is the buffer place where the received data is stored,
  * \param[in] Returns when 'buffCnt' is reached - regardless of data
  * \param[in] Return if timeout ('timeoutSec' sec) has passed with no data.
  * \return the number of characters added to the buffer. If a read error
  * occured, then -1 is returned. */
  int receiveFromDevice(FILE * device,
                               char * buff, int buffCnt, const double timeoutSec);

private:
  /**
   * Resource lock for variable logfile */
//   std::mutex dataLock;
  /**
  Clear all structures in this class */
  void clear();
  /**
  \brief Function to transform from Lat-Long to UTM in a given zone.
   *
   * The algorithm is supplied by Anders Gaasedal and is also used in
   * his truck control project.
   *
   * \attention This algorithm is supplied as is. There is no guarantee that
   * is works consistently and without faults. As can be seen from the code
   * there are several unused variables, which there is no use for. This
   * does not give much confidence in the code, but no errors has been seen
   * when parsing messages.
   */
  /**
  Conversion from UTM in a zone to lat-long.
  \param [in] easting in meter relative to central median (+ 500km)
  \param [in] northing in meter from equator
  \param [in] zone of central meridian
  \param [out] latitude as calculated (in degrees)
  \param [out] longitude as calculated (in degrees)
  \return true. */
  /**
  \brief  Parser for the contents of the NMEA strings.
   *
   * Currently GPRMC, GPGGA, GPGSA, GPGSV anf PFST strings are supported.
   *
   * \param[in] *inBuf Pointer to the string to be parsed.
   * \param[in] *tod is the computer time data were received, to be used in utmHistory stack.
   * \return true on validate checksum, else the message is not parsed
  */
  bool parseNMEA(char * inBuf, UTime tod);
  /**
  Get time local time for last (lat-long) message */
  UTime getTimeLocal();
  /**
   * checks if first character is a '$' and ends with a '*xx' sequence.
   * The CR character after the checksum is replaced by a '\0'
   * \param rxBuff is the buffer to check.
   * \param rxBuffCnt is the number of bytes in buffer
   * \param pEnd will be set to just after the message (most likely point to LF
   * character after checksum) if there is a message,
   * if no message is found, then pEnd is unchanged. (pEnd may be NULL).
   * \returns true if at least one NMEA message is available in buffer */
  bool hasNmeaMessage(char * rxBuff, int rxBuffCnt, char ** pEnd);
  /**
   * Removes one NMEA message from input buffer, and if there is more than one, then
   * the start of the new message - the '$' is moved to start of buffer, and the new buffer length is
   * returned.If no new measse is found (no '$') then the buffer is returned empty.
   * \param rxBuff is the buffer to check.
   * \param rxBuffCnt is the number of bytes in buffer.
   * \param pEnd is end of massage to be discarded.
   * \returns number of bytes in new buffer */
  int advanceNmeaMessage(char * rxBuff, int rxBuffCnt, char * pEnd);

protected:
  /**
  check if this is a valid NMEA message checksum is correct.
  \param [in] the sting with the message, of max length GPSSTRINGSIZE.
  \return true if checksum match */
  bool validateNMEA(char* in_buf);
  /**
  \brief Implementation of a ASCII to HEX coverter.
   *
   * Takes both upper and lower case ascii values but has no check for
   * non HEX values.
   * \param[in] input Character input in ascii values
   * \return hex value of the parameter for further processing. */
  char ascii2hex(char input);

protected:
  static const int MAX_FILENAME_LENGTH = 1000;
  /**
  Name of logfile */
  char logGpsName[MAX_FILENAME_LENGTH];
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
  Is thread actually running */
//   bool threadRunning;
  /**
  Should thread stop - terminate */
//   bool threadStop;
  /**
  Timestamp of the letest received data */
  UTime dataRxTime;
  /**
  Number of full messages received */
  int dataRxMsgCnt;

public:
  /**
  File handle to GPS message log */
  ULogFile logGps;

private:
  /// received Lat-Long position
  UGpsLatLong latLong;
  /// set when Lat-Long is updated
  bool latLongUpdated;
  /// Received UTM position
//   UGpsUTM utm;
  /// set when utm is updated
  bool utmUpdated;
  /// Status of the GPS reception
  UGpsStatus status;
  /// set when status is updated
  bool statusUpdated;
  /** Last time data was received - including date
  Used when message holds time but no date, especially needed during replay. */
  UTime lastTime;
  /// Local computer time for last message - to be used in UTM pose history
  UTime msgTime;
  /// last message received
  char sentence[GPS_SENTENCE_MAX_LENGTH];
  /// debug log
//   ULogFile log2;
  /// device filename for GPS device
  static const int MFL = 1000;
  char devicename[MFL] = {'\0'};
  /// baudrate
  int baudrate = 9600;
  /// is device open
  bool deviceOpen = false;
  /// should device void open
  bool deviceShouldOpen = true;
};

#endif

