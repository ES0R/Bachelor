


#include "udatabase.h"
#include <ustate.h>
#include "upose.h"
#include "uaccgyro.h"
#include "ujoy.h"
#include "uoptidata.h"
#include "uteensy.h"
#include "userverport.h"
#include "ucamera.h"
#include "uaruco.h"
#include "utime.h"
#include "upublish.h"
#include "urelay.h"
#include "uunhandled.h"
#include "ugps.h"
#include "umission.h"
#include "ugamepad.h"

/**
 * should all open logfiles use fflush to ensure
 * that all data is on disk - in case of a crash.
 * */
bool ULogFile::useFlush = true;


UDataBase::UDataBase()
{ // add all data storage objects
  gamepad = new UGamepad(this);
  data[dataCnt++] = gamepad;
  state = new UState(this);
  data[dataCnt++] = state;
  pose = new UPose(this);
  data[dataCnt++] = pose;
  imu = new UAccGyro(this);
  data[dataCnt++] = imu;
//   joy = new UJoy(this);
//   data[dataCnt++] = joy;
  relay = new URelay(this);
  data[dataCnt++] = relay;
  unh = new UUnhandled(this);
  data[dataCnt++] = unh;
}

UDataBase::~UDataBase()
{
  stopAll();
}

void UDataBase::stopAll()
{
  // stop all activity
  stop();
  for (int n = 0; n < dataCnt; n++)
  {
    printf("# stopping %d, a %s\n", n, data[n]->getKey(0));
    data[n]->stop();
  }
}



void UDataBase::setup()
{ // create base data source object(s) after pure data storage objects
  //teensy = new UTeensy(this, UDataBase::Teensy, "/dev/ttyACM0");
  teensy = new UTeensy(this, UDataBase::Teensy, teensyDevice);
  //teensy = new UTeensy(this, UDataBase::Teensy, "/dev/teensy");
  data[dataCnt++] = teensy;
  bridge = new UBridge(this, bridgeIp, UDataBase::Bridge);
  data[dataCnt++] = bridge;
  opti = new UOptiData(this, -1); // not really a source of commands
  data[dataCnt++] = opti;
  clients = new UServerPort(this, servicePort, 10); // can use up to 20 connections (ids)
  data[dataCnt++] = clients;
  gps = new UGps(this, gnssDevice);
  data[dataCnt++] = gps;
  camera = new UCamera(this); // 
  data[dataCnt++] = camera;
  arucos = new ArUcoVals(camera, this, UDataBase::ArUco);
  data[dataCnt++] = arucos;
  mission = new UMission(this);
  data[dataCnt++] = mission;
  //
  // additional setup as needed
  opti->setup(argv0);
  // start some of the processing loops
  clients->start();
  teensy->start();
  gps->start();
  gamepad->start();
  //
  //start tick'ing the data elements
  start();
  //
  // usleep(100000);
  readCommandsInIniFile(false);
}

const char * UDataBase::skipToNext(const char * p1)
{ // return pointer to next parameter - after blank or '='
  while (isgraph(*p1) and *p1 != '=')
    p1++;
  while (isblank(*p1) or *p1 == '=')
    p1++;
  return p1;
}


void UDataBase::readCommandsInIniFile(bool ignoreLogFile)
{
  // open ini-file and read first lines
  // with URL and port numbers
  std::ifstream ini;
//   printf("# - opening %s\n", iniFileName);
  ini.open(iniFileName);
  if (ini.is_open())
  {
    int n = 0;
    std::string str; 
    while (std::getline(ini, str)) 
    {
      n++;
//       printf("# got %s\n", str.c_str());
      // process file ...
      if (str[0] == '%' or str[0] == '#' or str[0] == ';')
        ; // ignore as comment line in file
      else if (str.find("bridgeIP", 0) == 0)
        ; // ignore (used already)
      else if (str.find("servicePort", 0) == 0)
        ; // ignore (used already)
      else if (str.find("gnssDevice", 0) == 0)
        ; // ignore (used already)
      else if (str.find("log", 0) == 0 and ignoreLogFile)
        ; // ignore (used already)
      else
      { // decode all lines as from keyboard (client 0)
        bool used = decode(str.c_str(), 0, true);
        if (not used)
          printf("# ini command line %d not used: %s\n", n, str.c_str());
      }
    }
    ini.close();
  }
  else
    printf("# No ini-file found\n");
}

void UDataBase::printStatus(int client)
{ // status is clear text
  for (int i = 0; i < dataCnt; i++)
  { // all data items should have a status
    data[i]->printStatus(client);
  }
}

bool UDataBase::decode(const char* message, int source, bool msgIsOK)
{
  if (*message != '\0')
  {
    bool used = false;
    char key1[UPublish::MAX_KEY_LENGTH];
    char key2[UPublish::MAX_KEY_LENGTH] = "";
    const char * p1 = message;
    getKeyString(&p1, key1);
    bool hasKey2 = getKeyString(&p1, key2);
    // p1 is now moved to after key2
    while (isspace(*p1))
      p1++; // 
    // debug
    //printf("# UDataBase::decode k1:%s, key2:%s (%s)\n", key1, key2, message);
    //
    for (int i = 0; i < dataCnt; i++)
    { // ask everyone in the list for use of this data
      used = data[i]->decode(message, source, key1, msgIsOK);
      if (used)
        break;
    }
    if (not used)
    { // there is other possibilities
      // printf("# used = %d (%s, %s (%lu))\n", used, key1, key2, strlen(key2));
      if (strcmp(key1, "help") == 0)
      { // help text (keys) on all or specific data item
        if (hasKey2)
        { // help for a specific key
          p1 = key2;
        }
        else
          // help on all topics
          p1 = nullptr;
        for (int i =0; i < dataCnt; i++)
        { // send requested help info
          used = data[i]->sendHelp(source, p1);
          if (used)
            break;
        }
        if (not used)
          sendHelp(source, p1);
        used = true;
      }
      else if (strcmp(key1, "sub") == 0)
      { // data subscription
        if (hasKey2)
        { // find data item
          for (int i =0; i < dataCnt; i++)
          {
            used = data[i]->subscribeFromThis(p1, key2, source);
            //printf("#UDB:: decode for %d/%d ended, key %s, found %d (source=%d, key=%s)\n", i, dataCnt, key2, used, source, data[i]->getKey(0));
            if (used)
              break;
          }
        }
      }
      else if (strcmp(key1, "status") == 0)
      { // status in clear text as comments
        for (int i =0; i < dataCnt; i++)
        {
          if (not hasKey2)
            // print all
            data[i]->printStatus(source);
          else if (data[i]->hasKey(key2))
          { // specific item
            data[i]->printStatus(source);
            break;
          }
        }
      }
      else if (strcmp(key1, "log") == 0)
      { // open or close logfiles
        if (hasKey2)
        { // find data item from key2 (p1 is further params, e.g. close)
//           printf("# UDatabase: log, key2=%s, param=%s\n", key2, p1);
          for (int i =0; i < dataCnt; i++)
          {
            used = data[i]->logOpenClose(p1, key2);
            if (used)
              break;
          }
          if (not used)
            printf("# Open log failed, no data item found for '%s'\n", key2);
        }
      }
    }
    if (not used)
    { // not (yet) supported message - 
//       printf("unhandled message: '%s'\n", message);
      unh->decodeUnhandled(message, source);
    }
  }
  return true;
}
  
bool UDataBase::sendReply(const char* message, int toClient)
{ // send reply to this client
  bool send = false;
  for (int n = 0; n < dataCnt; n++)
  {
    if (data[n]->sendReplyFromHere(message, toClient))
    {
      send = true;
      break;
    }
  }
  if (toClient == 0)
  { // to console
    printf("%s", message);
    send = true;
  } 
  return send;
}


bool UDataBase::sendToDataSource(const char* message, int clientID)
{ // send to robot
  bool isOK;
  char key[UPublish::MAX_KEY_LENGTH];
  const char * p1 = message;
  getKeyString(&p1, key, UPublish::MAX_KEY_LENGTH);
  isOK = teensy->decode(message, -1, key, false);
  if (not isOK and bridge != nullptr)
    isOK = bridge->send(message);
  if (not isOK)
    printf("# message to client %d is not send %s", clientID, message);
  return true;
}

void UDataBase::requestData(bool toTeensy)
{ // request data from source
  for (int i = 0; i < dataCnt; i++)
    data[i]->subscribeFromHW(toTeensy);
}


void UDataBase::dataSourceOpened(bool toTeensy)
{ // request subscriptions
  if (toTeensy)
  { // send the not-so-important message as the first
    // as there may be something in the rx buffer on the Teensy.
    teensy->send("alive\n");
    if (true)
      // should not be needed
      readCommandsInIniFile(true);
  }
  // requestData(toTeensy);
  //printf("# UDataBase::dataSourceOpened: - no subscribe requested here - use ini-file\n");
}

void UDataBase::openAllLogs()
{ // open all logs
  for (int i = 0; i < dataCnt; i++)
    data[i]->openLog();
}

void UDataBase::closeAllLogs()
{ // open all logs
  for (int i = 0; i < dataCnt; i++)
    data[i]->closeLog();
}

void UDataBase::run()
{
  while (not th1stop)
  {
    float dt = tickTime.getTimePassed();
    if ( dt >  tickInterval)
    {
      for (int i = 0; i < dataCnt; i++)
        data[i]->tick();
      tickTime += tickInterval;
    }
    // wait 2ms
    usleep(2000);
  }
}


bool UDataBase::getKeyString(const char** pm, char* key, int keyLen)
{
  const char * p1 = *pm;
  bool found = false;
  // find start of key
  while (isblank(*p1))
    p1++;
  const char * p2 = p1;
  // find end of key
  while (isgraph(*p2))
    p2++;
  int n = p2 - p1;
  if (n > 0 and n < keyLen and key!= nullptr)
  { // copy key and terminate
    strncpy(key, p1, n);
    key[n] = '\0';
    found = true;
    // advance pointer to after key
    *pm = p2;
  }
  return found;
}

bool UDataBase::sendHelp(int client, const char* key)
{
  bool found = false;
  if (key != nullptr)
  {
    found = strstr("help,sub,log",key) != nullptr;
  }
  if (found or key == nullptr)
  {
    sendReply("# ------- Service ----------\r\n", client);
    sendReply("# help [key]       Get on-line command help, optionally for a specific keyword.\r\n", client);
    sendReply("# log key [close]  Open (or close) logfile for the data element with this keyword.\r\n", client);
    sendReply("# sub key N [P]    Subscribe to data element, N=1 for just once, N=0 stop, N=-1 continued.\r\n", client);
    sendReply("#                  P = priority, default is 1 second, else time in units of 10ms (if data is updated) .\r\n", client);
    sendReply("# status [key]     Send status for all or data item with this key.\r\n", client);
  }
  return found;
}

bool UDataBase::readCommandLineParameters(int argc, char ** argv, bool * asDaemon)
{
  // are there mission parameters
  bool isHelp = false;
  snprintf(iniFileName, MIL, "%s.ini", argv[0]);
  argv0 = argv[0];
  // default values, if there is nothing in the ini-file
  strncpy(bridgeIp, "127.0.0.1:24001", MIL);
  strncpy(servicePort, "24002", MIL);
  strncpy(teensyDevice, "/dev/ttyACM0", MIL);
  strncpy(gnssDevice, "/dev/ttyACM7", MIL);
  for (int i = 1; i < argc; i++)
  {
    if (isHelp)
      break;
    char * p1 = argv[i];
    char * p2;
    while (*p1 == '-')
      p1++; // allow both "-h" and "--help" notation
    switch (*p1)
    {
      case 'a':
        *asDaemon = true;
        break;
      case 'h':
        isHelp = true;
        break;
      case 'b':
        // skip the n, but use the rest of this parameter
        p2 = &p1[1];
        while ((p2[0] <= ' ' and p2[0] > '\0') or p2[0] == '=')
          p2++;
        printf("# bridgeIP (%u) %s\n", (unsigned int)strlen(p2), p2);
        if (strlen(p2) == 0)
          p2 = argv[++i];
        strncpy(bridgeIp, p2, MIL);
        printf("b (bridge) URL '%s'\n", bridgeIp);
        break;
      case 's':
        // skip the s, but use the rest of this parameter
        p2 = &p1[1];
        while ((p2[0] <= ' ' and p2[0] > '\0') or p2[0] == '=')
          p2++;
        if (strlen(p2) == 0)
          p2 = argv[++i];
        strncpy(servicePort, p2, MIL);
        printf("s (service port) '%s'\n", servicePort);
        break;
      case 'i':
        // skip the i, but use the rest of this parameter
        p2 = &p1[1];
        while ((p2[0] <= ' ' and p2[0] > '\0') or p2[0] == '=')
          p2++;
        if (strlen(p2) == 0)
          p2 = argv[++i];
        strncpy(iniFileName, p2, MIL);
        printf("Using inifile '%s'\n", iniFileName);
        break;
      case 't':
        // skip the i, but use the rest of this parameter
        p2 = &p1[1];
        while ((p2[0] <= ' ' and p2[0] > '\0') or p2[0] == '=')
          p2++;
        if (strlen(p2) == 0)
          p2 = argv[++i];
        strncpy(teensyDevice, p2, MIL);
        printf("Teensy device is '%s'\n", teensyDevice);
        break;
      case 'g':
        // skip the i, but use the rest of this parameter
        p2 = &p1[1];
        while ((p2[0] <= ' ' and p2[0] > '\0') or p2[0] == '=')
          p2++;
        if (strlen(p2) == 0)
          p2 = argv[++i];
        strncpy(gnssDevice, p2, MIL);
        printf("GNSS device is '%s'\n", gnssDevice);
        break;
      default:
        break;
    }
  }
  if (not isHelp)
  { // open ini-file and read first lines
    // with URL and port numbers
    std::ifstream ini;
  //     printf("# - opening %s\n", iniFileName);
    ini.open(iniFileName);
    if (ini.is_open())
    {
      std::string str; 
      while (std::getline(ini, str)) 
      {
        // printf("# got %s\n", str.c_str());
        // process string ...
        // key [=] firstParameter
        // p1 points to first parameter after keyword
        const char * p1 = UDataBase::skipToNext(str.c_str());
        if (str[0] == '%' or str[0] == '#')
          ; // ignore
        else if (str.find("bridgeIP", 0) == 0)
          strncpy(bridgeIp, p1, MIL);
        else if (str.find("servicePort", 0) == 0)
          strncpy(servicePort, p1, MIL);
        else if (str.find("teensyDevice", 0) == 0)
          strncpy(teensyDevice, p1, MIL);
        else if (str.find("gnssDevice", 0) == 0)
          strncpy(gnssDevice, p1, MIL);
      }
      ini.close();
    }
    else
      printf("# No ini-file found\n");
  }
  return isHelp;
}
