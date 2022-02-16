
/***************************************************************************
*   Copyright (C) 2016 by DTU (Christian Andersen)                        *
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

#include <fstream>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <readline/readline.h>
#include <readline/history.h>
// #include <filesystem> // require C++17 to work
// #define PI_CAM
#ifdef PI_CAM
#include <raspicam/raspicam.h>
#endif
#include <ubridge.h>
#include <udatabase.h>

using namespace std;



void printHelp(char * name)
{ // show help
  printf("------------\r\n");
  printf("Usage: %s [-ha] [-sitbg parameter] \r\n", name);
  printf(" -s port     Service port on this host (default 24002)\r\n");
  printf(" -i file     Initialization filename (default %s.ini)\r\n", name);
  printf(" -t device   Teensy device (default /dev/ttyACM0)\r\n");
  printf(" -g device   GNSS device (default /dev/ttyACM7)\r\n");
  printf(" -b IP:port  Onboard server with HW access (default is 127.0.0.1:24001) (optional)\r\n");
  printf(" -a          Run as daemon (no keyboard)\r\n");
  printf(" -h          This help text\r\n");
  printf(" NB! if there is a %s.ini file then this may override command line parameters\r\n", name);
  printf("------------\r\n");
}




/**
 * Get a line from keyboard, but return also, when 
 * bission is finished or connection to bridge is
 * broken.
 * \param s is a buffer string, where the line is returned.
 * \param MSL is the length of the string buffer
 * \param fin  is a pointer to a finished flag - should return when flag is set.
 * \returns number of characters in line (0 if empty line or finished or connection is false.
 * */
int getLineFromKeyboard(char * s, const int MSL)
{
  int m = 0;
#define USE_READLINE_LIB
#ifdef USE_READLINE_LIB
  // readline allocates a string and returns a pointer to it
  char * line = readline(">> ");
  if (line != nullptr)
  {
    m = strlen(line);
    if (m > 0)
    { // add to history
      add_history(line);
      // copy to buffer
      snprintf(s, MSL, "%s\n", line);
    }
    free(line);
  }
  return m;
#else
  char * p1 = s;
  *p1 = '\0';
  int loop = 0;
  while (((p1 - s) < MSL) and loop < 50)
  {
    int n = read(stdin->_fileno, p1, 1);
    if (*p1 == '\n')
    { // terminate and stop on newline
      *++p1 = '\0';
      break;
    }
    if (n == 1)
    {
      p1++;
      loop = 0;
    }
    else
    {
      usleep(1000);
      loop++;
    }
  }
  if (loop == 50)
    m = -1;
  else
    m = strlen(s);
  return m;
#endif
}


/////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////
/**
 * main function.
 * creates all modules - bridge, camera and mission functions,
 * and then allow console functions.
 * */

int main ( int argc,char **argv ) 
{
  const int MSL = 300;
//   char bridgeIp[MSL] = "127.0.0.1:24001"; // default connection IP to bridge
//   char servicePort[MSL] = "24002"; // default port created
//   char iniFileName[MSL] = "mission.ini"; // default port created
  char s[MSL];
  bool asDaemon = false;
  printf("Starting ...\n");
  // create directory for logging
  std::system("mkdir -p log");
  //
  UDataBase * db = new UDataBase();
  bool isHelp = db->readCommandLineParameters(argc, argv, &asDaemon);
  if (isHelp)
    printHelp(argv[0]);
  else
  { // create connection to Regbot board through bridge 
    db->setup();
    //
    // logs
//     db->openLogs();
    // the rest is just to allow some command line input
    //
    sleep(4); // wait a bit to allow connection to bridge
    printf("Press h for help (q for quit)\n");
    printf(">> "); // command prompt
    fflush(stdout); // make sure command prompt gets displayed
    // make stdin (keyboard) non-blocking
    int flags = fcntl(stdin->_fileno, F_GETFL, 0); 
    fcntl(stdin->_fileno, F_SETFL, flags | O_NONBLOCK);
    bool stop = false;
    //
    if (asDaemon)
    { // stops this process, 
      // but a copy continues without keyboard and console
      // daemon(1,0);
    }
    //
    while (not stop)
    { // Just wait for quit
      if (not asDaemon)
      {
        int n = getLineFromKeyboard(s, MSL);
        const char * p1 = s;
        if (n > 0)
        {
          if (isalnum(p1[0]))
          {
            //printf("# got '%s' n=%d\n", s, n);
            switch (p1[0])
            {
              case 'q':
                stop = true;
                break;
              case 's':
                printf("# Status:\n");
                db->printStatus(0);
                break;
              case 'h':
                printf("# mission commands\n");
                printf("#    0    send rest of the line as incoming command (from client 0 (console))\n");
                printf("#         e.g. '0 help' for more help,\n");
                printf("#         or '0 log hbt' for data log of flight state (heartbeat),\n");
                printf("#         or '0 sub acc -1 100' subscribe to acc (-1=continued, 0=stop) each 100 x 10ms.\n");
                printf("#    q    Quit now\n");
                printf("#    s    Status (all)\n");
                printf("#\n");
                break;
              case '0':
                p1++;
                while (isblank(*p1))
                  p1++;
                //printf("# keyboard command to %c: %s", s[0], p1); 
                db->decode(p1, 0, true);
                break;
              default:
                printf("# unknown command '%s'\n", s);
                break;
            }
          }
          if (isdigit(p1[0]) and p1[0] != '0')
          { // to be send to a client with this number
            int dest = strtol(p1, (char **)&p1, 10);
            while (isblank(*p1))
              p1++;
            db->sendReply(p1, dest);
          }
          // wait long enough for function to finish.
          usleep(100000);
//           printf(">> ");
//           flush(cout);
        }
//         else if (n == 0)
//         {
//           printf(">> ");
//           flush(cout);
//         }
      }
      usleep(20000);
//       printf("# empty line\n");
    }
    printf("Main ended\n");
    db->stopAll();
  }
}
