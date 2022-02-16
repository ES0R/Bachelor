#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Primary class, that holds all the rest
 #*
 #*   This program is free software; you can redistribute it and/or modify  *
 #*   it under the terms of the GNU General Public License as published by  *
 #*   the Free Software Foundation; either version 2 of the License, or     *
 #*   (at your option) any later version.                                   *
 #*                                                                         *
 #*   This program is distributed in the hope that it will be useful,       *
 #*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 #*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 #*   GNU General Public License for more details.                          *
 #*                                                                         *
 #*   You should have received a copy of the GNU General Public License     *
 #*   along with this program; if not, write to the                         *
 #*   Free Software Foundation, Inc.,                                       *
 #*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 #***************************************************************************/

import sys 
import os
import threading
import numpy as np
#import pyqtgraph as pg
import serial
import time
import timeit
#import socket
try:
  import configparser
except:
  import ConfigParser  
from upaint import *
from PyQt5 import QtWidgets, QtCore, QtGui

class UUsb(object):
  # USB thread
  threadRunning = False
  stopThread = False
  thread = []
  dev = serial.Serial()
  sendLock = threading.Lock()
  dataTxCnt = 0
  dataRxCnt = 0
  timeLastCnt = 0;
  notSendCnt = 0;
  parent = []
  ui = []
  main = []
  debug = []
  # color
  dtugreen = QtGui.QColor(0, 136, 53, 127)
  dtured   = QtGui.QColor(153, 0, 0, 127)
  dtugrey  = QtGui.QColor(239, 240, 241, 255)
  dtuyellow = QtGui.QColor(0xf6, 0xd0, 0x4d, 64)
  
  # methods / functions
  def __init__(self, parent):
    self.parent = parent
    self.ui = parent.ui
    print("# USB __init__ called")

  def init(self):
    # connect frame
    self.main = self.parent.main
    self.debug = self.parent.debug
    self.ui.checkBox_connect.clicked.connect(self.connectClicked)
    self.close();
    self.thread = threading.Thread(target=self.run, name="drone_gui_USB_reader")
    self.thread.start()
    
  def isOpen(self):
    return self.dev.isOpen()
  
  # open USB to Teensy
  def usbopen(self, name):
    if (not self.dev.isOpen()):
      self.dev.port = str(name)
      self.dev.timeout = 0.5
      #self.conWriteTimeout = 0.5
      #print("Trying to open:" + self.con.port)
      try:
        self.dev.open()
        self.setStatusBar("USB - opened OK.")
        self.debug.mainStatus += "USB Connected OK\n"
        self.debug.mainStatusSet = True
        #self.failCnt = 0
      except:
        self.main.setStatusBar("USB open failed :" + str(name))
        #self.failCnt += 1
        #self.ui.connect_usb.setChecked(False)
        self.debug.mainStatus += "USB Failed to " + str(name) + "\n"
        self.debug.mainStatusSet = True
    if self.dev.isOpen():
      #self.dev.flushInput()
      self.dev.flushOutput()
      setFrameColor(self.ui.frame_usb, self.dtugreen)
      # subscribe to heartbeat messages (state of flight controller)
      self.usbWrite("\nsub state 400\n")
    pass

  def close(self):
    # USB close
    if self.dev.isOpen():
      #print("stopping push S=0")
      # self.usbWrite("\nsub state 0\n")
      #self.conWrite("S=0\n")
      self.dev.close()
      self.debug.mainStatus += "Teensy is disconnected\n"
      self.debug.mainStatusSet = True
      self.main.message("USB closed")
    if self.ui.checkBox_connect.isChecked():
      setFrameColor(self.ui.frame_usb, self.dtured)
    else:
      setFrameColor(self.ui.frame_usb, self.dtugrey)
    pass

  def connectClicked(self):
    # USB connect
    if self.ui.checkBox_connect.isChecked():
      print("Trying to connect to " + self.ui.lineEdit_connect.text())
      self.usbopen(self.ui.lineEdit_connect.text())
    else:
      self.close()

  # always active read from Teensy
  def run(self):
    count = 0
    m = 0
    n = 0
    #c = '\0'
    self.threadRunning = True
    print("# USB thread running")
    got = ""
    gotraw = []
    while (not self.stopThread):
      if self.dev.isOpen():
        n = 0
        try:
          # get characters until new-line (\n)
          gotraw = self.dev.readline()
          #if len(gotraw) > 0:
            #got = gotraw.decode('ascii')
          #if len(gotraw) > 0:
            #print("got " + str(len(gotraw)) + " chars:" + str(int(gotraw[0])) + ",'"+ str(gotraw) + "'")
          n = len(gotraw)
          m = 0
        except:
          m = m + 1
          time.sleep(0.01)
          print("# Read from USB failed " + str(m) + " times")
          if m > 15:
            self.close()
        if n > 3:
          ok = True
          for c in gotraw:
            ok = ok and c < 127
          if ok:
            got = gotraw.decode('ascii')
            self.dataRxCnt += 1
            if got[0] == ';':
              sum = 0
              try:
                q = int(got[1:3])
                if q > 0:
                  for i in range(3,len(gotraw)):
                    sum = sum + gotraw[i]
                ok = sum % 99 +1 == q
              except:
                ok = False # character 1 and 2 is not numeric
              if not ok:
                print("Teensy data failed q-test:" + got)
              else:
                self.main.decodeCommand(got[3:], n-3, "u")
            else:
              print("Teensy msg do not start with ; discarded:" + got)
          else:
            print("# code has illegal chars " + str(gotraw))
        #time.sleep(0.01)
      else:
        time.sleep(0.1)
    print("# USB read thread ended")
    self.threadRunning = False

  def stop(self):
    self.close()
    if self.threadRunning:
      self.stopThread = True
      self.thread.join(2)
      print("# usb thread joined")

  ### send string to USB connection
  def usbWrite(self, s):
    self.sendLock.acquire()
    if self.dev.isOpen():
      n = len(s)
      #print("# sending " + s)
      if (n > 0):
        try:
          n = self.dev.write(s.encode())
          #print("# USB write returned " + str(n) + " bytes send")
          if (n == 0):
             raise Exception("Write error")
        except:
          self.dev.close()
          print("UMain dev.write(...) failed - closing connection")
          self.ui.statusbar.showMessage("Robot usb - connection broken")
    self.sendLock.release()
    return self.isOpen()

  def timerUpdate(self, timerCnt):
    if not self.dev.isOpen():
      if self.ui.checkBox_connect.isChecked():
        self.timeLastCnt += 1
        setFrameColor(self.ui.frame_usb, self.dtured)
        if  self.timeLastCnt > 30:
          self.timeLastCnt = 0
          self.usbopen(self.ui.lineEdit_connect.text())
        pass
      pass
      
