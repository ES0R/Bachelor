#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2021 by DTU                             *
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
#import numpy as np
#import pyqtgraph as pg
import serial
import time
import timeit
import socket
try:
  import configparser
except:
  import ConfigParser  
from upaint import *
from PyQt5 import QtWidgets, QtCore, QtGui

class UMain(object):
  # class variables
  timerCnt = 0
  messageStr = ""
  messageSet = True
  version = "0.0"
  versionDate = 'noname'
  data = []
  debug = []
  log = []
  acc = []
  pose = []
  gyro = []
  mag = []
  control = []
  motor = []
  alt = []
  rc = []
  ref = []
  usbio = []
  wifi = []
  mixer = []
  teensyTime = -0.1
  deviceID = -1
  revision = -1
  batteryVoltage = -1
  armState = 1
  flightState = 0
  bypassRC = 0
  usbCtrl = False
  stateNew = False
  statusBarTime = time.time()
  decodeLock = threading.Lock()
  notSendCnt = 0
  wifiOn = False
  bridge = False
  dronename = "unnamed"
  dronenameSet = False
  
  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui

  def init(self):
    self.motor = self.parent.motor
    self.debug = self.parent.debug
    self.log = self.parent.log
    #self.close()
    self.ui.checkBox_connect.setChecked(True)
    self.ui.tabWidget.setCurrentIndex(2)
    self.acc = self.parent.acc
    self.pose = self.parent.pose
    self.gyro = self.parent.gyro
    self.mag = self.parent.mag
    self.control = self.parent.control
    self.alt = self.parent.alt
    self.rc = self.parent.rc
    self.control = self.parent.control
    self.ref = self.parent.ref
    self.usbio = self.parent.usbio
    self.wifi = self.parent.wifi
    self.mixer = self.parent.mixer
    # button actions
    self.ui.but_esc_arm.clicked.connect(self.arm)
    self.ui.but_esc_stop.clicked.connect(self.emergency_stop)
    self.ui.pushButton_message_clear.clicked.connect(self.messageClear)
    self.ui.pushButton_save_to_flash.clicked.connect(self.saveToFlash)
    
  def message(self, s):
    self.messageStr += s + "\n"
    self.messageSet = True
    #self.ui.text_message.setText(s)

  def messageClear(self):
    self.messageStr = ""
    self.messageSet = True

  def terminate(self):
    # stop connection
    # print("# Sending emergency stop")
    # self.emergency_stop()
    self.usbio.stop()
    self.wifi.stop()
    print("# all finished OK")
    

  def decodeCommand(self, got, n, w):
    self.decodeLock.acquire()
    pre = "(R" + w + ") "
    if n > 0:
      #print(got)
      if got[0] == '#' and self.ui.checkBox_debug_show_all_hash.isChecked():
        self.debug.mainStatus += pre + got
        self.debug.mainStatusSet = True
    if n > 3 and got[0] != '#':
      isOK = False
      if len(got) > 0:
        gg = got.split()
        if gg[0] != "hbt" and self.ui.checkBox_debug_show_all_rx.isChecked():
          self.debug.mainStatus += pre + got
          self.debug.mainStatusSet = True
        try:
          if gg[0] == "hbt":
            if self.ui.checkBox_debug_show_hbt.isChecked():
              self.debug.mainStatus += pre + got
              self.debug.mainStatusSet = True
            #/** format
            #* hbt 1 : time in seconds, updated every sample time
            #*     2 : device ID (probably 1)
            #*     3 : software revision number - from SVN
            #*     4 : Battery voltage (from AD converter)
            #*     5 : arm state =0=init,1=disarmed,2=armed,3=fail
            #*     6 : flight state: 0=on ground, 1=starting, 2=flying, 3=landing
            #*     7 : Bypass RC safety (full auto, no pilot)
            #*     8 : ref input (setpoint) over USB (auto mode)
            #* */
            if len(gg) > 5:
              self.teensyTime = float(gg[1])
              self.deviceID = int(gg[2])
              self.revision = int(gg[3])
              self.batteryVoltage = float(gg[4])
              self.armState = int(gg[5])
              self.flightState = int(gg[6])
              self.bypassRC = int(gg[7])
              self.usbCtrl = int(gg[8])
              self.stateNew = True
              pass
            pass
          elif gg[0] == "ver":
            self.version = '1.' + gg[3]
            self.versionDate = gg[4]
          elif gg[0] == "dname":
            self.dronename = gg[1]
            self.dronenameSet = True
          elif self.log.decode(gg, got):
            pass
          elif self.motor.decode(gg):
            pass
          elif self.acc.decode(gg):
            pass
          elif self.pose.decode(gg):
            pass
          elif self.gyro.decode(gg):
            pass
          elif self.mag.decode(gg):
            pass
          elif self.control.decode(gg):
            pass
          elif self.alt.decode(gg):
            pass
          elif self.rc.decode(gg):
            pass
          elif self.ref.decode(gg):
            pass
          elif self.mixer.decode(gg):
            pass
          elif gg[0] == "bridge":
            self.bridge = True
          elif gg[0] == "message":
            self.message(got[8:-1])
          else:
            print("# " + pre + "main.py::decodeCommand: noone decoded:" + got)
        except:
          print("# " + pre + "decoded failed for: " + got)
    self.decodeLock.release()
    pass
  
  def devWrite(self, s, addPreKey = False):
    isSend = False
    isWiFiSend = False
    if self.usbio.isOpen() or self.wifi.isOpen():
      # pre-key is for bridge only
      if (self.usbio.isOpen()):
        #print("# about to send to USB : " + s)
        isSend = self.usbio.usbWrite(s)
        if (isSend):
          self.usbio.dataTxCnt += 1 #len(s)
          if self.ui.checkBox_debug_show_all_tx.isChecked():
            self.debug.mainStatus += "(Tu) " + str(s)
            self.debug.mainStatusSet = True
      if self.wifi.isOpen():
        if addPreKey:
          pre = "teensy "
        else:
          pre = ""
        isWiFiSend = self.wifi.wifiWrite(pre + s)
        #print("# sending >" + pre + s +"<, as " + str(addPreKey))
        #isWiFiSend = self.wifi.isOpen()
        if (isWiFiSend):
          self.wifi.txCnt += 1 #len(s)
          if self.ui.checkBox_debug_show_all_tx.isChecked():
            self.debug.mainStatus += "(Tw) " + str(pre) + str(s)
            self.debug.mainStatusSet = True
      #self.sendLock.release();

    if not (isSend or isWiFiSend):
      if self.notSendCnt < 5:
        self.debug.mainStatus += "not connected, could not send: " + str(s)
        self.debug.mainStatusSet = True
        self.notSendCnt += 1
    else:
      self.notSendCnt = 0
    pass

  def isConnected(self):
    return self.usbio.isOpen() or self.wifi.isOpen()

  def timerUpdate(self):
    #print("uregbot::timerUpdate(self):\n")
    self.timerCnt += 1
    #self.timeLastCnt += 1;
    #self.wifiTimeLastCnt += 1
    self.debug.timerUpdate(self.timerCnt)
    self.motor.timerUpdate(self.timerCnt)
    self.log.timerUpdate(self.timerCnt)
    self.acc.timerUpdate(self.timerCnt)
    self.gyro.timerUpdate(self.timerCnt)
    self.alt.timerUpdate(self.timerCnt)
    self.pose.timerUpdate(self.timerCnt)
    self.rc.timerUpdate(self.timerCnt)
    self.mag.timerUpdate(self.timerCnt)
    self.control.timerUpdate(self.timerCnt)
    self.ref.timerUpdate(self.timerCnt)
    self.usbio.timerUpdate(self.timerCnt)
    self.wifi.timerUpdate(self.timerCnt)
    self.mixer.timerUpdate(self.timerCnt)
    #if self.timerCnt % 20 == 0:
      #self.ui.label_timer_cnt.setText("timer loop " + str(self.timerCnt))
      #pass
      
    if self.wifi.isOpen() and not self.wifiOn:
      self.wifiOn = True
      # subscribe to general messages
      self.wifi.wifiWrite("sub hbt -1 30\n")
      self.wifi.wifiWrite("sub unhandled -1 0\n")
      self.wifi.wifiWrite("sub # -1 0\n")
    if not self.wifi.isOpen() and self.wifiOn:
      self.wifiOn = False
    if self.usbio.isOpen() or self.wifi.isOpen():
      if self.timerCnt % 400 == 266:
        self.devWrite("version\n", True) # system status
      if self.timerCnt % 400 == 1:
        self.devWrite("id\n", True) # get name
      if self.timerCnt % 400 == 133:
        self.devWrite("alive\n", True) # get name
    if self.messageSet:
      self.ui.text_message.setText(self.messageStr)
      self.ui.text_message.verticalScrollBar().setValue(self.ui.text_message.verticalScrollBar().maximum())
      self.messageSet = False
    if self.dronenameSet:
      self.ui.label_dronename.setText(self.dronename + " " + str(self.deviceID))
    pass
    # HTML text formatting: <html><head/><body><p>State:<span style=" font-size:14pt;"> init</span></p></body></html>
    if self.stateNew:
      if self.armState == 2:
        # Armed
        if self.usbCtrl:
          sa = ' (Auto)'
        else:
          sa = ' (RC)'
      else:
        sa = ''
      # ARM state
      if self.armState == 0:
        ss = 'Init'
        rgb = '#FC7634;'
      elif self.armState == 1:
        ss = 'Disarmed'
        rgb = '#2f3ee9;'
      elif self.armState == 2:
        ss = 'Armed'
        rgb = '#990000;'
      elif self.armState >= 3:
        ss = 'Fail'
        rgb = '#77238e;'
      else:
        ss = 'ERROR'
        rgb = '#FF0000;'
      # set label 
      self.ui.label_main_state.setText('<html><head/><body><p>State: <span style=" font-weight:600; font-size:14pt; color:' + 
                                       rgb + '">' + 
                                       ss + sa +
                                       '</span></p></body></html>')
      # flight state
      if self.flightState == 0:
        ss = 'On ground'
        rgb = '#2f3ee9;'
      elif self.flightState == 1:
        ss = 'Starting'
        rgb = '#2f9ee9;'
      elif self.flightState == 2:
        ss = 'In flight'
        rgb = '#990000;'
      elif self.flightState == 3:
        ss = 'Landing'
        rgb = '#494949;'
      else:
        ss = 'ERROR'
        rgb = '#FF0000;'
      # set label
      self.ui.label_main_mode.setText('<html><head/><body><p><span style=" font-weight:600; font-size:12pt; color:' + rgb + '">' + ss + '</span></p></body></html>')
      # other status labels
      self.ui.label_main_batt.setText('Battery: ' + str(self.batteryVoltage) + ' V')
      self.ui.label_main_time.setText('devTime: {:.1f} sec'.format(self.teensyTime))
      self.stateNew = False
    if time.time() - self.statusBarTime > 10:
      self.setStatusBar("")
      
  def arm(self):
    self.devWrite("arm 1\n", True)
    pass

  def emergency_stop(self):
    self.debug.mainStatus +="Emergency stop\n"
    self.debug.mainStatusSet = True
    self.devWrite("esc 0 0 0\n", True)
    self.devWrite("esc 1 0 0\n", True)
    self.devWrite("esc 2 0 0\n", True)
    self.devWrite("esc 3 0 0\n", True)
    self.devWrite("esc 4 0 0\n", True)
    self.devWrite("esc 5 0 0\n", True)
    self.devWrite("esc 6 0 0\n", True)
    self.devWrite("esc 7 0 0\n", True)
    self.devWrite("stop\n", True)
    self.ui.statusbar.showMessage("All motor stop")
    pass

  def saveToFlash(self):
    self.devWrite("eew\n", True)

  def setStatusBar(self, message):
    self.ui.statusbar.showMessage(message, 3000)
    self.statusBarTime = time.time()
