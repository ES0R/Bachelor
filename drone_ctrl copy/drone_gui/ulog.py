#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2021 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for (fast) data logging
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
import socket
import time
#from time import sleep
import timeit
try:
  import configparser
except:
  import ConfigParser  
from PyQt5 import QtWidgets, QtCore


class ULog(object):
  # class variables
  logdata = ""
  logDataUpd = True
  lastFileName = "log.txt"
  lastFileNameSet = True
  loglineCnt = 0
  ui = []
  parent = []
  gotDataLine = 0

  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui

  def init(self):
    self.main = self.parent.main
    self.data = self.parent.motor
    self.ui.but_start_log_imu.clicked.connect(self.startLogImu)
    self.ui.but_start_log_motor.clicked.connect(self.startLogEsc)
    self.ui.but_get_log.clicked.connect(self.getLog)
    self.ui.pushButton_log_save_as.clicked.connect(self.saveLogAs)
    self.ui.pushButton_log_save.clicked.connect(self.saveLogToFile)
    self.ui.pushButton_log_clear.clicked.connect(self.clearLog)
    self.ui.pushButton_log_start_seq.clicked.connect(self.startSeq)
    self.ui.but_start_log_pose.clicked.connect(self.startLogPose)
    self.ui.but_start_log_pitch.clicked.connect(self.startLogPitch)
    self.ui.but_start_log_timing.clicked.connect(self.startLogTiming)
    self.ui.but_start_log_USB.clicked.connect(self.startLogUSB)
    self.ui.but_start_log_stop.clicked.connect(self.stopLog)
    #
    self.ui.but_start_log_ctrl.clicked.connect(self.startLogCtrl)
    self.ui.comboBox_log_ctrl.addItems(["vroll", "roll", "vpitch", "pitch", "vyaw", "height"])
    pass

  def timerUpdate(self, timerCnt):
    if self.ui.tabWidget.currentIndex() == self.ui.tabWidget.indexOf(self.ui.tab_log):
      if timerCnt % 10 == 0:
        if self.logDataUpd:
          self.ui.text_log.setPlainText(self.logdata)
          self.logDataUpd = False
        pass
      if self.lastFileNameSet:
        self.lastFileNameSet = False
        self.ui.lineEdit_log_filename.setText(str(self.lastFileName))
    pass 
  
  def startLogImu(self):
    self.main.devWrite("log imu " + str(self.ui.num_log_interval.value()) + "\n", True)
    pass
  def startLogEsc(self):
    self.main.devWrite("log motor " + str(self.ui.num_log_interval.value()) + "\n", True)
    pass
  def startLogCtrl(self):
    self.main.devWrite("log " + self.ui.comboBox_log_ctrl.currentText() + " " + str(self.ui.num_log_interval.value()) + "\n", True)
    pass
  def startLogPose(self):
    self.main.devWrite("log pose " + str(self.ui.num_log_interval.value()) + "\n", True)
    pass
  def startLogPitch(self):
    self.main.devWrite("log pitch " + str(self.ui.num_log_interval.value()) + "\n", True)
    pass
  def startLogTiming(self):
    self.main.devWrite("log time " + str(self.ui.num_log_interval.value()) + "\n", True)
    pass
  def startLogUSB(self):
    self.main.devWrite("log usb " + str(self.ui.num_log_interval.value()) + "\n", True)
    pass
  def stopLog(self):
    self.main.devWrite("log stop\n", True)
    pass
  
  def getLog(self):
    self.logdata = ""
    self.main.devWrite("log get\n", True)
    self.loglineCnt = 0;
    self.logDataUpd = True
    #self.ui.text_log.clear();
    #self.main.message("# getting log ...")
    pass
  
  def decode(self, gg, got):
    isOK = True
    if got[0] == '%':
      self.logdata += got
      self.gotDataLine = 0
    elif got[0].isdigit():
      self.logdata += got
      # tell sender we use flow control
      self.gotDataLine += 1
      if self.gotDataLine == 1:
        # tell sender we got first and send message for every 10 lines
        self.main.devWrite("log got 1 10\n", True)
      elif self.gotDataLine % 10 == 1:
        self.main.devWrite("log got " + str(self.gotDataLine) + " 10\n", True)
    elif gg[0] == "logdata":
      self.main.message("# log load finished")
      #self.ui.statusbar.showMessage("got all log data " + str(self.loglineCnt) + " lines", 3000)
    elif gg[0] == "logfull":
      self.main.message("# log filled")
      print("Got logfull - and " + str(self.gotDataLine) + " data lines")
    else:
      isOK = False
    if isOK:
      self.logDataUpd = True
      self.loglineCnt += 1
    return isOK

  def saveLogToFile(self):
      fn = self.ui.lineEdit_log_filename.text()
      self.saveLogToThisFile(fn)

  def clearLog(self):
      self.logdata = ""
      self.logDataUpd = True
      self.loglineCnt = 0;

  def saveLogToThisFile(self, fn):
    print("Saving to logfile " + fn)
    try:
      f = open(fn, "w");
      f.write(self.logdata)
      f.close()
      self.ui.statusbar.showMessage("Save log as " + str(fn), 3000)
    except:
      self.ui.statusbar.showMessage("Failed to save log as " + str(fn) + " !", 3000)

  def saveLogAs(self):
    # QString QFileDialog::
    # getSaveFileName(QWidget *parent = nullptr, 
    #                 const QString &caption = QString(), 
    #                 const QString &dir = QString(), 
    #                 const QString &filter = QString(), 
    #                 QString *selectedFilter = nullptr, 
    #                 QFileDialog::Options options = Options()
    #                 )
    name = QtWidgets.QFileDialog.getSaveFileName(self.parent, 'Save File',self.lastFileName, 'Text files (*.txt)')
    #lfn = QtGui.QFileDialog().getSaveFileName(self.parent,'Save log to', self.lastFileName, 'log (*.txt)')
    #lfn = QtGui.QFileDialog.getSaveFileName(self.parent,'Save log to', self.lastFileName, 'log (*.txt)')
    if (name is not None and len(name[0]) > 0):
      self.saveLogToThisFile(str(name[0]))
      self.lastFileName = name[0]
      print(str(name[0]))
      self.lastFileNameSet = True

  def startSeq(self):
    print("starting sequence")
    self.main.devWrite("seq {} {} {} {}\n".format(
      self.ui.num_log_interval_2.value(), 
      self.ui.num_log_steps.value(),
      self.ui.spinBox_esc_PWM_1.value(),
      self.ui.num_log_interval.value()), True)
    pass
