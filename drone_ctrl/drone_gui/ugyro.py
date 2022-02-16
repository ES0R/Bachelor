#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for gyro calibration and monitoring
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
import numpy as np
import time
import pyqtgraph as pg
import timeit
from PyQt5 import QtWidgets, QtCore


class UGyro(object):
  # class variables
  parent = []
  ui = []
  main = []
  gyro = [0,0,0]
  offs = [0,0,0]
  offsNew = True
  data = np.zeros((3,100))
  dataNew = True
  dataIdx = 0
  dataCg = []
  dataPw = []
  hasFocus = False


  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui
    self.main = self.parent.main
    # plot widget
    self.dataPw = pg.PlotWidget(name='gyro',title='Gyro') 
    self.dataPw.setLabel('left','turnrate','deg/s')
    self.dataPw.addLegend()
    self.ui.horizontalLayout_gyro.addWidget(self.dataPw)
    self.dataCg = [self.dataPw.plot(pen='r',name='x'), self.dataPw.plot(pen='c',name='y'), self.dataPw.plot(pen='g',name='z')]
    self.dataCg[0].setData(self.data[0])
    self.dataCg[1].setData(self.data[1])
    self.dataCg[2].setData(self.data[2])

  def init(self):
    self.ui.pushButton_gyro_cal_clear.clicked.connect(self.sendClearOffset)
    self.ui.pushButton_gyro_calibrate.clicked.connect(self.startCalibrate)
    pass

  def timerUpdate(self, timerCnt):
    if self.offsNew:
      self.ui.doubleSpinBox_gyro_roll_offset.setValue(self.offs[0])
      self.ui.doubleSpinBox_gyro_pitch_offset.setValue(self.offs[1])
      self.ui.doubleSpinBox_gyro_yaw_offset.setValue(self.offs[2])
      self.offsNew = False
    if self.dataNew:
      self.ui.doubleSpinBox_gyro_roll.setValue(self.gyro[0])
      self.ui.doubleSpinBox_gyro_pitch.setValue(self.gyro[1])
      self.ui.doubleSpinBox_gyro_yaw.setValue(self.gyro[2])
      self.dataCg[0].setData(self.data[0])
      self.dataCg[1].setData(self.data[1])
      self.dataCg[2].setData(self.data[2])
      self.dataNew = False
    if not self.main.isConnected():
      # act as leaving the page
      self.hasFocus = False
    thisTab = self.ui.tabWidget.indexOf(self.ui.tab_gyro)
    if self.hasFocus and self.ui.tabWidget.currentIndex() != thisTab:
      # just leaving this tab
      self.hasFocus = False
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe off
        self.main.devWrite("sub gyro 0\n") # data
      pass
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == thisTab:
      # just entering this tab
      self.hasFocus = True
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe
        self.main.devWrite("sub gyro -1 20\n") # data
        pass
    if self.hasFocus:
      if timerCnt % 3 == 2:
        self.main.devWrite("imu g\n", True) # imu actual data
      if timerCnt % 30 == 1:
        self.main.devWrite("imucal g\n", True) # imu calibration offsets
    pass 
  
  def sendClearOffset(self):
    self.main.devWrite("imuzero g\n", True)

  def startCalibrate(self):
    self.main.devWrite("offsetcal g\n", True)
    print("# send offsetcal g")
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "gyro":
      self.gyro[0] = float(gg[1])
      self.gyro[1] = float(gg[2])
      self.gyro[2] = float(gg[3])
      self.data[0,self.dataIdx] = self.gyro[0]
      self.data[1,self.dataIdx] = self.gyro[1]
      self.data[2,self.dataIdx] = self.gyro[2]
      self.dataIdx += 1
      self.dataNew = True
      if self.dataIdx >= 100:
        self.dataIdx = 0
    elif gg[0] == "gyrooff":
      self.offs[0] = float(gg[1])
      self.offs[1] = float(gg[2])
      self.offs[2] = float(gg[3])
      self.offsNew = True
      pass
    else:
      isOK = False
    return isOK

