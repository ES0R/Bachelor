#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for magnetometer data monitoring
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
#import os
import numpy as np
import time
#from time import sleep
#import ConfigParser
import timeit
import pyqtgraph as pg
try:
  import configparser
except:
  import ConfigParser  
from PyQt5 import QtWidgets, QtCore


class UMag(object):
  # class variables
  parent = []
  ui = []
  main = []
  mag = [0,0,0, 0,0,0, 0,0,0]
  magStrength = 0
  magOffset = [0,0,0]
  magData = [0,0,0]
  magcNew = True;
  magOfsNew = True
  magDataNew = True
  data = np.zeros((3,100))
  dataTime = np.zeros(100)
  dataIdx = 0
  time0 = time.time()
  lastTab = -2
  hasFocus = False

  
  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui
    self.main = self.parent.main
    # plot widget
    pg.setConfigOption('background', 'k')
    pg.setConfigOption('foreground', 'w')
    self.dataPw = pg.PlotWidget(name='magnetometer',title='Magnetometer') 
    #self.dataPw.setBackground((53,53,53))
    self.dataPw.setLabel('left','Magnetic strength','uT')
    self.dataPw.setLabel('bottom','Time','s')
    self.dataPw.showGrid(True, True, 0.5)
    self.dataPw.addLegend()
    self.ui.horizontalLayout_mag.addWidget(self.dataPw)
    self.dataCg = [self.dataPw.plot(pen='r',name='x'), self.dataPw.plot(pen='c',name='y'), self.dataPw.plot(pen='g',name='z')]
    self.dataCg[0].setData(self.dataTime, self.data[0])
    self.dataCg[1].setData(self.dataTime, self.data[1])
    self.dataCg[2].setData(self.dataTime, self.data[2])

  def init(self):
    pass

  def timerUpdate(self, timerCnt):
    if self.magcNew:
      self.ui.doubleSpinBox_mag_mat_1.setValue(self.mag[0])
      self.ui.doubleSpinBox_mag_mat_2.setValue(self.mag[1])
      self.ui.doubleSpinBox_mag_mat_3.setValue(self.mag[2])
      self.ui.doubleSpinBox_mag_mat_4.setValue(self.mag[3])
      self.ui.doubleSpinBox_mag_mat_5.setValue(self.mag[4])
      self.ui.doubleSpinBox_mag_mat_6.setValue(self.mag[5])
      self.ui.doubleSpinBox_mag_mat_7.setValue(self.mag[6])
      self.ui.doubleSpinBox_mag_mat_8.setValue(self.mag[7])
      self.ui.doubleSpinBox_mag_mat_9.setValue(self.mag[8])
      self.ui.doubleSpinBox_mag_strength.setValue(self.magStrength)
      self.dataCg[0].setData(self.dataTime, self.data[0])
      self.dataCg[1].setData(self.dataTime, self.data[1])
      self.dataCg[2].setData(self.dataTime, self.data[2])
      self.magcNew = False
    if self.magOfsNew:
      self.ui.doubleSpinBox_mag_x_offset.setValue(self.magOffset[0])
      self.ui.doubleSpinBox_mag_y_offset.setValue(self.magOffset[1])
      self.ui.doubleSpinBox_mag_z_offset.setValue(self.magOffset[2])
      self.magOfsNew = False
    if self.magDataNew:
      self.ui.doubleSpinBox_mag_x.setValue(self.magData[0])
      self.ui.doubleSpinBox_mag_y.setValue(self.magData[1])
      self.ui.doubleSpinBox_mag_z.setValue(self.magData[2])
      self.magDataNew = False
    #
    if not self.main.isConnected():
      # act as leaving the page
      self.hasFocus = False
    thisTab = self.ui.tabWidget.indexOf(self.ui.tab_mag)
    if self.hasFocus and self.ui.tabWidget.currentIndex() != thisTab:
      # just leaving this tab
      self.hasFocus = False
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe off
        self.main.devWrite("sub mag 0\n") # data
      pass
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == thisTab:
      # just entering this tab
      self.hasFocus = True
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe
        self.main.devWrite("sub mag -1 30\n") # data
        pass
    if self.hasFocus:
      if timerCnt % 2 == 1:
        self.main.devWrite("imu m\n", True) # imu actual data
      if timerCnt % 10 == 5:
        self.main.devWrite("imucal m\n", True) # imu calibration offsets - also acc
      if self.lastTab != thisTab:
        # tab just became visible
        self.lastTab = thisTab
        self.time0 = time.time() # to make graph start from 0 (ish)
        self.dataIdx = 0
        for i in range(0,100):
          self.dataTime[i] = 0
    pass 
  
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "imumagc":
      self.mag[0] = float(gg[1])
      self.mag[1] = float(gg[2])
      self.mag[2] = float(gg[3])
      self.mag[3] = float(gg[4])
      self.mag[4] = float(gg[5])
      self.mag[5] = float(gg[6])
      self.mag[6] = float(gg[7])
      self.mag[7] = float(gg[8])
      self.mag[8] = float(gg[9])
      self.magStrength = float(gg[10])
      self.magcNew = True;
    elif gg[0] == "mag":
      self.magData[0] = float(gg[1])
      self.magData[1] = float(gg[2])
      self.magData[2] = float(gg[3])
      self.data[0, self.dataIdx] = self.magData[0]
      self.data[1, self.dataIdx] = self.magData[1]
      self.data[2, self.dataIdx] = self.magData[2]
      self.dataTime[self.dataIdx] = time.time() - self.time0
      self.dataIdx += 1
      if self.dataIdx >= 100:
        self.dataIdx = 0
        self.time0 = time.time()
      self.magDataNew = True
    elif gg[0] == "magoff" or gg[0] == "magoff":
      self.magOffset[0] = float(gg[1])
      self.magOffset[1] = float(gg[2])
      self.magOffset[2] = float(gg[3])
      self.magOfsNew = True
    else:
      isOK = False
    return isOK

