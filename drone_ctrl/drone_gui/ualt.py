#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for altimeter data and settings
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
import timeit
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore


class UAlt(object):
  # class variables
  parent = []
  ui = []
  main = []
  data = np.zeros((3,100))
  dataTime = np.zeros(100)
  dataIdx = 0
  dataIdxTime = time.time()
  datam = np.zeros((2,100))
  datamTime = np.zeros(100)
  datamIdx = 0
  datamIdxTime = time.time()
  altNew = True
  altRaw = 0
  altFilt = 0
  velRaw = 0
  velFilt = 0
  filtNew = False
  height = 0
  heightVel = 0
  heightNew = False
  sonarHeight = 0
  sonarLimit = 0
  sonarNew = False
  sonarSampleTime = 0
  sonarOffset = 6*0.0254
  sonarValid = False
  temp = 25
  altOffset = 0
  filtTau = 2
  filtBeta = 10
  hasFocus = False
  flag = False



  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui
    self.main = self.parent.main
    self.flag = self.parent.ui.checkBox_center_stick.isChecked()
    for i in range(0,100):
      self.dataTime[i] = i/100
      self.datamTime[i] = i/100
    # plot widget for raw data
    self.dataPw = pg.PlotWidget(name='alt',title='Height raw') 
    self.dataPw.setLabel('left','altitude','m')
    self.dataPw.addLegend()
    self.dataPw.showGrid(True,True,0.6);
    self.dataPw.setLabel('bottom','Time','s')
    self.ui.horizontalLayout_alt.addWidget(self.dataPw)
    self.dataCg = [self.dataPw.plot(pen='y',name='altitude'), self.dataPw.plot(pen='r',name='filtered'), self.dataPw.plot(pen='g',name='sonar')]
    self.dataCg[0].setData(self.dataTime, self.data[0])
    self.dataCg[1].setData(self.dataTime, self.data[1])
    self.dataCg[2].setData(self.dataTime, self.data[2])
    # height velocity
    self.dataPwh = pg.PlotWidget(name='alt',title='Height and rate') 
    self.dataPwh.setLabel('left','height [m] velocity [m/s]','m/s')
    self.dataPwh.addLegend()
    self.dataPwh.showGrid(True,True,0.6);
    self.dataPwh.setLabel('bottom','Time','s')
    self.ui.horizontalLayout_height.addWidget(self.dataPwh)
    self.dataCgh = [self.dataPwh.plot(pen='y',name='height'), self.dataPwh.plot(pen='r',name='height rate')]
    self.dataCgh[0].setData(self.datamTime, self.datam[0])
    self.dataCgh[1].setData(self.datamTime, self.datam[1])

  def init(self):
    self.ui.pushButton_alt_apply.clicked.connect(self.setKnownAltitude)
    self.ui.pushButton_alt_calibrate.clicked.connect(self.doCalibrate)
    self.ui.pushButton_filter_apply.clicked.connect(self.setFilter)
    self.ui.pushButton_sonar_apply.clicked.connect(self.setSonar)
    pass

  def timerUpdate(self, timerCnt):
    if timerCnt % 10 == 0:
      pass
    #
    # update fields with new data
    if self.altNew:
      self.ui.doubleSpinBox_alt.setValue(self.altRaw)
      self.ui.doubleSpinBox_alt_offset.setValue(self.altOffset)
      self.ui.doubleSpinBox_alt_temp.setValue(self.temp)
      self.altNew = False
    if self.heightNew:
      self.ui.doubleSpinBox_alt_height.setValue(self.height)
      self.ui.doubleSpinBox_alt_height_vel.setValue(self.heightVel)
      # plot data
      #if(self.ui.checkBox_height_raw.isChecked()):
      #self.dataPw.setTitle("Height raw")
      #self.dataPw.setLabel('left','height','m')
      #self.dataCg = [self.dataPw.plot(pen='y',name='Altitude'), self.dataPw.plot(pen='r',name='Filtered'), self.dataPw.plot(pen='g',name='Sonar')]
      self.dataCg[0].setData(self.data[0])
      self.dataCg[1].setData(self.data[1])
      self.dataCg[2].setData(self.data[2])
      #else:
      #self.dataPw.setTitle("Merged height and rate") 
      #self.dataPw.setLabel('left','altitude','m')
      #self.dataCg = [self.dataPw.plot(pen='y',name='height'), self.dataPw.plot(pen='r',name='height rate')] #, self.dataPw.plot(pen='g',name='z_vel')]
      self.dataCgh[0].setData(self.datam[0])
      self.dataCgh[1].setData(self.datam[1])
      # @todo mangler ny merged height
      self.heightNew = False
    if self.sonarNew:
      self.ui.doubleSpinBox_sonar_height.setValue(self.sonarHeight)
      self.ui.doubleSpinBox_sonar_limit2.setValue(self.sonarLimit)
      self.ui.doubleSpinBox_sonar_offset2.setValue(self.sonarOffset)
      self.ui.doubleSpinBox_sonar_upd_time.setValue(self.sonarSampleTime)
      self.ui.checkBox_sonar_valid.setChecked(self.sonarValid)
      self.sonarNew = False
    if self.filtNew:
      self.ui.doubleSpinBox_filter_tau_2.setValue(self.filtTau)
      self.filtNew = False

    #
    # request data, if connected
    if not self.main.isConnected():
      # act as leaving the page
      self.hasFocus = False
    if self.hasFocus and self.ui.tabWidget.currentIndex() != self.ui.tabWidget.indexOf(self.ui.tab_height):
      # just leaving this tab
      self.hasFocus = False
      #if self.main.wifi.isOpen():
        ## we are talking to a bridge - so subscribe off
        #self.main.devWrite("alti subscribe 0\n") # imu actual data
        #self.main.devWrite("mhgt subscribe 0\n") # merged height
        #self.main.devWrite("uhgt subscribe 0\n") # sonar height
        #self.main.devWrite("fht subscribe 0\n") # height filter
      #pass
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == self.ui.tabWidget.indexOf(self.ui.tab_height):
      # just entering this tab
      self.hasFocus = True
      #if self.main.wifi.isOpen():
        ## we are talking to a bridge - so subscribe
        #self.main.devWrite("alti subscribe 1\n") # imu actual data
        #self.main.devWrite("mhgt subscribe 1\n") # merged height
        #self.main.devWrite("uhgt subscribe 1\n") # sonar
        #self.main.devWrite("fht subscribe 1\n") # height filter
        #pass
    if self.hasFocus:
      if timerCnt % 4 == 1:
        self.main.devWrite("alt\n", True) # pressure altitude
      if timerCnt % 4 == 2:
        self.main.devWrite("hgt\n", True) # merged height
      if timerCnt % 4 == 3:
        self.main.devWrite("uht\n", True) # ultrasonic (sonar) height
      if timerCnt % 31 == 1:
        self.main.devWrite("alti\n", True) # 
      if timerCnt % 31 == 12:
        self.main.devWrite("fhgt\n", True) # 
    pass 
  
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "alti": # mostly pressure based altitude
      # alti -1.17 -1.30 29.2 9.54 1 5
      self.altRaw = float(gg[1])
      self.altFilt = float(gg[2])
      self.temp = float(gg[3])
      self.altOffset = float(gg[4])
      #self.filtTau = float(gg[5])
      #self.filtBeta = float(gg[6])
      #self.velRaw = float(gg[7])
      #self.velFilt = float(gg[5])
      self.data[0,self.dataIdx] = self.altRaw - self.altOffset
      self.data[1,self.dataIdx] = self.altFilt
      self.data[2,self.dataIdx] = self.sonarHeight
      #self.data[3,self.dataIdx] = self.velFilt
      self.dataTime[self.dataIdx] = time.time() - self.dataIdxTime
      self.dataIdx += 1
      if self.dataIdx >= 100:
          self.dataIdx = 0;
          self.dataIdxTime = time.time()
      self.altNew = True
    elif gg[0] == "mhgt": # merged height and height velocity
      # mhgt h[m] hv[m/s]
      self.height = float(gg[1])
      self.heightVel = float(gg[2])
      self.datam[0,self.datamIdx] = self.height
      self.datam[1,self.datamIdx] = self.heightVel
      self.datamIdx += 1
      if self.datamIdx >= 100:
          self.datamIdx = 0;
          self.datamIdxTime = time.time()
      self.datamTime[self.datamIdx] = time.time() - self.datamIdxTime
      self.heightNew = True
    elif gg[0] == "uhgt": # ultrasonic sonar height and limit of use
      # mhgt h[m] hv[m/s]
      self.sonarHeight = float(gg[1])
      self.sonarVelocity = float(gg[2]) # not used
      self.sonarSampleTime = float(gg[3])
      self.sonarLimit = float(gg[4])
      self.sonarOffset = float(gg[5])
      self.sonarValid = int(gg[6])
      self.sonarNew = True
    elif gg[0] == "fht": # ultrasonic sonar height and limit of use
      # mhgt h[m] hv[m/s]
      self.filtTau = float(gg[1])
      self.filtNew = True
    else:
      isOK = False
    return isOK

  def setKnownAltitude(self):
    self.main.devWrite("alt {:g}\n".format(self.ui.doubleSpinBox_alt_set_to.value()), True)
    print("# sending alt {:g}\n".format(self.ui.doubleSpinBox_alt_set_to.value()))
    pass

  def doCalibrate(self):
    self.main.devWrite("offsetcal h\n", True)
    print("# sending offsetcal h")
    pass
  
  def setFilter(self):
    self.main.devWrite("hfilt {:g}\n".format(self.ui.doubleSpinBox_filter_tau.value()), True)
    print("# sending hfilt {:g}\n".format(self.ui.doubleSpinBox_filter_tau.value()))
    pass

  def setSonar(self):
    self.main.devWrite("sonar {:g} {:g}\n".format(self.ui.doubleSpinBox_sonar_limit.value(), self.ui.doubleSpinBox_sonar_offset.value()), True)
    print("# sending sonar {:g} {:g}\n".format(self.ui.doubleSpinBox_sonar_limit.value(), self.ui.doubleSpinBox_sonar_offset.value()))
    pass

