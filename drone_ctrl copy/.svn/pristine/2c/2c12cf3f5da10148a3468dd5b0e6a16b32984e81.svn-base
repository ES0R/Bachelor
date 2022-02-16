#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for pose monitoring
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


class UPose(object):
  # class variables
  parent = []
  ui = []
  main = []
  lastIdx = -1
  roll = 0
  pitch = 0
  yaw = 0
  height = 0
  heightVel = 0
  poseUpd = True
  data = np.zeros((3,300))
  datah = np.zeros((1,300))
  dataTime = np.zeros(300)
  dataIdx = 0
  dataIdxTime = time.time()
  hasFocus = False

  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui
    self.main = self.parent.main
    # plot widget
    self.dataPw = pg.PlotWidget(name='pose',title='Roll, Pitch') 
    self.dataPw.setLabel('left','angle','degrees')
    self.dataPw.addLegend()
    self.dataPw.showGrid(True,True,0.6);
    self.dataPw.setLabel('bottom','Time','s')
    self.ui.verticalLayout_pose.addWidget(self.dataPw)
    #self.dataCg = [self.dataPw.plot(pen='r',name='roll'), 
                   #self.dataPw.plot(pen='c',name='pitch'), 
                   #self.dataPw.plot(pen='g',name='yaw')]
    self.dataCg = [self.dataPw.plot(pen='r',name='roll'), 
                   self.dataPw.plot(pen='c',name='pitch')]
    # yaw widget
    self.dataPwY = pg.PlotWidget(name='pose yaw',title='Yaw') 
    self.dataPwY.setLabel('left','angle','degrees')
    self.dataPwY.setYRange(-180,180);
    self.dataPwY.showGrid(True,True,0.6);
    #self.dataPwY.addLegend()
    self.dataPwY.setLabel('bottom','Time','s')
    self.ui.verticalLayout_pose_yaw.addWidget(self.dataPwY)
    self.dataCgY = [self.dataPwY.plot(pen='g',name='Yaw')]
    # data
    for i in range(0,300):
      self.dataTime[i] = i/100
    self.dataCg[0].setData(self.dataTime, self.data[0])
    self.dataCg[1].setData(self.dataTime, self.data[1])
    # yaw
    self.dataCgY[0].setData(self.dataTime, self.data[2])
    #self.dataCg[3].setData(self.datah[0])


  def init(self):
    self.ui.checkBox_pose_autoupdate.clicked.connect(self.autoUpdate)
    self.ui.checkBox_pose_use_mag.clicked.connect(self.useMag)
    pass

  def timerUpdate(self, timerCnt):
    if self.poseUpd:
      self.ui.doubleSpinBox_pose_roll.setValue(self.roll*180/np.pi)
      self.ui.doubleSpinBox_pose_pitch.setValue(self.pitch*180/np.pi)
      self.ui.doubleSpinBox_pose_yaw.setValue(self.yaw*180/np.pi)
      self.ui.doubleSpinBox_pose_roll_rad.setValue(self.roll)
      self.ui.doubleSpinBox_pose_pitch_rad.setValue(self.pitch)
      self.ui.doubleSpinBox_pose_yaw_rad.setValue(self.yaw)
      self.ui.doubleSpinBox_pose_height.setValue(self.height)
      self.ui.doubleSpinBox_pose_height_velocity.setValue(self.heightVel)
      self.dataCg[0].setData(self.dataTime, self.data[0])
      self.dataCg[1].setData(self.dataTime, self.data[1])
      self.dataCgY[0].setData(self.dataTime, self.data[2])
      #self.dataCg[3].setData(self.datah[0])
      self.poseUpd = False
    #
    # data request
    thisTab = self.ui.tabWidget.indexOf(self.ui.tab_pose);
    if self.hasFocus and self.ui.tabWidget.currentIndex() != thisTab:
      # just leaving this tab
      self.hasFocus = False
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe off to bridge only
        self.main.devWrite("sub pos 0\n") # 
      else:
        # direct USB, so off subscription
        self.main.devWrite("sub pose 0\n", True) # kill subscription
      pass
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == thisTab:
      # just entering this tab
      self.hasFocus = True
      if self.ui.checkBox_pose_autoupdate.isChecked():
        # NB! lidt farligt, hvis der er andre klienter - e.g. via on-board raspberry pi
        
        self.main.devWrite("sub pose 4\n", True) # start stream
        # just arrived - set time to now
        self.dataIdxTime = time.time()
        self.dataIdx = 0
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe
        self.main.devWrite("sub pos -1 30\n") # subscribe to pose
      pass
    if self.hasFocus:
      pass
  
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "pos":
      self.roll = float(gg[1])
      self.pitch = float(gg[2])
      self.yaw = float(gg[3])
      self.height = float(gg[4])
      if len(gg) > 5:
        self.heightVel = float(gg[5])
      self.dataTime[self.dataIdx] = time.time() - self.dataIdxTime
      self.data[0,self.dataIdx] = self.roll * 180 / np.pi
      self.data[1,self.dataIdx] = self.pitch * 180 / np.pi
      self.data[2,self.dataIdx] = self.yaw * 180 / np.pi
      self.datah[0,self.dataIdx] = self.height
      self.dataIdx += 1
      if self.dataIdx >= 300:
        self.dataIdx = 0
        self.dataIdxTime = time.time();
      self.poseUpd = True
    else:
      isOK = False
    if isOK:
      self.poseUpd = True
    return isOK

  def autoUpdate(self):
    if self.ui.checkBox_pose_autoupdate.isChecked():
      self.main.devWrite("sub pose 4\n", True) # start stream
    else:      
      self.main.devWrite("sub pose 0\n", True) # stop stream

  def useMag(self):
    if self.ui.checkBox_pose_use_mag.isChecked():
      self.main.devWrite("usemag 1\n", True) # start using magnetometer
    else:      
      self.main.devWrite("usemag 0\n", True) # stop using magnetometer
