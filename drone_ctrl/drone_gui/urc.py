#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2021 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for radio control (RC) interface
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
try:
  import configparser as cp
except:
  import ConfigParser as cp
import timeit
from PyQt5 import QtWidgets, QtCore


class URc(object):
  # class variables
  parent = []
  ui = []
  main = []
  channel = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  offset =  [172,992,992,922,0,0,0,0,0,0,0,0,0,0,0,0]
  scale =   [0.55,0,0,-0.01,0,0,0,0,0,0,0,0,0,0,0,0]
  scaleOffsetNew = True
  failsafe = True
  frameCnt = 0;
  frameFail = 0;
  frameNew = True
  sampleTime = 1 # ms
  hasFocus = False
  nytflag = False

  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui
    self.main = self.parent.main
    

  def init(self):
    self.ui.pushButton_RC_apply.clicked.connect(self.sendScaleOffset)
    pass

  def timerUpdate(self, timerCnt):
    if self.frameNew or self.scaleOffsetNew:
      # recalculate output
      self.ui.label_RC_1.setText("{:7.2f}".format((self.channel[0] - self.offset[0])*self.scale[0]))
      self.ui.label_RC_2.setText("{:7.2f}".format((self.channel[1] - self.offset[1])*self.scale[1]))
      self.ui.label_RC_3.setText("{:7.2f}".format((self.channel[2] - self.offset[2])*self.scale[2]))
      self.ui.label_RC_4.setText("{:7.2f}".format((self.channel[3] - self.offset[3])*self.scale[3]))
      self.ui.label_RC_5.setText("{:7.1f}".format((self.channel[4] - self.offset[4])*self.scale[4]))
      self.ui.label_RC_6.setText("{:7.1f}".format((self.channel[5] - self.offset[5])*self.scale[5]))
      self.ui.label_RC_7.setText("{:7.2f}".format((self.channel[6] - self.offset[6])*self.scale[6]))
    if self.frameNew:
      self.ui.checkBox_rc_failsafe.setChecked(self.failsafe)
      self.ui.lineEdit_rc_fail_cnt.setText(str(self.frameFail))
      self.ui.lineEdit_rc_ok_cnt.setText(str(self.frameCnt))
      self.ui.horizontalSlider_RC_1.setValue(self.channel[0])
      self.ui.horizontalSlider_RC_2.setValue(self.channel[1])
      self.ui.horizontalSlider_RC_3.setValue(self.channel[2])
      self.ui.horizontalSlider_RC_4.setValue(self.channel[3])
      self.ui.horizontalSlider_RC_5.setValue(self.channel[4])
      self.ui.horizontalSlider_RC_6.setValue(self.channel[5])
      self.ui.horizontalSlider_RC_7.setValue(self.channel[6])
      self.ui.horizontalSlider_RC_8.setValue(self.channel[7])
      self.ui.horizontalSlider_RC_9.setValue(self.channel[8])
      #self.ui.horizontalSlider_RC_10.setValue(self.channel[9])
      #self.ui.horizontalSlider_RC_11.setValue(self.channel[10])
      #self.ui.horizontalSlider_RC_12.setValue(self.channel[11])
      #self.ui.horizontalSlider_RC_13.setValue(self.channel[12])
      #self.ui.horizontalSlider_RC_14.setValue(self.channel[13])
      #self.ui.horizontalSlider_RC_15.setValue(self.channel[14])
      #self.ui.horizontalSlider_RC_16.setValue(self.channel[15])
      self.ui.spinBox_rc_framerate.setValue(self.sampleTime)
      self.ui.spinBox_RC_1.setValue(self.channel[0])
      self.ui.spinBox_RC_2.setValue(self.channel[1])
      self.ui.spinBox_RC_3.setValue(self.channel[2])
      self.ui.spinBox_RC_4.setValue(self.channel[3])
      self.ui.spinBox_RC_5.setValue(self.channel[4])
      self.ui.spinBox_RC_6.setValue(self.channel[5])
      self.ui.spinBox_RC_7.setValue(self.channel[6])
      self.ui.spinBox_RC_8.setValue(self.channel[7])
      self.ui.spinBox_RC_9.setValue(self.channel[8])
      self.frameNew = False
    if self.scaleOffsetNew and not self.ui.checkBox_RC_edit.isChecked():
      self.ui.doubleSpinBox_RC_1_scale.setValue(self.scale[0])
      self.ui.doubleSpinBox_RC_2_scale.setValue(self.scale[1])
      self.ui.doubleSpinBox_RC_3_scale.setValue(self.scale[2])
      self.ui.doubleSpinBox_RC_4_scale.setValue(self.scale[3])
      self.ui.doubleSpinBox_RC_5_scale.setValue(self.scale[4])
      self.ui.doubleSpinBox_RC_6_scale.setValue(self.scale[5])
      self.ui.doubleSpinBox_RC_7_scale.setValue(self.scale[6])
      self.ui.spinBox_RC_1_offset.setValue(self.offset[0])
      self.ui.spinBox_RC_2_offset.setValue(self.offset[1])
      self.ui.spinBox_RC_3_offset.setValue(self.offset[2])
      self.ui.spinBox_RC_4_offset.setValue(self.offset[3])
      self.ui.spinBox_RC_5_offset.setValue(self.offset[4])
      self.ui.spinBox_RC_6_offset.setValue(self.offset[5])
      self.ui.spinBox_RC_7_offset.setValue(self.offset[6])
      self.scaleOffsetNew = False
    #
    if timerCnt % 10 == 0:
      pass
    if not self.main.isConnected():
      # act as leaving the page
      self.hasFocus = False
    thisTab = self.ui.tabWidget.indexOf(self.ui.tab_RC)
    if self.hasFocus and self.ui.tabWidget.currentIndex() != thisTab:
      # just leaving this tab
      self.hasFocus = False
      #if self.main.wifi.isOpen():
        ## we are talking to a bridge - so subscribe off
        #self.main.devWrite("rcs subscribe 0\n") # rc values
        #self.main.devWrite("rcosi subscribe 0\n") # scale offset
      pass
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == thisTab:
      # just entering this tab
      self.hasFocus = True
      #if self.main.wifi.isOpen():
        ## we are talking to a bridge - so subscribe
        #self.main.devWrite("rcs subscribe 1\n") # rc values
        #self.main.devWrite("rcosi subscribe 1\n") # scale offset
        #pass
    if self.hasFocus:
      if timerCnt % 3 == 1:
        self.main.devWrite("rci\n", True) # request RC values
      if timerCnt % 30 == 4:
        self.main.devWrite("rcos\n", True) # request scale and offset
    pass 
  
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "rcs":
      self.failsafe = int(gg[1])
      self.frameCnt = int(gg[2])
      self.frameFail = int(gg[3])
      self.sampleTime = float(gg[4])
      for i in range(0,15):
        try:
          self.channel[i] = int(gg[i+5])
        except:
          self.channel[i] = 0
          print("#urc.py:decode error - item " + str(i))
      self.frameNew = True
    elif gg[0] == "rcosi":
      for i in range(0,7):
        try:
          self.scale[i] = float(gg[i*2+1])
          self.offset[i] = int(gg[i*2+2])
        except:
          self.scale[i] = 1
          self.offset[i] = 180
          print("#urc.py:decode (scale offset) error - item " + str(i) +":" + gg[i*2+1] + " " + gg[i*2+2])
      self.scaleOffsetNew = True
    else:
      isOK = False
    return isOK

  def sendScaleOffset(self):
    s = "rcos {:f} {:d} {:f} {:d} {:f} {:d} {:f} {:d} {:f} {:d} {:f} {:d} {:f} {:d}".format(
      self.ui.doubleSpinBox_RC_1_scale.value(), self.ui.spinBox_RC_1_offset.value(),
      self.ui.doubleSpinBox_RC_2_scale.value(), self.ui.spinBox_RC_2_offset.value(),
      self.ui.doubleSpinBox_RC_3_scale.value(), self.ui.spinBox_RC_3_offset.value(),
      self.ui.doubleSpinBox_RC_4_scale.value(), self.ui.spinBox_RC_4_offset.value(),
      self.ui.doubleSpinBox_RC_5_scale.value(), self.ui.spinBox_RC_5_offset.value(),
      self.ui.doubleSpinBox_RC_6_scale.value(), self.ui.spinBox_RC_6_offset.value(),
      self.ui.doubleSpinBox_RC_7_scale.value(), self.ui.spinBox_RC_7_offset.value()
      )
    self.main.devWrite(s, True)
      
  def saveToIniFile(self, config):
    # settings
    config.add_section('RC')
    config.set('RC', 'rc1', str(self.ui.spinBox_RC_1.value()) + " trust")
    config.set('RC', 'offset1', str(self.ui.spinBox_RC_1_offset.value()))
    config.set('RC', 'scale1', str(self.ui.doubleSpinBox_RC_1_scale.value()))
    config.set('RC', 'rc2', str(self.ui.spinBox_RC_2.value()) + " roll")
    config.set('RC', 'offset2', str(self.ui.spinBox_RC_2_offset.value()))
    config.set('RC', 'scale2', str(self.ui.doubleSpinBox_RC_2_scale.value()))
    config.set('RC', 'rc3', str(self.ui.spinBox_RC_3.value()) + " pitch")
    config.set('RC', 'offset3', str(self.ui.spinBox_RC_3_offset.value()))
    config.set('RC', 'scale3', str(self.ui.doubleSpinBox_RC_3_scale.value()))
    config.set('RC', 'rc4', str(self.ui.spinBox_RC_4.value()) + " yaw")
    config.set('RC', 'offset4', str(self.ui.spinBox_RC_4_offset.value()))
    config.set('RC', 'scale4', str(self.ui.doubleSpinBox_RC_4_scale.value()))
    config.set('RC', 'rc5', str(self.ui.spinBox_RC_5.value()))
    config.set('RC', 'offset5', str(self.ui.spinBox_RC_5_offset.value()))
    config.set('RC', 'scale5', str(self.ui.doubleSpinBox_RC_5_scale.value()))
    config.set('RC', 'rc6', str(self.ui.spinBox_RC_6.value()))
    config.set('RC', 'offset6', str(self.ui.spinBox_RC_6_offset.value()))
    config.set('RC', 'scale6', str(self.ui.doubleSpinBox_RC_6_scale.value()))
    config.set('RC', 'rc7', str(self.ui.spinBox_RC_7.value()))
    config.set('RC', 'offset7', str(self.ui.spinBox_RC_7_offset.value()))
    config.set('RC', 'scale7', str(self.ui.doubleSpinBox_RC_7_scale.value()))
    config.set('RC', 'rc8', str(self.ui.spinBox_RC_8.value()))

  def loadFromIniFile(self, config):
    config = cp.SafeConfigParser()
    config.read("drone_ctrl.ini")
    try:
      self.ui.spinBox_RC_1_offset.setValue(config.getint('RC', 'offset1'))
      self.ui.spinBox_RC_2_offset.setValue(config.getint('RC', 'offset2'))
      self.ui.spinBox_RC_3_offset.setValue(config.getint('RC', 'offset3'))
      self.ui.spinBox_RC_4_offset.setValue(config.getint('RC', 'offset4'))
      self.ui.spinBox_RC_5_offset.setValue(config.getint('RC', 'offset5'))
      self.ui.spinBox_RC_6_offset.setValue(config.getint('RC', 'offset6'))
      self.ui.spinBox_RC_7_offset.setValue(config.getint('RC', 'offset7'))
      self.ui.doubleSpinBox_RC_1_scale.setValue(config.getfloat('RC', 'scale1'))
      self.ui.doubleSpinBox_RC_2_scale.setValue(config.getfloat('RC', 'scale2'))
      self.ui.doubleSpinBox_RC_3_scale.setValue(config.getfloat('RC', 'scale3'))
      self.ui.doubleSpinBox_RC_4_scale.setValue(config.getfloat('RC', 'scale4'))
      self.ui.doubleSpinBox_RC_5_scale.setValue(config.getfloat('RC', 'scale5'))
      self.ui.doubleSpinBox_RC_6_scale.setValue(config.getfloat('RC', 'scale6'))
      self.ui.doubleSpinBox_RC_7_scale.setValue(config.getfloat('RC', 'scale7'))
    except:
      self.main.message("# failed to load debug from ini-file")
    pass

  def updateRoll_lim(self):
    self.scale[1] = -self.parent.ui.doubleSpinBox_manual_roll_max.value()/820
    self.UpdateScaleOffset()      

  def updatePitch_lim(self):
    self.scale[2] = -self.parent.ui.doubleSpinBox_manual_pitch_max.value()/820
    self.UpdateScaleOffset()

  def updateYaw_lim(self):
    self.scale[3] = -self.parent.ui.doubleSpinBox_manual_yaw_max.value()/820
    self.UpdateScaleOffset()
  
  def updateHeight_lim(self):
    flag = self.parent.ui.checkBox_center_stick.isChecked()
    if flag == False:
      self.scale[0] = 0.55
      self.offset[0] = 172
      self.parent.ref.limits[0] = 0.0
      self.parent.ref.limits[1] = 1000
    # elif flag == True:
    #   self.parent.ref.limits[0] = -self.parent.ui.doubleSpinBox_manual_height_max
    #   self.parent.ref.limits[1] = self.parent.ui.doubleSpinBox_manual_height_max
    #   self.scale[0] = self.scale[3] = -self.parent.ui.doubleSpinBox_manual_height_max.value()/820
    #   self.offset[0] = 992
    #   print(self.scale[0])        
    self.ui.spinBox_RC_1_offset.setValue(self.offset[0])
    self.UpdateScaleOffset()

  def UpdateScaleOffset(self):
    self.scaleOffsetNew = True
    s = "rcos {:f} {:d} {:f} {:d} {:f} {:d} {:f} {:d} {:f} {:d} {:f} {:d} {:f} {:d}".format(
      self.scale[0], self.ui.spinBox_RC_1_offset.value(),
      self.scale[1], self.ui.spinBox_RC_2_offset.value(),
      self.scale[2], self.ui.spinBox_RC_3_offset.value(),
      self.scale[3], self.ui.spinBox_RC_4_offset.value(),
      self.scale[4], self.ui.spinBox_RC_5_offset.value(),
      self.scale[5], self.ui.spinBox_RC_6_offset.value(),
      self.scale[6], self.ui.spinBox_RC_7_offset.value()
      )
    if self.main.isConnected():
      self.main.devWrite(s, True)
