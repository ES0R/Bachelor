#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2021 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for manual control of setpoints (control reference)
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
from PyQt5 import QtWidgets, QtCore


class URef(object):
  # class variables
  parent = []
  ui = []
  main = []
  motor = []
  heightRef = 0
  rollRef = 0
  pitchRef = 0  
  yawRef = 0
  refNew = True
  limits = [0.0,100,-5,5,-5,5,-5,5]
  limitsNew = True
  heightRefOld = 0
  rollRefOld = 0
  pitchRefOld = 0  
  yawRefOld = 0
  limitChange = False  
  mixData = [0.0,0.0,0.0,0.0]
  mixDataNew = True
  bypassRC = False
  hasFocus = False
  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui

  def init(self):
    self.main = self.parent.main
    self.ui.doubleSpinBox_manual_height_min.valueChanged.connect(self.heightChange)
    self.ui.doubleSpinBox_manual_height_max.valueChanged.connect(self.heightChange)
    self.ui.doubleSpinBox_manual_roll_max.valueChanged.connect(self.RollChange)
    self.ui.doubleSpinBox_manual_pitch_max.valueChanged.connect(self.PitchChange)
    self.ui.doubleSpinBox_manual_yaw_max.valueChanged.connect(self.YawChange)
    #self.ui.horizontalSlider_manual_height.SliderValueChange.connect(self.sliderChange)
    self.manualChanged()
    self.ui.checkBox_manual_control.stateChanged.connect(self.manualChanged)
    self.ui.checkBox_manual_bypass.stateChanged.connect(self.manualBypass)
    self.ui.checkBox_center_stick.stateChanged.connect(self.centerStick)
    self.motor = self.parent.motor
    pass

  def timerUpdate(self, timerCnt):
    #
    thisTab = self.ui.tabWidget.indexOf(self.ui.tab_ref)
    if not self.main.isConnected():
      # act as leaving the page
      self.hasFocus = False
      self.ui.checkBox_manual_bypass.setChecked(False)
      self.ui.checkBox_manual_control.setChecked(False)
    if self.hasFocus and self.ui.tabWidget.currentIndex() != thisTab:
      # just leaving this tab
      self.hasFocus = False
      #if self.main.wifi.isOpen():
        ## we are talking to a bridge - so subscribe off
        #self.main.devWrite("refi subscribe 0\n") # ref values
        #self.main.devWrite("limiti subscribe 0\n") # 
        #self.main.devWrite("mixi subscribe 0\n") # 
        #self.main.devWrite("esd subscribe 0\n") # 
      #pass
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == thisTab:
      # just entering this tab
      self.hasFocus = True
      # set value when entering page, else not - to allow edit
      # @todo - virker ikke
      self.ui.checkBox_manual_bypass.setChecked(self.bypassRC)
      #if self.main.wifi.isOpen():
        ## we are talking to a bridge - so subscribe
        #self.main.devWrite("refi subscribe 1\n") # 
        #self.main.devWrite("limiti subscribe 1\n") # 
        #self.main.devWrite("mixi subscribe 1\n") # 
        #self.main.devWrite("esd subscribe 1\n") # 
        #pass
    if self.hasFocus:
      if self.refNew:
        self.ui.lineEdit_man_ref_height.setText("{:.1f}".format(self.heightRef))
        self.ui.lineEdit_man_ref_roll.setText("{:.2f}".format(self.rollRef))
        self.ui.lineEdit_man_ref_pitch.setText("{:.2f}".format(self.pitchRef))
        self.ui.lineEdit_man_ref_yaw.setText("{:.2f}".format(self.yawRef))
        if not self.ui.checkBox_manual_control.isChecked():
          h = (self.heightRef - self.ui.doubleSpinBox_manual_height_min.value()) / (self.ui.doubleSpinBox_manual_height_max.value() - self.ui.doubleSpinBox_manual_height_min.value()) * 1024
          self.ui.horizontalSlider_manual_height.setValue(h)
          r = (self.rollRef - self.ui.doubleSpinBox_manual_roll_min.value()) / (self.ui.doubleSpinBox_manual_roll_max.value() - self.ui.doubleSpinBox_manual_roll_min.value()) * 1024
          self.ui.horizontalSlider_manual_roll.setValue(r)
          p = (self.pitchRef - self.ui.doubleSpinBox_manual_pitch_min.value()) / (self.ui.doubleSpinBox_manual_pitch_max.value() - self.ui.doubleSpinBox_manual_pitch_min.value()) * 1024
          self.ui.horizontalSlider_manual_pitch.setValue(p)
          y = (self.yawRef - self.ui.doubleSpinBox_manual_yaw_min.value()) / (self.ui.doubleSpinBox_manual_yaw_max.value() - self.ui.doubleSpinBox_manual_yaw_min.value()) * 1024
          self.ui.horizontalSlider_manual_yaw.setValue(y)
        self.refNew = False;
      if not self.ui.checkBox_manual_control.isChecked():
        if self.limitsNew :
          self.ui.doubleSpinBox_manual_height_min.setValue(self.limits[0])
          self.ui.doubleSpinBox_manual_height_max.setValue(self.limits[1])
          self.ui.doubleSpinBox_manual_roll_min.setValue(self.limits[2])
          self.ui.doubleSpinBox_manual_roll_max.setValue(self.limits[3])
          self.ui.doubleSpinBox_manual_pitch_min.setValue(self.limits[4])
          self.ui.doubleSpinBox_manual_pitch_max.setValue(self.limits[5])
          self.ui.doubleSpinBox_manual_yaw_min.setValue(self.limits[6])
          self.ui.doubleSpinBox_manual_yaw_max.setValue(self.limits[7])
          self.limitsNew = False
          pass
        pass
      if self.mixDataNew :
        self.ui.lineEdit_man_mix_height.setText("{:.1f}".format(self.mixData[0]))
        self.ui.lineEdit_man_mix_roll.setText("{:.1f}".format(self.mixData[1]))
        self.ui.lineEdit_man_mix_pitch.setText("{:.1f}".format(self.mixData[2]))
        self.ui.lineEdit_man_mix_yaw.setText("{:.1f}".format(self.mixData[3]))
        self.mixDataNew = False;
        pass
      if self.motor.escData:
        self.ui.lineEdit_man_esc_1.setText(str(int(self.motor.escVal[0])))
        self.ui.lineEdit_man_esc_2.setText(str(int(self.motor.escVal[1])))
        self.ui.lineEdit_man_esc_3.setText(str(int(self.motor.escVal[2])))
        self.ui.lineEdit_man_esc_4.setText(str(int(self.motor.escVal[3])))
        self.ui.lineEdit_man_esc_5.setText(str(int(self.motor.escVal[4])))
        self.ui.lineEdit_man_esc_6.setText(str(int(self.motor.escVal[5])))
        self.ui.lineEdit_man_esc_7.setText(str(int(self.motor.escVal[6])))
        self.ui.lineEdit_man_esc_8.setText(str(int(self.motor.escVal[7])))
        self.motor.escData = False
      if self.ui.checkBox_manual_control.isChecked():
        #print("# man ctrl")
        # check for new position of sliders - we are in control
        vh = self.ui.horizontalSlider_manual_height.value()
        h = vh/1024*(self.ui.doubleSpinBox_manual_height_max.value() - self.ui.doubleSpinBox_manual_height_min.value()) +                    self.ui.doubleSpinBox_manual_height_min.value()
        v = self.ui.horizontalSlider_manual_roll.value()
        r = v/1024*(self.ui.doubleSpinBox_manual_roll_max.value() - self.ui.doubleSpinBox_manual_roll_min.value()) +                    self.ui.doubleSpinBox_manual_roll_min.value()
        v = self.ui.horizontalSlider_manual_pitch.value()
        p = v/1024*(self.ui.doubleSpinBox_manual_pitch_max.value() - self.ui.doubleSpinBox_manual_pitch_min.value()) +                    self.ui.doubleSpinBox_manual_pitch_min.value()
        vy = self.ui.horizontalSlider_manual_yaw.value()
        y = vy/1024*(self.ui.doubleSpinBox_manual_yaw_max.value() - self.ui.doubleSpinBox_manual_yaw_min.value()) +                    self.ui.doubleSpinBox_manual_yaw_min.value()
        changed = h != self.heightRefOld or r != self.rollRefOld or p != self.pitchRefOld or y != self.yawRefOld
        if changed:
          print("# man ctrl - sending new refs")
          self.heightRefOld = h
          self.rollRefOld = r
          self.pitchRefOld = p
          self.yawRefOld = y
          self.main.devWrite("ref {:g} {:g} {:g} {:g}\n".format(h, r, p, y), True) 
        pass
      if self.limitChange:
        self.main.devWrite("limit {:g} {:g} {:g} {:g} {:g}\n".format(
          self.ui.doubleSpinBox_manual_height_min.value(),
          self.ui.doubleSpinBox_manual_height_max.value(),
          self.ui.doubleSpinBox_manual_roll_max.value(),
          self.ui.doubleSpinBox_manual_pitch_max.value(),
          self.ui.doubleSpinBox_manual_yaw_max.value()
          ), True)
        self.limitChange = False
      pass
      if timerCnt % 3 == 1:
        self.main.devWrite("ref\n", True) # get ref from drone
      if timerCnt % 30 == 11:
        self.main.devWrite("limit\n", True) # get manuel limit from drone
      if timerCnt % 20 == 16:
        self.main.devWrite("mix\n", True)    # mixer input data
      if timerCnt % 8 == 3:
        self.main.devWrite("esi\n", True)    # ESC control data
    pass 
  
  def heightChange(self):
    self.limitChange = True;
    pass
  def RollChange(self):
    self.ui.doubleSpinBox_manual_roll_min.setValue(-self.ui.doubleSpinBox_manual_roll_max.value())
    self.limitChange = True;
    pass
  def PitchChange(self):
    self.ui.doubleSpinBox_manual_pitch_min.setValue(-self.ui.doubleSpinBox_manual_pitch_max.value())
    self.limitChange = True;
    pass
  def YawChange(self):
    self.ui.doubleSpinBox_manual_yaw_min.setValue(-self.ui.doubleSpinBox_manual_yaw_max.value())
    self.limitChange = True;
    pass
                  
  def centerStick(self):
    # en = self.ui.checkBox_center_stick.isChecked()
    self.parent.rc.updateHeight_lim()
    #self.parent.alt.UpdateGraphs()
    
  def manualChanged(self):
    en = not self.ui.checkBox_manual_control.isChecked()
    if not en:
       self.ui.horizontalSlider_manual_height.setValue(0)
    self.ui.doubleSpinBox_manual_height_min.setReadOnly(en)
    self.ui.doubleSpinBox_manual_height_max.setReadOnly(en)
    self.ui.doubleSpinBox_manual_roll_max.setReadOnly(en)
    self.ui.doubleSpinBox_manual_pitch_max.setReadOnly(en)
    self.ui.doubleSpinBox_manual_yaw_max.setReadOnly(en)
    # tell Teensy we are in control-over-USB mode
    self.manualBypass()
    if en:
      # force sending manual ref
      self.heightRefOld = -1;
    pass    
    
  def manualBypass(self):
    self.bypassRC = self.ui.checkBox_manual_bypass.isChecked()
    man = self.ui.checkBox_manual_control.isChecked()
    self.main.devWrite("bypass {:d} {:d}\n".format(self.bypassRC, man), True)
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "refi":
      try:
        self.heightRef = float(gg[1])
        self.rollRef = float(gg[2])
        self.pitchRef = float(gg[3])
        self.yawRef = float(gg[4])
        self.refNew = True
      except:
        print("# decode of 'refi' failed\n");
        self.main.message("# decode of 'refi' failed");
    elif gg[0] == "limiti":
      try:
        for i in range(0,8):
          self.limits[i] = float(gg[i+1])
        self.limitsNew = True
      except:
        print("# decode of 'limiti' failed\n");
        self.main.message("# decode of 'limiti' failed");
    elif gg[0] == "mixi":
      try:
        self.escMax = float(gg[5])
        for i in range(0,4):
          self.mixData[i] = float(gg[i+1]);
        self.mixDataNew = True
      except:
        print("# decode of 'mixi' failed\n");
        self.main.message("# decode of 'mixii' failed");
    else:
      isOK = False
    return isOK
  
  def requestLimitData(self):
    self.main.devWrite("limit\n", True)
    time.sleep(0.1);

  
  def saveToIniFile(self, config):
    if self.main.isConnected():
      self.requestLimitData()
    config.add_section("refLimit")
    config.set("refLimit", 'trustMin', str(self.limits[0]))
    config.set("refLimit", 'trustMax', str(self.limits[1]))
    config.set("refLimit", 'rollMin', str(self.limits[2]))
    config.set("refLimit", 'rollMax', str(self.limits[3]))
    config.set("refLimit", 'pitchMin', str(self.limits[4]))
    config.set("refLimit", 'pitchMax', str(self.limits[5]))
    config.set("refLimit", 'yawMin', str(self.limits[6]))
    config.set("refLimit", 'yawMax', str(self.limits[7]))
    
    
  def loadFromIniFile(self, config):
    try:
      self.limits[0] = config.getfloat("refLimit", 'trustMin')
      self.limits[1] = config.getfloat("refLimit", 'trustMax')
      self.limits[2] = config.getfloat("refLimit", 'rollMin')
      self.limits[3] = config.getfloat("refLimit", 'rollMax')
      self.limits[4] = config.getfloat("refLimit", 'pitchMin')
      self.limits[5] = config.getfloat("refLimit", 'pitchMax')
      self.limits[6] = config.getfloat("refLimit", 'yawMin')
      self.limits[7] = config.getfloat("refLimit", 'yawMax')
      self.limitsNew = True
    except:
      print("# load from ini failed to find refLimit block")
    pass
