#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2021 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for mixer settings
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
  import ConfigParser  as cp
import timeit
from PyQt5 import QtWidgets, QtCore


class UMixer(object):
  # class variables
  parent = []
  ui = []
  main = []
  motor = []
  ref = []
  hasFocus = False
  offset = [0,0,0,0,0,0,0,0]
  offsetNew = True
  escCnt = 6
  droneConfig = 1
  inTimerUpdate = False

  # methods / functions
  def __init__(self, parent):
    self.parent = parent
    self.ui = self.parent.ui
    self.main = self.parent.main
    

  def init(self):
    self.motor = self.parent.motor
    self.ref = self.parent.ref
    self.ui.spinBox_mixer_esc_cnt.valueChanged.connect(self.change_esc_count)
    self.ui.pushButton_mixer_copy.clicked.connect(self.copy_offset)
    self.ui.pushButton_mixer_apply.clicked.connect(self.apply_offset)
    self.ui.radioButton_mixer_x.clicked.connect(self.change_esc_count)
    self.ui.radioButton_mixer_plus.clicked.connect(self.change_esc_count)
    self.ui.radioButton_mixer_2x2.clicked.connect(self.change_esc_count)
    pass

  def timerUpdate(self, timerCnt):
    self.inTimerUpdate = True
    if self.hasFocus:
      if self.ref.mixDataNew :
        self.ui.spinBox_mixer_in_trust.setValue(self.ref.mixData[0])
        self.ui.spinBox_mixer_in_roll.setValue(self.ref.mixData[1])
        self.ui.spinBox_mixer_in_pitch.setValue(self.ref.mixData[2])
        self.ui.spinBox_mixer_in_yaw.setValue(self.ref.mixData[3])
        self.ref.mixDataNew = False;
        pass
      if self.motor.escData and self.hasFocus:
        self.ui.lineEdit_mixer_esc_1.setText(str(int(self.motor.escVal[0])))
        self.ui.lineEdit_mixer_esc_2.setText(str(int(self.motor.escVal[1])))
        self.ui.lineEdit_mixer_esc_3.setText(str(int(self.motor.escVal[2])))
        self.ui.lineEdit_mixer_esc_4.setText(str(int(self.motor.escVal[3])))
        self.ui.lineEdit_mixer_esc_5.setText(str(int(self.motor.escVal[4])))
        self.ui.lineEdit_mixer_esc_6.setText(str(int(self.motor.escVal[5])))
        self.ui.lineEdit_mixer_esc_7.setText(str(int(self.motor.escVal[6])))
        self.ui.lineEdit_mixer_esc_8.setText(str(int(self.motor.escVal[7])))
        self.motor.escData = False
      if self.offsetNew:
        self.ui.spinBox_mixer_esc_off_1.setValue(self.offset[0])
        self.ui.spinBox_mixer_esc_off_2.setValue(self.offset[1])
        self.ui.spinBox_mixer_esc_off_3.setValue(self.offset[2])
        self.ui.spinBox_mixer_esc_off_4.setValue(self.offset[3])
        self.ui.spinBox_mixer_esc_off_5.setValue(self.offset[4])
        self.ui.spinBox_mixer_esc_off_6.setValue(self.offset[5])
        self.ui.spinBox_mixer_esc_off_7.setValue(self.offset[6])
        self.ui.spinBox_mixer_esc_off_8.setValue(self.offset[7])
        self.ui.spinBox_mixer_esc_cnt.setValue(self.escCnt)
        if self.droneConfig == 1:
          self.ui.radioButton_mixer_x.setChecked(True)
        elif self.droneConfig == 2:
          self.ui.radioButton_mixer_plus.setChecked(True)
        elif self.droneConfig == 3:
          self.ui.radioButton_mixer_2x2.setChecked(True)
        else:
          self.ui.radioButton_mixer_x.setChecked(True)
        self.offsetNew = False
      pass
    self.inTimerUpdate = False
    pass
    #
    if not self.main.isConnected():
      # act as leaving the page
      self.hasFocus = False
    thisTab = self.ui.tabWidget.indexOf(self.ui.tab_mixer)
    if self.hasFocus and self.ui.tabWidget.currentIndex() != thisTab:
      # just leaving this tab
      self.hasFocus = False
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe off
        pass
        #self.main.devWrite("refi subscribe 0\n") # 
        #self.main.devWrite("mixi subscribe 0\n") # scale offset
        #self.main.devWrite("mixcon subscribe 0\n") # scale offset
      pass
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == thisTab:
      # just entering this tab
      self.hasFocus = True
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe
        pass
        #self.main.devWrite("refi subscribe 1\n") # 
        #self.main.devWrite("mixi subscribe 1\n") # scale offset
        #self.main.devWrite("mixcon subscribe 1\n") # scale offset
        #pass
    if self.hasFocus:
      if timerCnt % 30 == 7:
        self.main.devWrite("mixc\n", True) # request scale and offset
      if timerCnt % 15 == 16:
        self.main.devWrite("mix\n", True)    # mixer input data
      if timerCnt % 15 == 3:
        self.main.devWrite("esi\n", True)    # ESC control data
    pass 
  
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "mixcon":
      self.escCnt = int(gg[1])
      self.droneConfig = int(gg[2])
      for i in range(0,8):
        try:
          self.offset[i] = int(gg[i+3])
        except:
          self.offset[i] = 180
          print("#umixer.py:decode offset error - item " + str(i) +":" + gg[i+3])
      self.offsetNew = True
    else:
      isOK = False
    return isOK

      
  def saveToIniFile(self, config):
    # settings
    config.add_section('mixer')
    config.set('mixer', 'offset1', str(self.ui.offset[0]))
    config.set('mixer', 'offset2', str(self.ui.offset[1]))
    config.set('mixer', 'offset3', str(self.ui.offset[2]))
    config.set('mixer', 'offset4', str(self.ui.offset[3]))
    config.set('mixer', 'offset5', str(self.ui.offset[4]))
    config.set('mixer', 'offset6', str(self.ui.offset[5]))
    config.set('mixer', 'offset7', str(self.ui.offset[6]))
    config.set('mixer', 'offset8', str(self.ui.offset[7]))

  def loadFromIniFile(self, config):
    config = cp.SafeConfigParser()
    config.read("drone_ctrl.ini")
    try:
      self.ui.spinBox_mixer_esc_off_1e.setValue(config.getint('mixer', 'offset1'))
      self.ui.spinBox_mixer_esc_off_2e.setValue(config.getint('mixer', 'offset2'))
      self.ui.spinBox_mixer_esc_off_3e.setValue(config.getint('mixer', 'offset3'))
      self.ui.spinBox_mixer_esc_off_4e.setValue(config.getint('mixer', 'offset4'))
      self.ui.spinBox_mixer_esc_off_5e.setValue(config.getint('mixer', 'offset5'))
      self.ui.spinBox_mixer_esc_off_6e.setValue(config.getint('mixer', 'offset6'))
      self.ui.spinBox_mixer_esc_off_7e.setValue(config.getint('mixer', 'offset7'))
      self.ui.spinBox_mixer_esc_off_8e.setValue(config.getint('mixer', 'offset8'))
    except:
      self.main.message("# failed to load mixer offset from ini-file")
    pass


  def copy_offset(self):
    self.ui.spinBox_mixer_esc_off_1e.setValue(self.ui.spinBox_mixer_esc_off_1.value())
    self.ui.spinBox_mixer_esc_off_2e.setValue(self.ui.spinBox_mixer_esc_off_2.value())
    self.ui.spinBox_mixer_esc_off_3e.setValue(self.ui.spinBox_mixer_esc_off_3.value())
    self.ui.spinBox_mixer_esc_off_4e.setValue(self.ui.spinBox_mixer_esc_off_4.value())
    self.ui.spinBox_mixer_esc_off_5e.setValue(self.ui.spinBox_mixer_esc_off_5.value())
    self.ui.spinBox_mixer_esc_off_6e.setValue(self.ui.spinBox_mixer_esc_off_6.value())
    self.ui.spinBox_mixer_esc_off_7e.setValue(self.ui.spinBox_mixer_esc_off_7.value())
    self.ui.spinBox_mixer_esc_off_8e.setValue(self.ui.spinBox_mixer_esc_off_8.value())

  def apply_offset(self):
    s = "mixo {:d} {:d} {:d} {:d} {:d} {:d} {:d} {:d}\n".format(
      self.ui.spinBox_mixer_esc_off_1e.value(),
      self.ui.spinBox_mixer_esc_off_2e.value(),
      self.ui.spinBox_mixer_esc_off_3e.value(),
      self.ui.spinBox_mixer_esc_off_4e.value(),
      self.ui.spinBox_mixer_esc_off_5e.value(),
      self.ui.spinBox_mixer_esc_off_6e.value(),
      self.ui.spinBox_mixer_esc_off_7e.value(),
      self.ui.spinBox_mixer_esc_off_8e.value(),
      )
    if self.main.isConnected():
      self.main.devWrite(s, True)

  def change_esc_count(self):
    if not self.inTimerUpdate:
      cf = 0
      if self.ui.radioButton_mixer_x.isChecked():
        print(" x is checked")
        cf = 1
      elif self.ui.radioButton_mixer_plus.isChecked():
        print(" + is checked")
        cf = 2
      elif self.ui.radioButton_mixer_2x2.isChecked():
        print(" 2x2 is checked")
        cf = 3
      else:
        print(" none is checked")
      s = "mixv {:d} {:d}\n".format(self.ui.spinBox_mixer_esc_cnt.value(), cf)
      if self.main.isConnected():
        self.main.devWrite(s, True)
      print(" sending " + s)
      pass
    pass
