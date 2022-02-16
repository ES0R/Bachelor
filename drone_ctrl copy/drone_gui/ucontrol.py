#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for control loop settings
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
from time import sleep
#import ConfigParser
import timeit
try:
  import configparser
except:
  import ConfigParser  
from ucontrol_edit import UControlUnit
from PyQt5 import QtWidgets, QtCore


class UControl(object):
  # class variables
  parent = []
  ui = []
  main = []

  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui
    self.main = self.parent.main

  def init(self):
    self.ctrlRollVel = UControlUnit("vroll", self.parent, "Roll rate control")
    self.ctrlRoll = UControlUnit("roll", self.parent, "Roll angle control")
    self.ctrlPitchVel = UControlUnit("vpitch", self.parent, "Pitch rate control")
    self.ctrlPitch = UControlUnit("pitch", self.parent, "Pitch angle control")
    self.ctrlYawVel = UControlUnit("vyaw", self.parent, "Yaw rate control")
    self.ctrlYaw = UControlUnit("yaw", self.parent, "Yaw angle control")
    self.ctrlHeight = UControlUnit("height", self.parent, "Height control")
    self.ctrlRollVel.init()
    self.ctrlRoll.init()
    self.ctrlPitchVel.init()
    self.ctrlPitch.init()
    self.ctrlYawVel.init()
    self.ctrlYaw.init()
    self.ctrlHeight.init()
    self.ui.pushButton_ctrl_roll_vel.clicked.connect(self.ctrlRollVel.editControlValues)
    self.ui.pushButton_ctrl_roll.clicked.connect(self.ctrlRoll.editControlValues)
    self.ui.pushButton_ctrl_pitch_vel.clicked.connect(self.ctrlPitchVel.editControlValues)
    self.ui.pushButton_ctrl_pitch.clicked.connect(self.ctrlPitch.editControlValues)
    self.ui.pushButton_ctrl_yaw_vel.clicked.connect(self.ctrlYawVel.editControlValues)
    self.ui.pushButton_ctrl_yaw.clicked.connect(self.ctrlYaw.editControlValues)
    self.ui.pushButton_ctrl_height.clicked.connect(self.ctrlHeight.editControlValues)
    pass

  def timerUpdate(self, timerCnt):
    self.ctrlRollVel.timerUpdate()
    self.ctrlRoll.timerUpdate()
    self.ctrlPitchVel.timerUpdate()
    self.ctrlPitch.timerUpdate()
    self.ctrlYawVel.timerUpdate()
    self.ctrlYaw.timerUpdate()
    self.ctrlHeight.timerUpdate()
    if timerCnt % 10 == 0:
      pass
    if self.ui.tabWidget.currentIndex() == self.ui.tabWidget.indexOf(self.ui.tab_control):
      # no data in this tab, just buttons
      # self.main.message("control timer update")
      pass
    pass 
  
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "control":
        if self.ctrlRollVel.fromString(gg):
          pass
        elif self.ctrlRoll.fromString(gg):
          pass
        elif self.ctrlPitchVel.fromString(gg):
          pass
        elif self.ctrlPitch.fromString(gg):
          pass
        elif self.ctrlYawVel.fromString(gg):
          pass
        elif self.ctrlYaw.fromString(gg):
          pass
        elif self.ctrlHeight.fromString(gg):
          pass
        else:
          isOK = False
    else:
      isOK = False
    return isOK

  def requestControlData(self):
    self.main.devWrite("ctrl roll\n", True)
    self.main.devWrite("ctrl vroll\n", True)
    self.main.devWrite("ctrl pitch\n", True)
    self.main.devWrite("ctrl vpitch\n", True)
    self.main.devWrite("ctrl yaw\n", True)
    self.main.devWrite("ctrl vyaw\n", True)
    self.main.devWrite("ctrl height\n", True)
    time.sleep(0.1);

  def saveToIniFile(self, config):
    # fetch data from teensy
    self.requestControlData()
    self.ctrlRollVel.configurationFileSave(config)
    self.ctrlRoll.configurationFileSave(config)
    self.ctrlPitchVel.configurationFileSave(config)
    self.ctrlPitch.configurationFileSave(config)
    self.ctrlYawVel.configurationFileSave(config)
    self.ctrlYaw.configurationFileSave(config)
    self.ctrlHeight.configurationFileSave(config)

