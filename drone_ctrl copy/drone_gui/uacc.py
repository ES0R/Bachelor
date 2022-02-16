#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for accelerometer data and calibration
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
import threading
import numpy as np
import pyqtgraph as pg
import time
import timeit
try:
  import configparser
except:
  import ConfigParser  
from PyQt5 import QtWidgets, QtCore

class UAccVal:
  x = 0
  y = 0
  z = 0
  n = 0 # samples
  upd = True
  isOK = False
  label = []
  axis = 0
  #
  def __init__(self, label, axis):
    self.label = label
    self.axis = axis
    self.upd = True
  #
  def check(self):
    v = np.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
    self.isOK = v > 0.9 and v < 1.1
  #
  def isOKcal(self):
    ok = False
    if self.axis == 0: # horizontal
      ok = self.z > 0.9
    elif self.axis == 1: # 90 deg roll
      ok = self.y > 0.9
    elif self.axis == 5: # -90 deg pitch
      ok = self.x > 0.9
    # negative g
    elif self.axis == 2: # roll 180
      ok = self.z < -0.9
    elif self.axis == 3: # -90 roll
      ok = self.y < -0.9
    elif self.axis == 4: # 90 deg pitch
      ok = self.x < -0.9
    return ok
  #
  def clear(self):
    self.x = 0
    self.y = 0
    self.z = 0
    self.isOK = False
    self.upd = True
    self.n = 0
  #
  def setIfBetter(self, acc):
    # positive g
    ok = False
    lm = 0.07;
    ax = abs(acc[0])
    ay = abs(acc[1])
    az = abs(acc[2])
    if self.axis == 0: # horizontal
      ok = ax < lm and ay < lm
    elif self.axis == 1: # 90 deg roll
      ok = ax < lm and az < lm
    elif self.axis == 5: # -90 deg pitch
      ok = ay < lm and az < lm
    # negative g
    elif self.axis == 2: # roll 180
      ok = ax < lm and ay < lm
    elif self.axis == 3: # -90 roll
      ok = ax < lm and az < lm
    elif self.axis == 4: # 90 deg pitch
      ok = ay < lm and az < lm
    else:
      print("# UAccVal::axis error")
    if ok:
      self.x += acc[0]
      self.y += acc[1]
      self.z += acc[2]
      self.n += 1
      self.upd = True
    pass
  #
  def getx(self):
    if  self.n > 0:
      return self.x/self.n
    else:
      return 0
    
  def gety(self):
    if  self.n > 0:
      return self.y/self.n
    else:
      return 0

  def getz(self):
    if  self.n > 0:
      return self.z/self.n
    else:
      return 0

  def timerUpdate(self):
    if self.upd:
      if self.n > 0:
        self.label.setText("({:7.4f}, {:7.4f}, {:7.4f})".format(self.x/self.n, self.y/self.n, self.z/self.n))
      else:
        self.label.setText("(no data)")
      self.upd = False


# ###########################################################################
# ###########################################################################
# ###########################################################################
# ###########################################################################

class UAcc(object):
  # class variables
  ui = []
  parent = []
  main = []
  #
  accDataUpd = True
  acccal = []
  propRotated180 = False
  # current value
  acc = [0,0,0]
  accNew = True
  accOfs = [0,0,0, 1]
  accOfsNew = True
  gyroOfs = [0,0,0]
  # data history
  dataa = np.zeros((3,100))
  dataaIdx = 0
  dataaNew = True
  dataaCg = []
  dataaPw = []
  hasFocus = False
  # board orientation (radians)
  boardXr = 0.0
  boardYr = 0.0
  boardZr = 0.0
  boardNew = False
  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui
    self.main = self.parent.main
    # make list of all 6 calibration values
    self.acccal.append(UAccVal(self.ui.label_acc_horizontal,0))
    self.acccal.append(UAccVal(self.ui.label_acc_roll90, 1)) 
    self.acccal.append(UAccVal(self.ui.label_acc_roll180, 2)) 
    self.acccal.append(UAccVal(self.ui.label_acc_roll270, 3)) 
    self.acccal.append(UAccVal(self.ui.label_acc_pitch90, 4)) 
    self.acccal.append(UAccVal(self.ui.label_acc_pitch270, 5)) 
    #
    # plot widget
    self.dataaPw = pg.PlotWidget(name='acc',title='Accelerometer') 
    self.dataaPw.setLabel('left','acceleration','g')
    self.dataaPw.addLegend()
    self.dataaPw.showGrid(True,True,0.6);
    self.ui.horizontalLayout_acc.addWidget(self.dataaPw)
    self.dataaCg = [self.dataaPw.plot(pen='r',name='x'), self.dataaPw.plot( pen='c',  name='y'), self.dataaPw.plot( pen='g', name='z')]
    self.dataaCg[0].setData(self.dataa[0])
    self.dataaCg[1].setData(self.dataa[1])
    self.dataaCg[2].setData(self.dataa[2])
    # disable invalid buttons
    #self.ui.pushButton_acc_offset_apply.setEnabled(False)

  def init(self):
    self.ui.pushButton_acc_cal_clear.clicked.connect(self.clearCalibFields)
    self.ui.pushButton_acc_cal_apply.clicked.connect(self.sendAccCalibrate)
    self.ui.pushButton_acc_cal_simple.clicked.connect(self.doCalibrate)
    self.ui.pushButton_board_rot_apply.clicked.connect(self.sendBoardRot)
    pass

  def timerUpdate(self, timerCnt):
    for a in self.acccal:
      a.timerUpdate()
    if self.accOfsNew:
      self.ui.doubleSpinBox_acc_x_offset.setValue(self.accOfs[0])
      self.ui.doubleSpinBox_acc_y_offset.setValue(self.accOfs[1])
      self.ui.doubleSpinBox_acc_z_offset.setValue(self.accOfs[2])
      self.accOfsNew = False
    # plot of acc
    if self.accNew:
      self.ui.doubleSpinBox_acc_x.setValue(self.acc[0])
      self.ui.doubleSpinBox_acc_y.setValue(self.acc[1])
      self.ui.doubleSpinBox_acc_z.setValue(self.acc[2])
      self.dataaCg[0].setData(self.dataa[0])
      self.dataaCg[1].setData(self.dataa[1])
      self.dataaCg[2].setData(self.dataa[2])
      self.accNew = False
    if self.boardNew:
      self.ui.doubleSpinBox_board_x_rot2.setValue(self.boardXr * 180 / np.pi)
      self.ui.doubleSpinBox_board_y_rot2.setValue(self.boardYr * 180 / np.pi)
      self.ui.doubleSpinBox_board_z_rot2.setValue(self.boardZr * 180 / np.pi)
      self.boardNew = False
    if not self.main.isConnected():
      # act as leaving the page
      self.hasFocus = False
    if self.hasFocus and self.ui.tabWidget.currentIndex() != self.ui.tabWidget.indexOf(self.ui.tab_acc):
      # just leaving this tab
      self.hasFocus = False
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe off
        #self.main.devWrite("sub unhandled 0\n") # acc offset
        self.main.devWrite("sub acc 0\n") # acc data
      pass
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == self.ui.tabWidget.indexOf(self.ui.tab_acc):
      # just entering this tab
      self.hasFocus = True
      if self.main.wifi.isOpen():
        # we are talking to a bridge - so subscribe
        #self.main.devWrite("sub unhandled -1 0\n") # acc offset
        self.main.devWrite("sub acc -1 30\n") # acc data
        pass
    if self.hasFocus:
      if timerCnt % 3 == 1:
        self.main.devWrite("imu a\n", True) # imu actual data
      if timerCnt % 30 == 7:
        self.main.devWrite("imucal a\n", True) # imu calibration offsets - also acc
      if timerCnt % 30 == 0:
        self.checkAccCal()
      if timerCnt % 30 == 10:
        self.main.devWrite("brd\n", True) # imu calibration offsets - also acc
    pass 
  
  
  def decode(self, gg):
    isOK = True
    if gg[0] == "accoff":
      self.accOfs[0] = float(gg[1])
      self.accOfs[1] = float(gg[2])
      self.accOfs[2] = float(gg[3])
      if len(gg) > 4:
        self.propRotated180 = float(gg[4])
      self.accOfsNew = True
    elif gg[0] == "acc":
      self.acc[0] = float(gg[1])
      self.acc[1] = float(gg[2])
      self.acc[2] = float(gg[3])
      self.dataa[0,self.dataaIdx] = self.acc[0]
      self.dataa[1,self.dataaIdx] = self.acc[1]
      self.dataa[2,self.dataaIdx] = self.acc[2]
      self.dataaIdx += 1
      if self.dataaIdx >= 100:
        self.dataaIdx = 0
      # update calibration fields (if selected)
      if self.ui.checkBox_acc_horizontal.isChecked() and self.acc[2] > 0.5:
        self.acccal[0].setIfBetter(self.acc)
      elif self.ui.checkBox_acc_roll90.isChecked() and self.acc[1] > 0.5:
        self.acccal[1].setIfBetter(self.acc)
      elif self.ui.checkBox_acc_roll180.isChecked() and self.acc[2] < -0.5:
        self.acccal[2].setIfBetter(self.acc)
      elif self.ui.checkBox_acc_roll270.isChecked() and self.acc[1] < -0.5:
        self.acccal[3].setIfBetter(self.acc)
      elif self.ui.checkBox_acc_pitch90.isChecked() and self.acc[0] < -0.5:
        self.acccal[4].setIfBetter(self.acc)
      elif self.ui.checkBox_acc_pitch270.isChecked() and self.acc[0] > 0.5:
        self.acccal[5].setIfBetter(self.acc)
      self.accNew = True
      pass
    elif gg[0] == "brdr": # board rotation
      self.boardXr = float(gg[1])
      self.boardYr = float(gg[2])
      self.boardZr = float(gg[3])
      self.boardNew = True
    else:
      isOK = False
    return isOK

  def clearCalibFields(self):
    if self.ui.checkBox_acc_horizontal.isChecked():
      self.acccal[0].clear()
    if self.ui.checkBox_acc_roll90.isChecked():
      self.acccal[1].clear()
    if self.ui.checkBox_acc_roll180.isChecked():
      self.acccal[2].clear()
    if self.ui.checkBox_acc_roll270.isChecked():
      self.acccal[3].clear()
    if self.ui.checkBox_acc_pitch90.isChecked():
      self.acccal[4].clear()
    if self.ui.checkBox_acc_pitch270.isChecked():
      self.acccal[5].clear()

  #def sendAccClearOffset(self):
    #self.main.devWrite("imuzero a\n", True)
    

  def checkAccCal(self):
    ok = True
    for a in self.acccal:
      ok &= a.isOKcal()
    self.ui.pushButton_acc_cal_apply.setEnabled(ok)

  def doCalibrate(self):
    # simple calibration
    self.main.devWrite("offsetcal a\n", True)

  def sendAccCalibrate(self):
    # send more advanced calibration after full rotation on all axes
    # send (and set) calibration result of 6-value calibration
    # offset
    aox = (self.acccal[5].getx() + self.acccal[4].getx())/2.0;
    aoy = (self.acccal[1].gety() + self.acccal[3].gety())/2.0;
    aoz = (self.acccal[0].getz() + self.acccal[2].getz())/2.0;
    # scale
    asx = (self.acccal[5].getx() - self.acccal[4].getx())/2.0;
    asy = (self.acccal[1].gety() - self.acccal[3].gety())/2.0;
    asz = (self.acccal[0].getz() - self.acccal[2].getz())/2.0;
    self.main.devWrite("imucal A {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}\n".format(aox, aoy, aoz, 1/asx, 1/asy, 1/asz), True)
    print("# acc offset {:.5f} {:.5f} {:.5f}".format(aox, aoy, aoz))
    print("# acc scale  {:.5f} {:.5f} {:.5f}".format(asx, asy, asz))

  def sendBoardRot(self):
    self.main.devWrite("board {:f} {:f} {:f}\n".format(self.ui.doubleSpinBox_board_x_rot.value()/180*np.pi, 
                                                       self.ui.doubleSpinBox_board_y_rot.value()/180*np.pi, 
                                                       self.ui.doubleSpinBox_board_z_rot.value()/180*np.pi), True)
