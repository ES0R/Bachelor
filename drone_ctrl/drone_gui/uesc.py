#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   Class for motor control and test-bed measurements
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
try:
  import configparser
except:
  import ConfigParser    
from PyQt5 import QtWidgets, QtCore
import timeit
import os.path
#from pathlib import Path
import datetime

class UMotor(object):
  # class variables
  rps = [0,0]
  ccv = True
  rpsRead = False
  escManual = [False, False, False, False, False, False, False, False]
  escVal = [1, 2, 3, 4, 5, 6, 7, 8]
  escVel = [10, 10, 10, 10, 10, 10, 10, 10]
  escData = False
  escPWMfrq = 0
  tmp1 = 0
  tmp2 = 0
  tmpData = True
  tmpPi = 22.0;
  tmpPiData = True
  batteryAmps = 7.0
  batteryVolts = 7.0
  liPoCellCnt = 0
  onBattery = True
  batteryData = True
  parent = []
  ui = []
  main = []
  ampsSensorInstalled = False
  tempSensorInstalled = False
  rpmSensorInstalled = False
  installedInEdit = False

  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui

  def init(self):
    self.main = self.parent.main
    self.ui.pushButton_data_apply.clicked.connect(self.data_apply)
    self.ui.pushButton_data_save_to_file.clicked.connect(self.data_save)
    self.ui.checkBox_esc_on_battery.clicked.connect(self.on_battery)
    self.ui.checkBox_esc_temp_installed.clicked.connect(self.sensor_installed_x)
    self.ui.checkBox_esc_RPM_installed.clicked.connect(self.sensor_installed_x)
    self.ui.checkBox_esc_current_installed.clicked.connect(self.sensor_installed_x)
    self.ui.pushButton_esc_save_installed.clicked.connect(self.saveInstalledSensors)
    pass
  
  def saveInstalledSensors(self):
    self.main.devWrite("sensi {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_current_installed.isChecked(), self.ui.checkBox_esc_temp_installed.isChecked(), self.ui.checkBox_esc_RPM_installed.isChecked()))
    self.ui.pushButton_esc_save_installed.setEnabled(False)
    self.installedInEdit = False
    #print("# installed saved")

  def sensor_installed_x(self):
    if not self.inTimerUpdate:
      self.installedInEdit = True
      self.ui.pushButton_esc_save_installed.setEnabled(True)
      self.sensor_installed()
      #print("# installed clicked {:d} {:d} {:d}".format(self.ui.checkBox_esc_current_installed.isChecked(), self.ui.checkBox_esc_temp_installed.isChecked(), self.ui.checkBox_esc_RPM_installed.isChecked()))
  
  def sensor_installed(self):
    # current sensor installed or not
    ai = self.ui.checkBox_esc_current_installed.isChecked()
    self.ui.label_esc_10.setVisible(ai)
    self.ui.label_esc_17.setVisible(ai)
    self.ui.lineEdit_esc_current.setVisible(ai)
    self.ui.lineEdit_esc_watt.setEnabled(ai)
    # RPM sensor installed
    ri = self.ui.checkBox_esc_RPM_installed.isChecked()
    self.ui.lineEdit_esc_RPS_1.setVisible(ri)
    self.ui.lineEdit_esc_RPS_2.setVisible(ri)
    self.ui.lineEdit_esc_RPS_CCV.setVisible(ri)
    self.ui.label_esc_13.setVisible(ri)
    self.ui.label_esc_23.setVisible(ri)
    self.ui.label_esc_24.setVisible(ri)
    self.ui.label_esc_29.setVisible(ri)
    # temp sensor installed
    ti = self.ui.checkBox_esc_temp_installed.isChecked()
    self.ui.label_esc_15.setVisible(ti)
    self.ui.label_esc_16.setVisible(ti)
    self.ui.label_esc_21.setVisible(ti)
    self.ui.label_esc_22.setVisible(ti)
    self.ui.lineEdit_esc_temp_motor.setVisible(ti)
    self.ui.lineEdit_esc_temp_esc.setVisible(ti)
    # Raspberry temperature
    rt = self.main.wifiOn
    self.ui.label_esc_20.setVisible(rt)
    self.ui.label_esc_27.setVisible(rt)
    self.ui.lineEdit_esc_temp_pi.setVisible(rt)


  def timerUpdate(self, timerCnt):
    # set widgets and ask for new data
    self.inTimerUpdate = True
    thisTab = self.ui.tabWidget.indexOf(self.ui.tab_motor)
    if not self.main.isConnected():
      # act as leaving the page
      self.hasFocus = False
    if self.hasFocus and self.ui.tabWidget.currentIndex() != thisTab:
      # just leaving this tab
      self.hasFocus = False
      self.installedInEdit = False
      self.ui.pushButton_esc_save_installed.setEnabled(False)
    if not self.hasFocus and self.ui.tabWidget.currentIndex() == thisTab:
      # just entering this tab
      self.hasFocus = True
      self.ui.pushButton_esc_save_installed.setEnabled(False)
      self.ui.checkBox_esc_current_installed.setChecked(self.ampsSensorInstalled)
      self.ui.checkBox_esc_temp_installed.setChecked(self.tempSensorInstalled)
      self.ui.checkBox_esc_RPM_installed.setChecked(self.rpmSensorInstalled)
      self.sensor_installed()
    if self.hasFocus:
      # update fields in GUI
      if self.rpsRead:
        self.ui.lineEdit_esc_RPS_1.setText("{:.1f}".format(self.rps[0]))
        self.ui.lineEdit_esc_RPS_2.setText("{:.1f}".format(self.rps[1]))
        if self.ccv:
          self.ui.lineEdit_esc_RPS_CCV.setText("CCV")
        else:
          self.ui.lineEdit_esc_RPS_CCV.setText("CV")
        if not self.installedInEdit:
          self.ui.checkBox_esc_RPM_installed.setChecked(self.rpmSensorInstalled)
        self.rpsRead = False
      if self.tmpData:
        self.ui.lineEdit_esc_temp_motor.setText("{:.1f}".format(self.tmp2))
        self.ui.lineEdit_esc_temp_esc.setText("{:.1f}".format(self.tmp1))
        if not self.installedInEdit:
          self.ui.checkBox_esc_temp_installed.setChecked(self.tempSensorInstalled)
        self.tmpData = False
      if self.tmpPiData:
        self.ui.lineEdit_esc_temp_pi.setText("{:.1f}".format(self.tmpPi))
        self.tmpPiData = False
      if self.batteryData:
        self.ui.lineEdit_esc_voltage.setText("{:.2f}".format(self.batteryVolts))
        self.ui.lineEdit_esc_current.setText("{:.2f}".format(self.batteryAmps))
        self.ui.lineEdit_esc_watt.setText("{:.1f}".format(self.batteryAmps * self.batteryVolts))
        self.ui.lineEdit_esc_lipo_cells.setText("{:d}".format(self.liPoCellCnt))
        self.ui.checkBox_esc_on_battery.setChecked(self.onBattery)
        if self.installedInEdit == False:
          self.ui.checkBox_esc_current_installed.setChecked(self.ampsSensorInstalled)
        self.batteryData = False
      if self.escData:
        self.ui.lineEdit_esc_1.setText(str(int(self.escVal[0])))
        self.ui.lineEdit_esc_2.setText(str(int(self.escVal[1])))
        self.ui.lineEdit_esc_3.setText(str(int(self.escVal[2])))
        self.ui.lineEdit_esc_4.setText(str(int(self.escVal[3])))
        self.ui.lineEdit_esc_5.setText(str(int(self.escVal[4])))
        self.ui.lineEdit_esc_6.setText(str(int(self.escVal[5])))
        self.ui.lineEdit_esc_7.setText(str(int(self.escVal[6])))
        self.ui.lineEdit_esc_8.setText(str(int(self.escVal[7])))
        self.escData = False
      if timerCnt % 20 == 8:
        self.main.devWrite("sensor\n", True) # rotation sensor
      if timerCnt % 20 == 15:
        self.main.devWrite("esi\n", True)    # ESC control data
      if timerCnt % 20 == 17:
        self.main.devWrite("temp\n", True)   # motor and esc temp
      if timerCnt % 20 == 19:
        self.main.devWrite("temp\n", False)   # motor and esc temp
      if timerCnt % 20 == 5:
        self.main.devWrite("batt\n", True)   # battery voltage and current
    self.inTimerUpdate = False
    pass 
  
  def data_apply(self):
    vel = self.ui.spinBox_esc_velocity.value()
    T = self.ui.spinBox_esc_velocity_T.value()
    #print("apply")
    self.main.devWrite("esc 0 {:d} {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_1.isChecked(), int(self.ui.spinBox_esc_PWM_1.value()), int(vel), int(T)), True)
    self.main.devWrite("esc 1 {:d} {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_2.isChecked(), int(self.ui.spinBox_esc_PWM_2.value()), int(vel), int(T)), True)
    self.main.devWrite("esc 2 {:d} {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_3.isChecked(), int(self.ui.spinBox_esc_PWM_3.value()), int(vel), int(T)), True)
    self.main.devWrite("esc 3 {:d} {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_4.isChecked(), int(self.ui.spinBox_esc_PWM_4.value()), int(vel), int(T)), True)
    self.main.devWrite("esc 4 {:d} {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_5.isChecked(), int(self.ui.spinBox_esc_PWM_5.value()), int(vel), int(T)), True)
    self.main.devWrite("esc 5 {:d} {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_6.isChecked(), int(self.ui.spinBox_esc_PWM_6.value()), int(vel), int(T)), True)
    self.main.devWrite("esc 6 {:d} {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_7.isChecked(), int(self.ui.spinBox_esc_PWM_7.value()), int(vel), int(T)), True)
    self.main.devWrite("esc 7 {:d} {:d} {:d} {:d}\n".format(self.ui.checkBox_esc_8.isChecked(), int(self.ui.spinBox_esc_PWM_8.value()), int(vel), int(T)), True)
    pass
  
  def data_save(self):
    # save measurement data (when in propeller test stand)
    fn = self.ui.lineEdit_data_file.text()
    ff = os.path.isfile(fn) 
    if ff:
      mode = 'a'
      print("saving - append")
    else:
      mode = 'w'
      print("saving - create")
    if True: # try:
      fil = open(fn, mode)
      if not ff:
        # write header
        fil.write("% Measurement from esc,motor,propeller test\r\n")
        fil.write("% file created " + str(datetime.datetime.now()) + "\r\n")
        fil.write("% 1: esc value (motor 1) 0=1ms, 1024 = 2ms\r\n")
        fil.write("% 2: rps (motor 1) a rotations per second)\r\n")
        fil.write("% 3: rps (motor 1) b rotations per second)\r\n")
        fil.write("% 4: Motor voltage (volt)\r\n")
        fil.write("% 5: total current (amps)\r\n")
        fil.write("% 6: trust force (gram force)\r\n")
        fil.write("% 7: CCV (rotation direction)\r\n")
        fil.write("% 8: Temperature motor (deg C)\r\n")
        fil.write("% 9: Temperature ESC (deg C)\r\n")
      fil.write("{:d} {:.2f} {:.2f} {:s} {:s} {:s} {:d} {:.1f} {:.1f}\n".format(
                                      self.escVal[0], 
                                      self.rps[0], 
                                      self.rps[1], 
                                      self.ui.lineEdit_esc_voltage.text(),
                                      self.ui.lineEdit_esc_current.text(),
                                      self.ui.lineEdit_esc_trust.text(),
                                      int(self.ccv),
                                      self.tmp2, self.tmp1
                                      ))
    #except:
      #self.main.message("Failed to open file " + fn)
    fil.close()

  def decode(self, gg):
    "sen rps(%g %g), idle limit: %d,\r\n"
    " adc time=%ldus, cycle: %ldus, pin1: %ld - %ld = %ld pin2: %ld - %ld = %ld\n\r", 
    #rps[0], rps[1], spanLimit, 
    #adcConvertUs, adcIntervalUs,
    #rpsMax[0], rpsMin[0], rpsMax[0] - rpsMin[0] , 
    #rpsMax[1], rpsMin[1], rpsMax[1] - rpsMin[1]);
    isOK = gg[0] == "sen"
    if isOK:
      try:
        self.rps[0] = float(gg[1])
        self.rps[1] = float(gg[2])
        self.ccv = gg[3] == 0
        self.rpmSensorInstalled = int(gg[4]) == 1 # 
        self.rpsRead = True
      except:
        print("# 'sen' (sensor) decode error");
    elif gg[0] == "esd":
      isOK = True
      self.escPWMfrq = int(gg[1])
      j = 2
      try:
        for i in range(0,8):
          self.escManual[i] = gg[j] == 1
          self.escVal[i] = int(gg[j+1])
          self.escVel[i] = int(gg[j+2])
          j += 3
      except:
        print("# uesc.py::decode: too few values in 'esd' message")
      self.escData = True
    elif gg[0] == "tmp":
      isOK = True
      self.tmp1 = float(gg[1]) # AD 19 (ESC)
      self.tmp2 = float(gg[2]) # AD 20 (motor)
      self.tempSensorInstalled = int(gg[3]) == 1 # 
      self.tmpData = True
    elif gg[0] == "tmppi":
      isOK = True
      self.tmpPi = float(gg[1]) # from mission app
      self.tmpPiData = True
    elif gg[0] == "bat":
      isOK = True
      self.batteryVolts = float(gg[1]) # Voltage (volts)
      self.batteryAmps  = float(gg[2]) # Current (amps)
      self.liPoCellCnt = int(gg[3]) # calculated from voltage
      self.onBattery = int(gg[4]) # drone is on battery
      self.ampsSensorInstalled = int(gg[5]) == 1 # current sensor installed
      self.batteryData = True
    return isOK

  def on_battery(self):
    self.main.devWrite("onbat {:d}\n".format(self.ui.checkBox_esc_on_battery.isChecked()), True) # request RC values
    pass
