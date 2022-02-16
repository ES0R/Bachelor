#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* initialization and GUI window class
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
import time
try:
  import configparser as cp
except:
  import ConfigParser as cp
from PyQt5 import QtWidgets, QtCore, QtGui
from mainwindow import Ui_UGui
#from pyqtgraph.Qt import QtGui, QtCore
from umain import UMain
from udebug import UDebug
from ulog import ULog
from uesc import UMotor
from uacc import UAcc
from upose import UPose
from umag import UMag
from ucontrol import UControl
from ugyro import UGyro
from ualt import UAlt
from urc import URc
from ucontrol import UControl
from uref import URef 
from uusb import UUsb 
from uwifi import UWiFi
from umixer import UMixer

CLIENT_REV = "$Id: drone_gui.py 1289 2021-07-18 17:03:23Z jcan $"

class MainWindow(QtWidgets.QMainWindow):
  def clientVersion(self):
    gg = CLIENT_REV.split()
    return "1." + gg[2]
  def clientVersionDate(self):
    gg = CLIENT_REV.split()
    return gg[3]

  def __init__(self):
        super(MainWindow, self).__init__()
        #
        # create modules
        self.ui = Ui_UGui()
        self.ui.setupUi(self)
        self.debug = UDebug(self, self.ui)
        self.main = UMain(self, self.ui)
        self.log = ULog(self, self.ui)
        self.motor = UMotor(self, self.ui)
        self.acc = UAcc(self, self.ui)
        self.pose = UPose(self, self.ui)
        self.gyro = UGyro(self, self.ui)
        self.mag = UMag(self, self.ui)
        self.alt = UAlt(self, self.ui)
        self.rc = URc(self, self.ui)
        self.ref = URef(self, self.ui)
        self.control = UControl(self, self.ui)
        self.usbio = UUsb(self)
        self.wifi = UWiFi(self)
        self.mixer = UMixer(self)
        # initialize now all modules are loaded
        print("Starting ...")
        self.debug.init()
        self.main.init()
        self.log.init()
        self.motor.init()
        self.acc.init()
        self.pose.init()
        self.rc.init()
        self.gyro.init()
        self.alt.init()
        self.control.init()
        self.ref.init()
        self.usbio.init()
        self.wifi.init()
        self.mixer.init()
        # connect buttons and other actions
        self.main.setStatusBar("Starting ...")
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timerUpdate)
        # timer event every 50 ms
        self.timer.start(50)
        # menu buttons
        self.ui.actionQuit.triggered.connect(self.stop)
        self.ui.actionAbout.triggered.connect(self.menuAbout)
        # view menu
        self.ui.actionDebug.changed.connect(self.menuViewDebug)
        self.ui.actionLogging.changed.connect(self.menuViewLogging)
        self.ui.actionMotor.changed.connect(self.menuViewMotor)
        self.ui.actionGyro.changed.connect(self.menuViewGyro)
        self.ui.actionAcc.changed.connect(self.menuViewAcc)
        self.ui.actionHeight.changed.connect(self.menuViewHeight)
        self.ui.actionPose.changed.connect(self.menuViewPose)
        self.ui.actionControl.changed.connect(self.menuViewControl)
        self.ui.actionMagentometer.changed.connect(self.menuViewMagnetometer)
        self.ui.actionMixer.changed.connect(self.menuViewMixer)
        self.ui.actionRC.changed.connect(self.menuViewRC)
        self.ui.actionManual.changed.connect(self.menuViewRef)
        #
        self.ui.actionHide_most.triggered.connect(self.menuViewHideMost)
        self.ui.actionShow_all.triggered.connect(self.menuViewShowAll)
        # ini
        self.ui.actionSave_to_ini_file.triggered.connect(self.menuSaveToIniFile)
        self.ui.actionLoad_from_ini_file.triggered.connect(self.loadFromIniFile)
        # # load from ini file
        # ensure tabs are in correct order
        self.menuViewShowAll()
        self.menuViewHideMost()
        # load from file - if exist
        self.loadFromIniFile();


  def menuAbout(self):
    # QMessageBox.about (QWidget parent, QString caption, QString text)
    about_box = QtWidgets.QMessageBox(my_mainWindow)
    about_box.setText('''<p><span style=" font-size:20pt;">
               <a href="http://www.dtu.dk">DTU</a>
               <a href="http://www.elektro.dtu.dk"> Elektro</a>
               <a href="http://rsewiki.elektro.dtu.dk/index.php/Drone_control"> Drone control<a></span></p>
               <p><span style=" font-size:10pt;">This is a drone motor test GUI <br> 
               Use Drone control link above for more details.</span></p>
               <p><span style=" font-size:10pt;">Version ''' + self.clientVersion() + ''' (''' + self.clientVersionDate() + ''')</span></p>
               <p><span style=" font-size:10pt;">Teensy version</span></p>
               <p><span style=" font-size:10pt;">''' + self.main.version + ''' (''' + str(self.main.versionDate) + ''')</span></p>
               <p><span style=" font-size:10pt;">(contact: jca@elektro.dtu.dk)</span></p>''');
    about_box.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
    about_box.setWindowTitle("Drone control about")
    about_box.exec_()


  def closeEvent(self, event):
    print("DRONE_GUI: Stopping (close [X])\n")
    self.main.terminate();
    event.accept() # let the window close

  def stop(self):
    print("DRONE_GUI: Stopping (file->quit)\n")
    self.main.terminate();
    QtWidgets.QApplication.quit();


  #def advanceSlider(self):
        ##self.ui.progressBar.setValue(self.ui.progressBar.value() + 1)
        #self.ui.statusbar.showMessage("Arming ...")
        
  def timerUpdate(self):
        #self.ui.statusbar.showMessage("Robot client starting ...", 6000)
        # print("timer update")
        # esc will call the rest
        self.main.timerUpdate()
        
  def getIndex(self, tab):
    i = 0
    if tab == 0:
      return i
    if (self.ui.actionDebug.isChecked()): 
      i = i + 1
    if (tab == 1):
      return i
    if (self.ui.actionLogging.isChecked()):
      i = i + 1
    if (tab == 2):
      return i
    if (self.ui.actionMotor.isChecked()):
      i = i + 1
    if (tab <= 3):
      return i
    if (self.ui.actionGyro.isChecked()):
      i = i + 1
    if (tab <= 4):
      return i
    if (self.ui.actionAcc.isChecked()):
      i = i + 1
    if (tab <= 5):
      return i
    if (self.ui.actionHeight.isChecked()):
      i = i + 1
    if (tab <= 6):
      return i
    if (self.ui.actionMagentometer.isChecked()):
      i = i + 1
    if (tab <= 7):
      return i
    if (self.ui.actionPose.isChecked()):
      i = i + 1
    if (tab <= 8):
      return i
    if (self.ui.actionControl.isChecked()):
      i = i + 1
    if (tab <= 9):
      return i
    if (self.ui.actionMixer.isChecked()):
      i = i + 1
    if (tab <= 10):
      return i
    if (self.ui.actionRC.isChecked()):
      i = i + 1
    if (tab <= 11):
      return i
    if (self.ui.actionRef.isChecked()):
      i = i + 1
    if (tab <= 12):
      return i
    #if (self.ui.actionTiming.isChecked()):
      #i = i + 1
    return i

  def menuShowTab(self, action, order, tab, tabName):
    idx = self.getIndex(order)
    if (action.isChecked()):
      self.ui.tabWidget.insertTab(idx, tab, tabName)
      self.ui.tabWidget.setCurrentIndex(idx)
    else:
      t = self.ui.tabWidget.indexOf(tab)
      if t >= 0:
        self.ui.tabWidget.removeTab(t)
    pass

  def menuViewDebug(self):
    self.menuShowTab(self.ui.actionDebug, 0, self.ui.tab_debug, "Debug")
  def menuViewLogging(self):
    self.menuShowTab(self.ui.actionLogging, 1, self.ui.tab_log, "Logging")
  def menuViewMotor(self):
    self.menuShowTab(self.ui.actionMotor, 2, self.ui.tab_motor, "Motor")
  def menuViewGyro(self):
    self.menuShowTab(self.ui.actionGyro, 3, self.ui.tab_gyro, "Gyro")
  def menuViewAcc(self):
    self.menuShowTab(self.ui.actionAcc, 4, self.ui.tab_acc, "Acc")
  def menuViewHeight(self):
    self.menuShowTab(self.ui.actionHeight, 5, self.ui.tab_height, "Height")
  def menuViewMagnetometer(self):
    self.menuShowTab(self.ui.actionMagentometer, 6, self.ui.tab_mag, "Mag")
  def menuViewPose(self):
    self.menuShowTab(self.ui.actionPose, 7, self.ui.tab_pose, "Pose")
  def menuViewControl(self):
    self.menuShowTab(self.ui.actionControl, 8, self.ui.tab_control, "Control")
  def menuViewMixer(self):
    self.menuShowTab(self.ui.actionMixer, 9, self.ui.tab_mixer, "Mixer")
  def menuViewRC(self):
    self.menuShowTab(self.ui.actionRC, 10, self.ui.tab_RC, "RC")
  def menuViewRef(self):
    self.menuShowTab(self.ui.actionManual, 11, self.ui.tab_ref, "Manual")

  def menuViewShowAll(self):
    #print("# show all")
    self.ui.actionDebug.setChecked(True)
    self.ui.actionLogging.setChecked(True)
    self.ui.actionMotor.setChecked(True)
    self.ui.actionGyro.setChecked(True)
    self.ui.actionAcc.setChecked(True)
    self.ui.actionHeight.setChecked(True)
    self.ui.actionMagentometer.setChecked(True)
    self.ui.actionPose.setChecked(True)
    self.ui.actionControl.setChecked(True)
    self.ui.actionRC.setChecked(True)
    self.ui.actionManual.setChecked(True)
    self.ui.actionMixer.setChecked(True)

  def menuViewHideMost(self):
    #print("# hiding all tabs")
    self.ui.actionDebug.setChecked(True)
    self.ui.actionLogging.setChecked(False)
    self.ui.actionMotor.setChecked(False)
    self.ui.actionGyro.setChecked(False)
    self.ui.actionAcc.setChecked(False)
    self.ui.actionHeight.setChecked(False)
    self.ui.actionMagentometer.setChecked(False)
    self.ui.actionPose.setChecked(False)
    self.ui.actionControl.setChecked(False)
    self.ui.actionRC.setChecked(False)
    self.ui.actionManual.setChecked(False)
    self.ui.actionMixer.setChecked(False)

  def menuSaveToIniFile(self):
    filename = 'drone_ctrl.ini'
    print("# saving to ini-file " + filename)
    # must request data first
    if self.main.isConnected() or True:
      #print("# trying to save to ini-file")
      config = cp.ConfigParser()
      config.add_section('main')
      config.set('main', 'version', self.clientVersion())
      config.set('main', 'version date', self.clientVersionDate())
      config.set('main', 'usbDeviceName', str(self.ui.lineEdit_connect.text()))
      config.set('main', 'wifiDeviceName', str(self.ui.lineEdit_connect_wifi.text()))
      config.set('main', 'connect', str(self.ui.checkBox_connect.isChecked()))
      config.set('main', 'connectWifi', str(self.ui.checkBox_connect_wifi.isChecked()))
      # and then all control parts
      self.control.saveToIniFile(config)
      self.ref.saveToIniFile(config)
      self.debug.saveToIniFile(config)
      self.rc.saveToIniFile(config)
      # menu settings
      config.add_section('menu')
      config.set('menu', 'debug', str(self.ui.actionDebug.isChecked()))
      config.set('menu', 'log', str(self.ui.actionLogging.isChecked()))
      config.set('menu', 'motor', str(self.ui.actionMotor.isChecked()))
      config.set('menu', 'gyro', str(self.ui.actionGyro.isChecked()))
      config.set('menu', 'acc', str(self.ui.actionAcc.isChecked()))
      config.set('menu', 'height', str(self.ui.actionHeight.isChecked()))
      config.set('menu', 'mag', str(self.ui.actionMagentometer.isChecked()))
      config.set('menu', 'pose', str(self.ui.actionPose.isChecked()))
      config.set('menu', 'control', str(self.ui.actionControl.isChecked()))
      config.set('menu', 'rc', str(self.ui.actionRC.isChecked()))
      config.set('menu', 'manual', str(self.ui.actionManual.isChecked()))
      config.set('menu', 'mixer', str(self.ui.actionMixer.isChecked()))
      config.set('menu', 'current', str(self.ui.tabWidget.currentIndex()))
      # save old version - if any
      if os.path.exists(filename):
        newname = 'drone_ctrl_' + time.strftime("%Y%2m%2d_%02H%2M%2S") + '.ini'
        self.main.setStatusBar("# Renamed " + filename + " to " + newname)
        os.rename(filename, newname)
      #
      try:
        with open(filename, 'w') as configFile:
          config.write(configFile)
          self.main.message("# Saved configuration to " + filename);
      except:
        self.main.message("# failed to save configuration to " + filename);
    else:
      print("# will not save configuration with no contact to drone")
      self.main.message("# will not save configuration with no contact to drone: (" + filename + ")");
  
  def loadFromIniFile(self):
    config = cp.ConfigParser()
    filename = 'drone_ctrl.ini'
    print("#Trying to load from " + filename)
    config.read(filename)
    self.debug.loadFromIniFile(config)
    self.ref.loadFromIniFile(config)
    self.rc.loadFromIniFile(config)
    tab = 0
    try:
      #if True:
      self.ui.actionDebug.setChecked(config.getboolean('menu', 'debug'))
      self.ui.actionLogging.setChecked(config.getboolean('menu', 'log'))
      self.ui.actionMotor.setChecked(config.getboolean('menu', 'motor'))
      self.ui.actionGyro.setChecked(config.getboolean('menu', 'gyro'))
      self.ui.actionAcc.setChecked(config.getboolean('menu', 'acc'))
      self.ui.actionHeight.setChecked(config.getboolean('menu', 'height'))
      self.ui.actionMagentometer.setChecked(config.getboolean('menu', 'mag'))
      self.ui.actionPose.setChecked(config.getboolean('menu', 'pose'))
      self.ui.actionControl.setChecked(config.getboolean('menu', 'control'))
      self.ui.actionRC.setChecked(config.getboolean('menu', 'rc'))
      self.ui.actionManual.setChecked(config.getboolean('menu', 'manual'))
      self.ui.actionMixer.setChecked(config.getboolean('menu', 'mixer'))
      # set also the active tab
      tab = config.getint('menu', 'current')
      # connect
      self.usbio.close()
      self.wifi.close()
      self.ui.lineEdit_connect.setText(config.get('main','usbDeviceName'))
      self.ui.lineEdit_connect_wifi.setText(config.get('main','wifiDeviceName'))
      self.ui.checkBox_connect.setChecked(config.getboolean('main', 'connect'))
      self.ui.checkBox_connect_wifi.setChecked(config.getboolean('main', 'connectWifi'))
      self.usbio.close()
      self.wifi.close()
      print("# loaded settings from 'drone_ctrl.ini', usb = " + 
                             str(self.ui.checkBox_connect.isChecked()) + ", wifi = " +
                             str(self.ui.checkBox_connect_wifi.isChecked()))
    except:
      self.main.message("# missing part of drone_ctrl.ini")
      print("# missing part of drone_ctrl.ini")
    #self.configurationFileLoad(config)
    self.ui.tabWidget.setCurrentIndex(tab)
    pass

#
# Main start of APP
#

app = QtWidgets.QApplication(sys.argv)

my_mainWindow = MainWindow()
my_mainWindow.setWindowIcon(QtGui.QIcon("dtulogoicon_123x123.png"))
my_mainWindow.setWindowTitle("drone_GUI")
my_mainWindow.show()

sys.exit(app.exec_())
