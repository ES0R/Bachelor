#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2021 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   Class for debug tab
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
  import configparser as cp
except:
  import ConfigParser as cp
  pass
import timeit
from PyQt5 import QtWidgets, QtCore

class UDebug(object):
  # class variables
  mainStatus  = "Drone GUI\n main debug status\n"
  mainStatusSet = True
  dataRxCntOld = 0
  oldTab = -1

  # methods / functions
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui

  def init(self):
    # make a read thread to take care of infut from Teensy
    #self.stop = threading.Event()
    #self.doRead = threading.Thread(target=self.readThread, name="regbot_usb_reader")
    #self.doRead.start()
    ## the rest
    self.main = self.parent.main
    self.usbio = self.parent.usbio
    self.wifi = self.parent.wifi
    self.ui.pushButton_debug_send.clicked.connect(self.debugSend)
    self.ui.pushButton_debug_send_2.clicked.connect(self.debugSend2)
    self.ui.pushButton_debug_send_3.clicked.connect(self.debugSend3)
    self.ui.pushButton_debug_help.clicked.connect(self.debugHelp)
    self.ui.pushButton_debug_help_teensy.clicked.connect(self.debugHelpT)
    self.ui.pushButton_debug_clear.clicked.connect(self.clear)
    self.ui.pushButton_factory_reset.clicked.connect(self.setIdZero)
    pass

  def timerUpdate(self, timerCnt):
    if self.mainStatusSet:
      if (len(self.mainStatus) > 1000000):
        self.mainStatus = "# status truncated\n"
      self.ui.textEdit_debug_text.setPlainText(str(self.mainStatus))
      self.mainStatusSet = False
      self.ui.textEdit_debug_text.verticalScrollBar().setValue(self.ui.textEdit_debug_text.verticalScrollBar().maximum())
    #if self.main.dataRxCnt != self.dataRxCntOld:
      #self.ui.label_debug_rx_cnt.setText("(rx) " + str(self.main.dataRxCnt) + " lines")
      #self.ui.label_debug_tx_cnt.setText("(tx) " + str(self.main.dataTxCnt) + " lines")
      #self.dataRxCntOld = self.main.dataRxCnt
    if timerCnt % 20 == 0 or self.usbio.dataRxCnt != self.dataRxCntOld:
      self.ui.label_message_cnt.setText("tx: USB " + str(self.usbio.dataTxCnt) + 
                                        " net " + str(self.wifi.txCnt) + 
                                        ", rx: USB " + str(self.usbio.dataRxCnt) + 
                                        " net " + str(self.wifi.rxCnt) + 
                                        " lines (loop " + str(timerCnt) + ")")
      self.dataRxCntOld = self.usbio.dataRxCnt
    if self.oldTab != self.ui.tabWidget.currentIndex():
      if self.ui.tabWidget.currentIndex() == 0:
        # getting focus
        self.mainStatus += "----\r\n"
        self.mainStatusSet = True;
      self.oldTab = self.ui.tabWidget.currentIndex()
      pass
    pass 
  
  def debugSend(self):
    s = str(self.ui.lineEdit_debug_command.text()) + "\n"
    #print(s)
    self.main.devWrite(s)
    pass
  
  def debugSend2(self):
    s = str(self.ui.lineEdit_debug_command_2.text()) + "\n"
    #print(s)
    self.main.devWrite(s)
    pass
  
  def debugSend3(self):
    s = str(self.ui.lineEdit_debug_command_3.text()) + "\n"
    #print(s)
    self.main.devWrite(s)
    pass

  def debugHelp(self):
    self.main.devWrite("help\n")
    pass

  def debugHelpT(self):
    self.main.devWrite("help\n", True)
    pass

  def clear(self):
    self.mainStatus = ""
    self.mainStatusSet = True

  def saveToIniFile(self, config):
    # settings
    config.add_section('debug')
    config.set('debug', 'show#', str(self.ui.checkBox_debug_show_all_hash.isChecked()))
    config.set('debug', 'showTx', str(self.ui.checkBox_debug_show_all_tx.isChecked()))
    config.set('debug', 'showRx', str(self.ui.checkBox_debug_show_all_rx.isChecked()))
    config.set('debug', 'showHbt', str(self.ui.checkBox_debug_show_hbt.isChecked()))
    config.set('debug', 'cmd1', str(self.ui.lineEdit_debug_command.text()))
    config.set('debug', 'cmd2', str(self.ui.lineEdit_debug_command_2.text()))
    config.set('debug', 'cmd3', str(self.ui.lineEdit_debug_command_3.text()))

  def loadFromIniFile(self, config):
    config = cp.SafeConfigParser()
    config.read("drone_ctrl.ini")
    try:
      self.ui.checkBox_debug_show_all_hash.setChecked(config.getboolean('debug', 'show#'))
      self.ui.checkBox_debug_show_all_tx.setChecked(config.getboolean('debug', 'showTx'))
      self.ui.checkBox_debug_show_all_rx.setChecked(config.getboolean('debug', 'showRx'))
      self.ui.checkBox_debug_show_hbt.setChecked(config.getboolean('debug', 'showHbt'))
      self.ui.lineEdit_debug_command.setText(config.get('debug', 'cmd1'))
      self.ui.lineEdit_debug_command_2.setText(config.get('debug', 'cmd2'))
      self.ui.lineEdit_debug_command_3.setText(config.get('debug', 'cmd3'))
    except:
      self.main.message("# failed to load debug from ini-file")
    pass
  
  def setIdZero(self):
    self.main.devWrite("setid 0", True);
    self.main.message("Remember 'save to flash'")
    
