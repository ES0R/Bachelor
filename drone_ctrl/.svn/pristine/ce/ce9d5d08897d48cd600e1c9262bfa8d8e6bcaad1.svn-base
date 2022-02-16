#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #*   class for editing one full PID controller
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
import timeit
try:
  import configparser
except:
  import ConfigParser  
from dialog_control import Ui_dialog_control
from PyQt5 import QtWidgets, QtCore


# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////

class UControlPZ(object):
  use = False
  tauNum = 0.0
  tauDen = 0.0
  def setTauNumTauDen(self, doUse, tNum, tDen):
    self.use = doUse
    self.tauNum = tNum
    self.tauDen = tDen
  def setLead(self, doUse, tau, alpha):
    self.use = doUse
    self.tauNum = tau
    self.tauDen = tau * alpha
  def setPole(self, doUse, tauPole):
    self.use = doUse
    self.tauNum = 0
    self.tauDen = tauPole
  def asString(self):
    return " {:d} {:g} {:g}".format(self.use, self.tauNum, self.tauDen)
  def fromString(self, gg1, gg2, gg3):
    #print("# control PZ")
    self.use = int(gg1, 0)
    self.tauNum = float(gg2)
    self.tauDen = float(gg3)
    #print("# control PZ ud")
  pass

# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////

class UControlI(object):
  use = False
  tau = 0.0
  limit = 0.0
  andZero = True
  def setIntegrator(self, doUse, taui, ilimit, zero):
    self.use = doUse
    self.tau = taui
    self.limit = ilimit
    # if andZero is false, then the input is not added to output of integrator
    self.andZero = zero
  #def setIntegratorKi(self, doUse, ki, ilimit):
    #self.use = doUse
    #if ki > 0.0:
      #self.tau = 1/ki
    #self.limit = ilimit
  def asString(self):
    return " {:d} {:g} {:g} {:d}".format(self.use, self.tau, self.limit, self.andZero)
  def fromString(self, gg1, gg2, gg3, gg4):
    #print("# control I " + gg1 + " " + gg2 + " " + gg3 + " " + gg4)
    self.use = int(gg1, 0)
    self.tau = float(gg2)
    self.limit = float(gg3)
    self.andZero = int(gg4, 0)

# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////

# class that combines QT-design setup and a GUI-dialog class
class FormControllerEdit(QtWidgets.QDialog, Ui_dialog_control):
  def __init__(self, parent):
    QtWidgets.QWidget.__init__(self, parent)
    self.setupUi(self)
    self.checkBox_integrator_use.clicked.connect(self.check_integrator)
    self.checkBox_lead_fwd.setChecked(True)
    self.checkBox_lead_fwd.clicked.connect(self.check_lead_fwd)
    self.checkBox_lead_back.clicked.connect(self.check_lead_back)
    self.checkBox_pre_use.clicked.connect(self.check_pre_use)
    self.checkBox_out_limit.clicked.connect(self.check_out_limit)
    self.checkBox_ff_use.clicked.connect(self.check_ff_use)
    self.checkBox_ff_use_filt.clicked.connect(self.check_ff_use)
    
    
  def check_integrator(self):
    checked = self.checkBox_integrator_use.isChecked()
    self.edit_tau_i.setVisible(checked)
    self.label_tau_i.setVisible(checked)
    self.label_integrator_limit.setVisible(checked)
    self.edit_integrator_limit.setVisible(checked)
  def check_lead_fwd(self):
    self.label_tau_zero_fwd.setVisible(self.checkBox_lead_fwd.isChecked())
    self.label_tau_pole_fwd.setVisible(self.checkBox_lead_fwd.isChecked())
    self.edit_tau_pole_fwd.setVisible(self.checkBox_lead_fwd.isChecked())
    self.edit_tau_zero_fwd.setVisible(self.checkBox_lead_fwd.isChecked())
  def check_lead_back(self):
    self.label_tau_zero_back.setVisible(self.checkBox_lead_back.isChecked())
    self.label_tau_pole_back.setVisible(self.checkBox_lead_back.isChecked())
    self.edit_tau_pole_back.setVisible(self.checkBox_lead_back.isChecked())
    self.edit_tau_zero_back.setVisible(self.checkBox_lead_back.isChecked())
  def check_pre_use(self):
    checked = self.checkBox_pre_use.isChecked()
    self.label_pre_tau_pole.setVisible(checked)
    self.label_pre_tau_zero.setVisible(checked)
    self.edit_pre_tau_zero.setVisible(checked)
    self.edit_pre_tau_pole.setVisible(checked) 
  def check_out_limit(self):
    checked = self.checkBox_out_limit.isChecked()
    self.label_out_limit.setVisible(checked)
    self.label_out_limit_2.setVisible(checked)
    self.edit_output_limit_min.setVisible(checked)
    self.edit_output_limit_max.setVisible(checked)
  def check_ff_use(self):
    checked = self.checkBox_ff_use.isChecked()
    if not checked:
      self.checkBox_ff_use_filt.setChecked(False)
    checkedFilt = self.checkBox_ff_use_filt.isChecked()
    self.label_ff_tau_pole.setVisible(checkedFilt)
    self.label_ff_tau_zero.setVisible(checkedFilt)
    self.edit_ff_tau_zero.setVisible(checkedFilt)
    self.edit_ff_tau_pole.setVisible(checkedFilt)
    self.label_ff_Kff.setVisible(checked)
    self.edit_ff_Kff.setVisible(checked)
    self.checkBox_ff_use_filt.setVisible(checked)

# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////


class UControlUnit(object):
  # class variables
  parent = []
  ui = []
  main = []
  ctrlID = ""
  use = False
  Kp = 0.0
  Kff = 0.0
  ffUse = False
  outLimit_min = -10.0
  outLimit_max = 10.0
  outLimitUse = True
  needUpdate = False
  name = "description"
  plusMinusPiCheck = False
  useesDiffInput = False;


  # methods / functions
  def __init__(self, id, parent, name):
    self.parent = parent
    self.ui = parent.ui
    self.main = self.parent.main
    self.name = name
    self.ctrlID = id

  def init(self):
    print("## init " + self.name)
    self.editDlg = FormControllerEdit(self.parent)
    self.integrator = UControlI()
    self.leadFwd = UControlPZ()
    self.leadBack = UControlPZ()
    self.preFilt = UControlPZ()
    self.ffFilt = UControlPZ()
    self.editDlg.pushButton_ctrlRetry.clicked.connect(self.button_retry_clicked)
    self.editDlg.pushButton_ctrlIniLoad.clicked.connect(self.button_ini_load_clicked)
    self.editDlg.buttonBox.button(QtWidgets.QDialogButtonBox.Apply).clicked.connect(self.configuration_to_robot)
    self.editDlg.buttonBox.button(QtWidgets.QDialogButtonBox.Cancel).clicked.connect(self.buttonCancelClicked)
    self.editDlg.buttonBox.button(QtWidgets.QDialogButtonBox.Ok).clicked.connect(self.buttonOkClicked)
    pass

  def timerUpdate(self):
    #print("-- timer update")
    if self.needUpdate:
      #print("---- timer update 1")
      self.fillDialog()
      self.needUpdate = False
  
  
  #def decode(self, gg):
    #isOK = True
    #if gg[0] == "control":
      #self.main.message("# @todo control")
      #print("# unhandled control")
    #else:
      #isOK = False
    #return isOK

  def buttonCancelClicked(self):
    # update dialog fields from memory
    self.needUpdate = True
    pass
  
  def buttonOkClicked(self):
    self.setFromDialog()
    self.configuration_to_robot()
    pass

  def button_retry_clicked(self):
    self.main.devWrite("ctrl " + self.ctrlID + "\n", True)
    #print("Send: " + "ctrl " + self.ctrlID)
    #self.fillDialog()
    #print("#filled dialog")
    pass

  def button_ini_load_clicked(self):
    config = configparser.SafeConfigParser()
    config.read("drone_ctrl.ini")
    self.configurationFileLoad(config)
    pass

  def asString(self):
    s = (self.ctrlID + 
         " {:d} {:g}".format(self.use, self.Kp) + 
         self.integrator.asString() +
         self.leadFwd.asString() +
         self.leadBack.asString() + 
         self.preFilt.asString() + 
         " {:d} {:g} {:g}".format(self.outLimitUse, self.outLimit_min, self.outLimit_max) +
         " {:d} {:g}".format(self.ffUse, self.Kff) + 
         self.ffFilt.asString() +
         " {:d}".format(self.plusMinusPiCheck)
        )
    return s
  
  
  def fromString(self, gg):
    isOK = gg[1] == self.ctrlID and len(gg) > 17
    if isOK:
      #self.lock.acquire()
      print("# control decode " + self.ctrlID + " has len={:d} expected 27".format(len(gg)))
      try:
        self.use = gg[2] != '0'
        self.Kp = float(gg[3])
        self.integrator.fromString(gg[4], gg[5], gg[6], gg[7])
        self.leadFwd.fromString(gg[8], gg[9], gg[10])
        self.leadBack.fromString(gg[11], gg[12], gg[13])
        self.preFilt.fromString(gg[14], gg[15], gg[16])
        self.outLimitUse = gg[17] != '0'
        self.outLimit_min = float(gg[18])
        self.outLimit_max = float(gg[19])
        self.ffUse = gg[20] != '0'
        self.Kff = float(gg[21])
        self.ffFilt.fromString(gg[22], gg[23], gg[24])
        self.plusMinusPiCheck = int(gg[25])
        self.useesDiffInput = int(gg[26])
      except:
        print("# control " + self.ctrlID + " failed data read from regbot, len={:d} expected 27".format(len(gg)))
      self.needUpdate = True
    return isOK

  def fillDialog(self):
    if True: # try:
      #print("fill in dialog")
      self.editDlg.label_ID_lead.setText("Controller parameters (" + self.ctrlID + ")")
      self.editDlg.label_ID.setText(self.name )
      self.editDlg.ctrl_main_use.setChecked(self.use)
      self.editDlg.edit_kp.setValue(self.Kp)
      self.editDlg.ctrl_pm_pi_check.setChecked(self.plusMinusPiCheck)
      # i-term
      self.editDlg.checkBox_integrator_use.setChecked(self.integrator.use)
      self.editDlg.edit_tau_i.setValue(self.integrator.tau)
      self.editDlg.edit_integrator_limit.setValue(self.integrator.limit)
      self.editDlg.check_integrator()
      # lead forward pole-zero term
      self.editDlg.checkBox_lead_fwd.setChecked(self.leadFwd.use)
      self.editDlg.edit_tau_zero_fwd.setValue(self.leadFwd.tauNum)
      self.editDlg.edit_tau_pole_fwd.setValue(self.leadFwd.tauDen)
      self.editDlg.check_lead_fwd()
      # lead backward pole-zero term
      self.editDlg.checkBox_lead_back.setChecked(self.leadBack.use)
      self.editDlg.edit_tau_zero_back.setValue(self.leadBack.tauNum)
      self.editDlg.edit_tau_pole_back.setValue(self.leadBack.tauDen)
      if self.useesDiffInput:
        self.editDlg.label_tau_pole_back.setText("Diff_pole")
      else:
        self.editDlg.label_tau_pole_back.setText("Tau_pole")
      self.editDlg.check_lead_back()
      # prefilter pole-zero term
      self.editDlg.checkBox_pre_use.setChecked(self.preFilt.use)
      self.editDlg.edit_pre_tau_zero.setValue(self.preFilt.tauNum)
      self.editDlg.edit_pre_tau_pole.setValue(self.preFilt.tauDen)
      self.editDlg.check_pre_use()
      # output limit
      self.editDlg.checkBox_out_limit.setChecked(self.outLimitUse)
      self.editDlg.edit_output_limit_min.setValue(self.outLimit_min)
      self.editDlg.edit_output_limit_max.setValue(self.outLimit_max)
      self.editDlg.check_out_limit()
      # feed forward + ff pole-zero term
      #print(" ffUse = " + str(self.ffUse) + ", ffFilt = " + str(self.ffFilt.use))
      self.editDlg.checkBox_ff_use.setChecked(int(self.ffUse))
      #print(" ffUse checked = " + str(self.editDlg.checkBox_ff_use.isChecked()))
      self.editDlg.edit_ff_Kff.setValue(self.Kff)
      self.editDlg.checkBox_ff_use_filt.setChecked(self.ffFilt.use)
      self.editDlg.checkBox_ff_use.setChecked(self.ffUse)
      self.editDlg.edit_ff_tau_zero.setValue(self.ffFilt.tauNum)
      self.editDlg.edit_ff_tau_pole.setValue(self.ffFilt.tauDen)
    try:
      self.editDlg.check_ff_use()
    except:
      print("failed to set values in dialog")
      exc_type, exc_obj, exc_tb = sys.exc_info()
      print(exc_type, exc_tb.tb_lineno)
      pass
    pass

  
  def configuration_to_robot(self):
    self.setFromDialog()
    self.main.devWrite("ctrl " + self.asString() + "\n", True)
    pass

  def setFromDialog(self):
    self.use = self.editDlg.ctrl_main_use.isChecked()
    self.Kp = self.editDlg.edit_kp.value()
    self.plusMinusPiCheck = self.editDlg.ctrl_pm_pi_check.isChecked()
    # i-term
    self.integrator.use = self.editDlg.checkBox_integrator_use.isChecked()
    self.integrator.tau = self.editDlg.edit_tau_i.value()
    self.integrator.limit = self.editDlg.edit_integrator_limit.value()
    # lead in forward branch
    self.leadFwd.use = self.editDlg.checkBox_lead_fwd.isChecked()
    self.leadFwd.tauNum = self.editDlg.edit_tau_zero_fwd.value()
    self.leadFwd.tauDen = self.editDlg.edit_tau_pole_fwd.value()
    # lead in feedback branch
    self.leadBack.use = self.editDlg.checkBox_lead_back.isChecked()
    self.leadBack.tauNum = self.editDlg.edit_tau_zero_back.value()
    self.leadBack.tauDen = self.editDlg.edit_tau_pole_back.value()
    # prefilter
    self.preFilt.use = self.editDlg.checkBox_pre_use.isChecked()
    self.preFilt.tauNum = self.editDlg.edit_pre_tau_zero.value()
    self.preFilt.tauDen = self.editDlg.edit_pre_tau_pole.value()
    # output limit
    self.outLimitUse = self.editDlg.checkBox_out_limit.isChecked()
    self.outLimit_min = self.editDlg.edit_output_limit_min.value()
    self.outLimit_max = self.editDlg.edit_output_limit_max.value()
    # feed forward
    self.ffUse = self.editDlg.checkBox_ff_use.isChecked()
    self.Kff = self.editDlg.edit_ff_Kff.value()
    self.ffFilt.use = self.editDlg.checkBox_ff_use_filt.isChecked()
    self.ffFilt.tauNum = self.editDlg.edit_ff_tau_zero.value()
    self.ffFilt.tauDen = self.editDlg.edit_ff_tau_pole.value()
    

  def requestData(self):
    self.main.devWrite("ctrl " + self.ctrlID + "\n", True)
    print("# requesting data for " + self.ctrlID)

  def editControlValues(self):
    self.requestData()
    print("# load dialog for " + self.ctrlID)
    self.needUpdate = True
    self.editDlg.show()
    pass
  #
  # save configuration to regbot.ini 
  def configurationFileSave(self, config):
    config.add_section(self.ctrlID)
    config.set(self.ctrlID, 'name', self.name)
    config.set(self.ctrlID, 'use', str(self.use))
    config.set(self.ctrlID, 'kp', str(self.Kp))
    config.set(self.ctrlID, 'pmPIcheck', str(self.plusMinusPiCheck))
    config.set(self.ctrlID, 'i_Use', str(self.integrator.use))
    config.set(self.ctrlID, 'i_tau', str(self.integrator.tau))
    config.set(self.ctrlID, 'i_limit', str(self.integrator.limit))
    config.set(self.ctrlID, 'lead_fwd_use', str(self.leadFwd.use))
    config.set(self.ctrlID, 'lead_fwd_tau_zero', str(self.leadFwd.tauNum))
    config.set(self.ctrlID, 'lead_fwd_tau_pole', str(self.leadFwd.tauDen))
    config.set(self.ctrlID, 'lead_back_use', str(self.leadBack.use))
    config.set(self.ctrlID, 'lead_back_tau_zero', str(self.leadBack.tauNum))
    config.set(self.ctrlID, 'lead_back_tau_pole', str(self.leadBack.tauDen))
    config.set(self.ctrlID, 'pre_filt_use', str(self.preFilt.use))
    config.set(self.ctrlID, 'pre_filt_tau_zero', str(self.preFilt.tauNum))
    config.set(self.ctrlID, 'pre_filt_tau_pole', str(self.preFilt.tauDen))
    config.set(self.ctrlID, 'out_limit_use', str(self.outLimitUse))
    config.set(self.ctrlID, 'out_limit_min', str(self.outLimit_min))
    config.set(self.ctrlID, 'out_limit_max', str(self.outLimit_max))
    config.set(self.ctrlID, 'ff_use', str(self.ffUse))
    config.set(self.ctrlID, 'Kff', str(self.Kff))
    config.set(self.ctrlID, 'ff_use_filt', str(self.ffFilt.use))
    config.set(self.ctrlID, 'ff_filt_tau_zero', str(self.ffFilt.tauNum))
    config.set(self.ctrlID, 'ff_filt_tau_pole', str(self.ffFilt.tauDen))
  #
  # load configuration for regot.ini
  def configurationFileLoad(self, config):
    try:
      self.use = config.getboolean(self.ctrlID, 'use')
      self.Kp = config.getfloat(self.ctrlID, 'kp')
      self.plusMinusPiCheck = config.getboolean(self.ctrlID, 'pmPIcheck')
      self.integrator.use = config.getboolean(self.ctrlID, 'i_use')
      self.integrator.tau = config.getfloat(self.ctrlID, 'i_tau')
      self.integrator.limit = config.getfloat(self.ctrlID, 'i_limit')
      self.leadFwd.use = config.getboolean(self.ctrlID, 'lead_fwd_use')
      self.leadFwd.tauNum = config.getfloat(self.ctrlID, 'lead_fwd_tau_zero')
      self.leadFwd.tauDen = config.getfloat(self.ctrlID, 'lead_fwd_tau_pole')
      self.leadBack.use = config.getboolean(self.ctrlID, 'lead_back_use')
      self.leadBack.tauNum = config.getfloat(self.ctrlID, 'lead_back_tau_zero')
      self.leadBack.tauDen = config.getfloat(self.ctrlID, 'lead_back_tau_pole')
      self.preFilt.use = config.getboolean(self.ctrlID, 'pre_filt_use')
      self.preFilt.tauNum = config.getfloat(self.ctrlID, 'pre_filt_tau_zero')
      self.preFilt.tauDen = config.getfloat(self.ctrlID, 'pre_filt_tau_pole')
      self.outLimitUse = config.getboolean(self.ctrlID, 'out_limit_use')
      self.outLimit_min = config.getfloat(self.ctrlID, 'out_limit_min')
      self.outLimit_max = config.getfloat(self.ctrlID, 'out_limit_max')
      self.ffFilt.use = config.getboolean(self.ctrlID, 'ff_use_filt')
      self.ffUse = config.getboolean(self.ctrlID, 'ff_use')
      self.Kff = config.getfloat(self.ctrlID, 'Kff')
      self.ffFilt.tauNum = config.getfloat(self.ctrlID, 'ff_filt_tau_zero')
      self.ffFilt.tauDen = config.getfloat(self.ctrlID, 'ff_filt_tau_pole')
      print("# loaded control data from ini file")
      self.needUpdate = True
    except:
      print("Missed config file info for " + self.ctrlID + " or one of its elements - continues")
