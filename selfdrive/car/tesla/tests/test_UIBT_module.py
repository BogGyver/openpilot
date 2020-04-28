#!/usr/bin/env python3 

import time
import sys
import unittest
from selfdrive.car.modules.UIBT_module import UIButtons
from selfdrive.car.tesla.ACC_module import ACCMode
from selfdrive.car.tesla.PCC_module import PCCModes
from selfdrive.car.modules.UIEV_module import UIEvents

class CarState():
  def __init__(self):
    # labels for buttons
    self.btns_init = [["alca",                "ALC",                      ["MadMax", "Normal", "Calm"]],
                      [PCCModes.BUTTON_NAME,   PCCModes.BUTTON_ABREVIATION, PCCModes.labels()],
                      ["dsp",               "DSP",                      ["OP","MIN","OFF","GYRO"]],
                      ["",               "",                      [""]],
                      ["msg",                 "MSG",                      [""]],
                      ["sound",               "SND",                      [""]]]
    self.UE = UIEvents(self)
    self.cstm_btns = UIButtons(self,"Tesla Model S","tesla", False, False)
    self.custom_alert_counter = 0
    for i in range(6):
        print (self.cstm_btns.btns[i].btn_name,len(self.cstm_btns.btns[i].btn_name))
        print (self.cstm_btns.btns[i].btn_label,len(self.cstm_btns.btns[i].btn_label))
        print (self.cstm_btns.btns[i].btn_label2,len(self.cstm_btns.btns[i].btn_label2))
        print (self.cstm_btns.btns[i].btn_status)

class UIBT_Tests(unittest.TestCase):

  def setUp(self):
    self.cs = CarState()

  def tearDown(self):
    print("Testing complete")

  # Tests continuous messaging for 10 seconds
  def test_defaults_buttons(self):
    i = 0
    while i < 100:
      self.cs.UE.update_custom_ui()
      self.cs.cstm_btns.send_button_info()
      time.sleep(0.1)
      i = i + 1

if __name__ == '__main__':
  i=0
  continuous = False
  if len(sys.argv) == 2:
      if sys.argv[1] == '-cont':
          continuous = True
          cs = CarState()
          while (i<100) or continuous:
              cs.UE.update_custom_ui()
              cs.cstm_btns.send_button_info()
              time.sleep(0.1)
              i = i + 1
      else:
          print ("for continuous test use %s -cont" % sys.argv[0])
          exit(0)
  else:
      unittest.main()
