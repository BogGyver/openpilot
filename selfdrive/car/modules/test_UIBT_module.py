import time
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
    

cs = CarState()

while 1:
    cs.UE.update_custom_ui()
    CS.cstm_btns.send_button_info()
    time.sleep(0.1)