import os
import time
from typing import Dict

from cereal import car,log
from common.kalman.simple_kalman import KF1D
from common.realtime import DT_CTRL
from selfdrive.car import gen_empty_fingerprint
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from selfdrive.controls.lib.events import Events
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.modules.CFG_module import read_config_file,load_bool_param
from selfdrive.car.modules.HSO_module import HSOController
from selfdrive.car.modules.BLNK_module import BLNKController
from selfdrive.car.modules.ALC_module import ALCController
import cereal.messaging as messaging

GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
MAX_CTRL_SPEED = (V_CRUISE_MAX + 4) * CV.KPH_TO_MS  # 135 + 4 = 86 mph

# generic car and radar interfaces


class CarInterfaceBase():
  def __init__(self, CP, CarController, CarState):
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.low_speed_alert = False

    if CarState is not None:
      self.CS = CarState(CP)
      self.cp = self.CS.get_can_parser(CP)
      self.cp_cam = self.CS.get_cam_can_parser(CP)
      self.cp_body = self.CS.get_body_can_parser(CP)
    
      #initialize modules
      self.CS.HSO = HSOController()
      self.CS.blinker_controller = BLNKController()
      self.CS.alca_controller = ALCController()

      #initialize listeners
      self.CS.laP = messaging.sub_sock('lateralPlan')

      #alca info
      self.CS.prev_alca_pre_engage = False
      self.CS.alca_pre_engage = False
      self.CS.alca_engaged = False
      self.CS.alca_direction = 0
      self.CS.alca_need_engagement = False
      self.CS.alca_done = False

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP, self.VM)

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.

  @staticmethod
  def compute_gb(accel, speed):
    raise NotImplementedError

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    raise NotImplementedError

  # returns a set of default params to avoid repetition in car specific params
  @staticmethod
  def get_std_params(candidate, fingerprint):
    ret = car.CarParams.new_message()
    ret.carFingerprint = candidate

    # standard ALC params
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerMaxBP = [0.]
    ret.steerMaxV = [1.]
    ret.minSteerSpeed = 0.

    # stock ACC by default
    ret.enableCruise = True
    ret.minEnableSpeed = -1.  # enable is done by stock ACC, so ignore this
    ret.steerRatioRear = 0.  # no rear steering, at least on the listed cars aboveA
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [.5]  # half max brake
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [1.]
    ret.openpilotLongitudinalControl = False
    ret.startAccel = 0.0
    ret.minSpeedCan = 0.3
    ret.stoppingBrakeRate = 0.2 # brake_travel/s while trying to stop
    ret.startingBrakeRate = 0.8 # brake_travel/s while releasing on restart
    ret.stoppingControl = False
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [1.]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [1.]
    return ret

  # returns a car.CarState, pass in car.CarControl
  def update(self, c, can_strings):
    raise NotImplementedError

  def post_update(self,c):
    if self.CS.enableHAO:
      self.CS.out.gas = 0
      self.CS.out.gasPressed = False
    #Trick the alca if autoStartAlcaDelay is set
    if (self.CS.enableALC) and (self.CS.alca_need_engagement):
      self.CS.out.steeringPressed = True
      if self.CS.alca_direction == log.LateralPlan.LaneChangeDirection.left:
        self.CS.out.steeringTorque = self.CS.out.steeringTorque + 0.1
      if self.CS.alca_direction == log.LateralPlan.LaneChangeDirection.right:
        self.CS.out.steeringTorque = self.CS.out.steeringTorque - 0.1

  def pre_apply(self,c):
    #read params once a second
    if self.frame % 100 == 0:
      self.CS.enableHSO = load_bool_param("TinklaHso",True)
      self.CS.enableHAO = load_bool_param("TinklaHao",False)
      self.CS.enableALC = load_bool_param("TinklaAlc",False)

    self.CS.lat_plan = messaging.recv_one_or_none(self.CS.laP) 
    # Update HSO module
    self.CS.human_control = self.CS.HSO.update_stat(self.CS, c.enabled, c.actuators, self.frame)
    
    #update blinker tap module
    self.CS.blinker_controller.update_state(self.CS, c.frame)
    self.CS.tap_direction = self.CS.blinker_controller.tap_direction

    #update ALC module
    self.CS.alca_controller.update(c.enabled, self.CS, c.frame, self.CS.lat_plan)

  # return sendcan, pass in a car.CarControl

  def apply(self, c):
    raise NotImplementedError

  def create_common_events(self, cs_out, extra_gears=[], gas_resume_speed=-1, pcm_enable=True):  # pylint: disable=dangerous-default-value
    events = Events()

    if cs_out.doorOpen:
      events.add(EventName.doorOpen)
    if cs_out.seatbeltUnlatched:
      events.add(EventName.seatbeltNotLatched)
    if cs_out.gearShifter != GearShifter.drive and cs_out.gearShifter not in extra_gears:
      events.add(EventName.wrongGear)
    if cs_out.gearShifter == GearShifter.reverse:
      events.add(EventName.reverseGear)
    if not cs_out.cruiseState.available:
      events.add(EventName.wrongCarMode)
    if cs_out.espDisabled:
      events.add(EventName.espDisabled)
    if cs_out.gasPressed:
      events.add(EventName.gasPressed)
    if cs_out.stockFcw:
      events.add(EventName.stockFcw)
    if cs_out.stockAeb:
      events.add(EventName.stockAeb)
    if cs_out.vEgo > MAX_CTRL_SPEED:
      events.add(EventName.speedTooHigh)
    if cs_out.cruiseState.nonAdaptive:
      events.add(EventName.wrongCruiseMode)

    if cs_out.steerError:
      events.add(EventName.steerUnavailable)
    elif cs_out.steerWarning:
      if cs_out.steeringPressed:
        events.add(EventName.steerTempUnavailableUserOverride)
      else:
        events.add(EventName.steerTempUnavailable)

    # Disable on rising edge of gas or brake. Also disable on brake when speed > 0.
    # Optionally allow to press gas at zero speed to resume.
    # e.g. Chrysler does not spam the resume button yet, so resuming with gas is handy. FIXME!
    if (cs_out.gasPressed and (not self.CS.out.gasPressed) and cs_out.vEgo > gas_resume_speed) or \
       (cs_out.brakePressed and (not self.CS.out.brakePressed or not cs_out.standstill)):
      events.add(EventName.pedalPressed)

    # we engage when pcm is active (rising edge)
    if pcm_enable:
      if cs_out.cruiseState.enabled and not self.CS.out.cruiseState.enabled:
        events.add(EventName.pcmEnable)
      elif not cs_out.cruiseState.enabled:
        events.add(EventName.pcmDisable)

    return events


class RadarInterfaceBase():
  def __init__(self, CP):
    self.pts = {}
    self.delay = 0
    self.radar_ts = CP.radarTimeStep
    self.no_radar_sleep = 'NO_RADAR_SLEEP' in os.environ

  def update(self, can_strings):
    ret = car.RadarData.new_message()
    if not self.no_radar_sleep:
      time.sleep(self.radar_ts)  # radard runs on RI updates
    return ret


class CarStateBase:
  def __init__(self, CP):
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.out = car.CarState.new_message()

    self.cruise_buttons = 0
    self.left_blinker_cnt = 0
    self.right_blinker_cnt = 0

    #start config section
    self.forcePedalOverCC = load_bool_param("TinklaEnablePedal",False)
    self.useFollowModeAcc = load_bool_param("TinklaUseFollowACC",False)
    self.autoresumeAcc = load_bool_param("TinklaAutoResumeACC",False)
    self.enableHSO = load_bool_param("TinklaHso",True)
    self.enableHAO = load_bool_param("TinklaHao",False)
    self.enableALC = load_bool_param("TinklaAlc",False)
    self.useTeslaRadar = load_bool_param("TinklaUseTeslaRadar",False)
    self.usesApillarHarness = load_bool_param("TinklaUseAPillarHarness",False)
    self.autoStartAlcaDelay = 2
    self.radarVIN = '"                 "'
    self.radarOffset = 0.0
    self.radarEpasType = 0
    self.radarPosition = 0
    self.hsoNumbPeriod = 1.5

    read_config_file(self)
    #end config section


    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, DT_CTRL], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])

  def update_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_ego_raw], [0.0]]

    v_ego_x = self.v_ego_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])

  def update_blinker(self, blinker_time: int, left_blinker_lamp: bool, right_blinker_lamp: bool):
    self.left_blinker_cnt = blinker_time if left_blinker_lamp else max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = blinker_time if right_blinker_lamp else max(self.right_blinker_cnt - 1, 0)
    return self.left_blinker_cnt > 0, self.right_blinker_cnt > 0

  @staticmethod
  def parse_gear_shifter(gear: str) -> car.CarState.GearShifter:
    d: Dict[str, car.CarState.GearShifter] = {
        'P': GearShifter.park, 'R': GearShifter.reverse, 'N': GearShifter.neutral,
        'E': GearShifter.eco, 'T': GearShifter.manumatic, 'D': GearShifter.drive,
        'S': GearShifter.sport, 'L': GearShifter.low, 'B': GearShifter.brake
    }
    return d.get(gear, GearShifter.unknown)

  @staticmethod
  def get_cam_can_parser(CP):
    return None

  @staticmethod
  def get_body_can_parser(CP):
    return None
