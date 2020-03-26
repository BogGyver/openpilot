#!/usr/bin/env python
from cereal import car, tesla
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET, get_events
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.tesla.values import CruiseButtons, CM, BP, AH, CAR,DBC
from common.params import read_db
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.tesla.readconfig import CarSettings
import cereal.messaging as messaging
from cereal.services import service_list
from selfdrive.controls.lib.planner import _A_CRUISE_MAX_V
from selfdrive.car.interfaces import CarInterfaceBase

A_ACC_MAX = max(_A_CRUISE_MAX_V)
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
VisualAlert = car.CarControl.HUDControl.VisualAlert

K_MULT = 0.8 
K_MULTi = 280000.



class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.CP = CP

    self.frame = 0
    self.last_enable_pressed = 0
    self.last_enable_sent = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.can_invalid_count = 0
    

    # *** init the major players ***
    self.CS = CarState(CP)
    self.VM = VehicleModel(CP)
    mydbc = DBC[CP.carFingerprint]['pt']
    if CP.carFingerprint == CAR.MODELS and self.CS.fix1916:
      mydbc = mydbc + "1916"
    self.cp = self.CS.get_can_parser2(CP,mydbc)
    self.epas_cp = None
    self.pedal_cp = None
    if self.CS.useWithoutHarness:
      self.epas_cp = self.CS.get_epas_parser(CP,0)
      self.pedal_cp = self.CS.get_pedal_parser(CP,0)
    else:
      self.epas_cp = self.CS.get_epas_parser(CP,2)
      self.pedal_cp = self.CS.get_pedal_parser(CP,2)

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name,CP,self.VM)


  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.



  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    # limit the pcm accel cmd if:
    # - v_ego exceeds v_target, or
    # - a_ego exceeds a_target and v_ego is close to v_target

    # normalized max accel. Allowing max accel at low speed causes speed overshoots
    max_accel_bp = [10, 20]    # m/s
    max_accel_v = [0.714, 1.0] # unit of max accel
    max_accel = interp(v_ego, max_accel_bp, max_accel_v)

    eA = a_ego - a_target
    valuesA = [1.0, 0.1]
    bpA = [0.3, 1.1]

    eV = v_ego - v_target
    valuesV = [1.0, 0.1]
    bpV = [0.0, 0.5]

    valuesRangeV = [1., 0.]
    bpRangeV = [-1., 0.]

    # only limit if v_ego is close to v_target
    speedLimiter = interp(eV, bpV, valuesV)
    accelLimiter = max(interp(eA, bpA, valuesA), interp(eV, bpRangeV, valuesRangeV))

    # accelOverride is more or less the max throttle allowed to pcm: usually set to a constant
    # unless aTargetMax is very high and then we scale with it; this help in quicker restart

    return float(max(max_accel, a_target / A_ACC_MAX)) * min(speedLimiter, accelLimiter)

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):

    # Scaled tire stiffness
    ts_factor = 8 

    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    ret.carName = "tesla"
    ret.carFingerprint = candidate

    teslaModel = read_db('/data/params','TeslaModel')
    if teslaModel is not None:
      teslaModel = teslaModel.decode()
    if teslaModel is None:
      teslaModel = "S"

    ret.safetyModel = car.CarParams.SafetyModel.tesla
    ret.safetyParam = 1
    ret.carVin = "TESLAFAKEVIN12345"

    ret.enableCamera = True
    ret.enableGasInterceptor = False #keep this False for now
    print ("ECU Camera Simulated: ", ret.enableCamera)
    print ("ECU Gas Interceptor: ", ret.enableGasInterceptor)

    ret.enableCruise = not ret.enableGasInterceptor

    mass_models = 4722./2.205 + STD_CARGO_KG
    wheelbase_models = 2.959
    # RC: I'm assuming center means center of mass, and I think Model S is pretty even between two axles
    centerToFront_models = wheelbase_models * 0.5 #BB was 0.48
    centerToRear_models = wheelbase_models - centerToFront_models
    rotationalInertia_models = 2500
    tireStiffnessFront_models = 85100 #BB was 85400
    tireStiffnessRear_models = 90000
    # will create Kp and Ki for 0, 20, 40, 60 mph
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 8.94, 17.88, 26.82 ], [0., 8.94, 17.88, 26.82]]
    if candidate == CAR.MODELS:
      stop_and_go = True
      ret.mass = mass_models
      ret.wheelbase = wheelbase_models
      ret.centerToFront = centerToFront_models
      ret.steerRatio = 11.5
      # Kp and Ki for the lateral control for 0, 20, 40, 60 mph
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[1.20, 0.80, 0.60, 0.30], [0.16, 0.12, 0.08, 0.04]]
      ret.lateralTuning.pid.kf = 0.00006 # Initial test value TODO: investigate FF steer control for Model S?
      ret.steerActuatorDelay = 0.2

      #ret.steerReactance = 1.0
      #ret.steerInductance = 1.0
      #ret.steerResistance = 1.0
          
      # Kp and Ki for the longitudinal control
      if teslaModel == "S":
        ret.longitudinalTuning.kpBP = [0., 5., 35.]
        ret.longitudinalTuning.kpV = [0.50, 0.45, 0.4]
        ret.longitudinalTuning.kiBP = [0., 5., 35.]
        ret.longitudinalTuning.kiV = [0.01,0.01,0.01]
      elif teslaModel == "SP":
        ret.longitudinalTuning.kpBP = [0., 5., 35.]
        ret.longitudinalTuning.kpV = [0.375, 0.325, 0.325]
        ret.longitudinalTuning.kiBP = [0., 5., 35.]
        ret.longitudinalTuning.kiV = [0.00915,0.00825,0.00725]
      elif teslaModel == "SD":
        ret.longitudinalTuning.kpBP = [0., 5., 35.]
        ret.longitudinalTuning.kpV = [0.50, 0.45, 0.4]
        ret.longitudinalTuning.kiBP = [0., 5., 35.]
        ret.longitudinalTuning.kiV = [0.01,0.01,0.01]
      elif teslaModel == "SPD":
        ret.longitudinalTuning.kpBP = [0., 5., 35.]
        ret.longitudinalTuning.kpV = [0.375, 0.325, 0.325]
        ret.longitudinalTuning.kiBP = [0., 5., 35.]
        ret.longitudinalTuning.kiV = [0.00915,0.00825,0.00725]
      else:
        #use S numbers if we can't match anything
        ret.longitudinalTuning.kpBP = [0., 5., 35.]
        ret.longitudinalTuning.kpV = [0.375, 0.325, 0.3]
        ret.longitudinalTuning.kiBP = [0., 5., 35.]
        ret.longitudinalTuning.kiV = [0.08,0.08,0.08]
      

    else:
      raise ValueError("unsupported car %s" % candidate)

    ret.steerControlType = car.CarParams.SteerControlType.angle

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter. Otherwise, add 0.5 mph margin to not
    # conflict with PCM acc
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else 25.5 * CV.MPH_TO_MS

    centerToRear = ret.wheelbase - ret.centerToFront
    # TODO: get actual value, for now starting with reasonable value for Model S
    ret.rotationalInertia = rotationalInertia_models * \
                            ret.mass * ret.wheelbase**2 / (mass_models * wheelbase_models**2)

    # TODO: start from empirically derived lateral slip stiffness and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront = (tireStiffnessFront_models * ts_factor) * \
                             ret.mass / mass_models * \
                             (centerToRear / ret.wheelbase) / (centerToRear_models / wheelbase_models)
    ret.tireStiffnessRear = (tireStiffnessRear_models * ts_factor) * \
                            ret.mass / mass_models * \
                            (ret.centerToFront / ret.wheelbase) / (centerToFront_models / wheelbase_models)

    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.

    # no max steer limit VS speed
    ret.steerMaxBP = [0.,15.]  # m/s
    ret.steerMaxV = [420.,420.]   # max steer allowed

    ret.gasMaxBP = [0.]  # m/s
    ret.gasMaxV = [0.3] #if ret.enableGasInterceptor else [0.] # max gas allowed
    ret.brakeMaxBP = [0., 20.]  # m/s
    ret.brakeMaxV = [1., 1.]   # max brake allowed - BB: since we are using regen, make this even

    ret.longitudinalTuning.deadzoneBP = [0., 9.] #BB: added from Toyota to start pedal work; need to tune
    ret.longitudinalTuning.deadzoneV = [0., 0.] #BB: added from Toyota to start pedal work; need to tune; changed to 0 for now

    ret.stoppingControl = True
    ret.openpilotLongitudinalControl = True
    ret.steerLimitAlert = False
    ret.startAccel = 0.5
    ret.steerRateCost = 1.0

    ret.radarOffCan = not CarSettings().get_value("useTeslaRadar")
    ret.radarTimeStep = 0.05 #20Hz

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    canMonoTimes = []

    self.cp.update_strings(can_strings)
    ch_can_valid = self.cp.can_valid
    self.epas_cp.update_strings(can_strings)
    epas_can_valid = self.epas_cp.can_valid
    self.pedal_cp.update_strings(can_strings)
    pedal_can_valid = self.pedal_cp.can_valid

    can_rcv_error = not (ch_can_valid and epas_can_valid and pedal_can_valid)

    self.CS.update(self.cp, self.epas_cp, self.pedal_cp)

    # create message
    ret = car.CarState.new_message()
    ret.canValid = ch_can_valid #and epas_can_valid #and pedal_can_valid
    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gas pedal, we don't use with with interceptor so it's always 0/False
    ret.gas = self.CS.user_gas 
    if not self.CP.enableGasInterceptor:
      ret.gasPressed = self.CS.user_gas_pressed
    else:
      ret.gasPressed = self.CS.user_gas_pressed

    # brake pedal
    ret.brakePressed =False # (self.CS.brake_pressed != 0) and (self.CS.cstm_btns.get_button_status("brake") == 0)
    # FIXME: read sendcan for brakelights
    brakelights_threshold = 0.1
    ret.brakeLights = bool(self.CS.brake_switch or
                           c.actuators.brake > brakelights_threshold)

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate

    # gear shifter lever
    ret.gearShifter = self.CS.gear_shifter

    ret.steeringTorque = self.CS.steer_torque_driver
    ret.steeringPressed = self.CS.steer_override

    # cruise state
    ret.cruiseState.enabled = True #self.CS.pcm_acc_status != 0
    ret.cruiseState.speed = self.CS.v_cruise_pcm * CV.KPH_TO_MS * (CV.MPH_TO_KPH if self.CS.imperial_speed_units else 1.)
    ret.cruiseState.available = bool(self.CS.main_on)
    ret.cruiseState.speedOffset = 0.
    ret.cruiseState.standstill = False

    # TODO: button presses
    buttonEvents = []
    ret.leftBlinker = bool(self.CS.turn_signal_state_left == 1)
    ret.rightBlinker = bool(self.CS.turn_signal_state_right == 1)


    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    if self.CS.prev_turn_signal_stalk_state != self.CS.turn_signal_stalk_state:
      if self.CS.turn_signal_stalk_state == 1 or self.CS.prev_turn_signal_stalk_state == 1:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'leftBlinker'
        be.pressed = self.CS.turn_signal_stalk_state == 1
        buttonEvents.append(be)
      if self.CS.turn_signal_stalk_state == 2 or self.CS.prev_turn_signal_stalk_state == 2:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'rightBlinker'
        be.pressed = self.CS.turn_signal_stalk_state == 2
        buttonEvents.append(be)

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'unknown'
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        be.type = 'accelCruise'
      elif but == CruiseButtons.DECEL_SET:
        be.type = 'decelCruise'
      elif but == CruiseButtons.CANCEL:
        be.type = 'cancel'
      elif but == CruiseButtons.MAIN:
        be.type = 'altButton3'
      buttonEvents.append(be)

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'unknown'
      be.pressed = bool(self.CS.cruise_buttons)
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents

    # events
    events = []

    #notification messages for DAS
    if (not c.enabled) and (self.CC.opState == 2):
      self.CC.opState = 0
    if c.enabled and (self.CC.opState == 0):
      self.CC.opState = 1
    if can_rcv_error:
      self.can_invalid_count += 1
      if self.can_invalid_count >= 100: #BB increased to 100 to see if we still get the can error messages
        events.append(create_event('invalidGiraffeHonda', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
        self.CS.DAS_canErrors = 1
        if self.CC.opState == 1:
          self.CC.opState = 2
    else:
      self.can_invalid_count = 0
    if self.CS.steer_error:
      if not self.CS.enableHSO:
        events.append(create_event('steerUnavailable', [ET.NO_ENTRY, ET.WARNING]))
    elif self.CS.steer_warning:
      if not self.CS.enableHSO:
         events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
         if self.CC.opState == 1:
          self.CC.opState = 2
    if self.CS.brake_error:
      events.append(create_event('brakeUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
      if self.CC.opState == 1:
          self.CC.opState = 2
    if not ret.gearShifter == 'drive':
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
      if c.enabled:
        self.CC.DAS_222_accCameraBlind = 1
        self.CC.warningCounter = 300
        self.CC.warningNeeded = 1
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
      self.CS.DAS_doorOpen = 1
      if self.CC.opState == 1:
        self.CC.opState = 0
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
      if c.enabled:
        self.CC.DAS_211_accNoSeatBelt = 1
        self.CC.warningCounter = 300
        self.CC.warningNeeded = 1
      if self.CC.opState == 1:
        self.CC.opState = 2
    if self.CS.esp_disabled:
      events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
      if self.CC.opState == 1:
          self.CC.opState = 2
    if not self.CS.main_on:
      events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
      if self.CC.opState == 1:
          self.CC.opState = 0
    if ret.gearShifter == 'reverse':
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
      self.CS.DAS_notInDrive = 1
      if self.CC.opState == 1:
          self.CC.opState = 0
    if ret.gearShifter == 'drive':
        self.CS.DAS_notInDrive =0
    if self.CS.brake_hold:
      events.append(create_event('brakeHold', [ET.NO_ENTRY, ET.USER_DISABLE]))
      if self.CC.opState == 1:
          self.CC.opState = 0
    if self.CS.park_brake:
      events.append(create_event('parkBrake', [ET.NO_ENTRY, ET.USER_DISABLE]))
      if self.CC.opState == 1:
          self.CC.opState = 0
    if (not c.enabled) and (self.CC.opState == 1):
      self.CC.opState = 0

    if self.CP.enableCruise and ret.vEgo < self.CP.minEnableSpeed:
      events.append(create_event('speedTooLow', [ET.NO_ENTRY]))

    # Standard OP method to disengage:
    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    #    if (ret.gasPressed and not self.gas_pressed_prev) or \
    #       (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
    #      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

    #if (self.CS.cstm_btns.get_button_status("brake")>0):
    #  if ((self.CS.brake_pressed !=0) != self.brake_pressed_prev): #break not canceling when pressed
    #  self.CS.cstm_btns.set_button_status("brake", 2 if self.CS.brake_pressed != 0 else 1)
    #else:
    #  if ret.brakePressed:
    #    events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))

    # it can happen that car cruise disables while comma system is enabled: need to
    # keep braking if needed or if the speed is very low
    if self.CP.enableCruise and not ret.cruiseState.enabled and c.actuators.brake <= 0.:
      # non loud alert if cruise disbales below 25mph as expected (+ a little margin)
      if ret.vEgo < self.CP.minEnableSpeed + 2.:
        events.append(create_event('speedTooLow', [ET.IMMEDIATE_DISABLE]))
      else:
        events.append(create_event("cruiseDisabled", [ET.IMMEDIATE_DISABLE]))
    if self.CS.CP.minEnableSpeed > 0 and ret.vEgo < 0.001:
      events.append(create_event('manualRestart', [ET.WARNING]))

    cur_time = self.frame * DT_CTRL
    enable_pressed = False
    # handle button presses
    for b in ret.buttonEvents:

      # do enable on both accel and decel buttons
      if b.type == "altButton3" and not b.pressed:
        print ("enabled pressed at", cur_time)
        self.last_enable_pressed = cur_time
        enable_pressed = True

      # do disable on button down
      if b.type == "cancel" and b.pressed:
        events.append(create_event('buttonCancel', [ET.USER_DISABLE]))

    if self.CP.enableCruise:
      # KEEP THIS EVENT LAST! send enable event if button is pressed and there are
      # NO_ENTRY events, so controlsd will display alerts. Also not send enable events
      # too close in time, so a no_entry will not be followed by another one.
      # TODO: button press should be the only thing that triggers enble
      if ((cur_time - self.last_enable_pressed) < 0.2 and # pylint: disable=chained-comparison
          (cur_time - self.last_enable_sent) > 0.2 and
          ret.cruiseState.enabled) or \
         (enable_pressed and get_events(events, [ET.NO_ENTRY])): 
        if ret.seatbeltUnlatched:
          self.CC.DAS_211_accNoSeatBelt = 1
          self.CC.warningCounter = 300
          self.CC.warningNeeded = 1
        elif not ret.gearShifter == 'drive':
          self.CC.DAS_222_accCameraBlind = 1
          self.CC.warningCounter = 300
          self.CC.warningNeeded = 1
        elif not self.CS.apEnabled:
          self.CC.DAS_206_apUnavailable = 1
          self.CC.warningCounter = 300
          self.CC.warningNeeded = 1
        else:
          events.append(create_event('buttonEnable', [ET.ENABLE]))
        self.last_enable_sent = cur_time
    elif enable_pressed:
      if ret.seatbeltUnlatched:
        self.CC.DAS_211_accNoSeatBelt = 1
        self.CC.warningCounter = 300
        self.CC.warningNeeded = 1
      elif not ret.gearShifter == 'drive':
        self.CC.DAS_222_accCameraBlind = 1
        self.CC.warningCounter = 300
        self.CC.warningNeeded = 1
      elif not self.CS.apEnabled:
        self.CC.DAS_206_apUnavailable = 1
        self.CC.warningCounter = 300
        self.CC.warningNeeded = 1
      else:
        events.append(create_event('buttonEnable', [ET.ENABLE]))
     
        

    ret.events = events
    ret.canMonoTimes = canMonoTimes

    # update previous brake/gas pressed
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = self.CS.brake_pressed != 0


    # cast to reader so it can't be modified
    return ret.as_reader()

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):
    if c.hudControl.speedVisible:
      hud_v_cruise = c.hudControl.setSpeed * CV.MS_TO_KPH
    else:
      hud_v_cruise = 255

    VISUAL_HUD = {
      VisualAlert.none: AH.NONE,
      VisualAlert.fcw: AH.FCW,
      VisualAlert.steerRequired: AH.STEER,
      VisualAlert.brakePressed: AH.BRAKE_PRESSED,
      VisualAlert.wrongGear: AH.GEAR_NOT_D,
      VisualAlert.seatbeltUnbuckled: AH.SEATBELT,
      VisualAlert.speedTooHigh: AH.SPEED_TOO_HIGH}

    AUDIO_HUD = {
      AudibleAlert.none: (BP.MUTE, CM.MUTE),
      AudibleAlert.chimeEngage: (BP.SINGLE, CM.MUTE),
      AudibleAlert.chimeDisengage: (BP.SINGLE, CM.MUTE),
      AudibleAlert.chimeError: (BP.MUTE, CM.DOUBLE),
      AudibleAlert.chimePrompt: (BP.MUTE, CM.SINGLE),
      AudibleAlert.chimeWarning1: (BP.MUTE, CM.DOUBLE),
      AudibleAlert.chimeWarning2: (BP.MUTE, CM.REPEATED),
      AudibleAlert.chimeWarningRepeat: (BP.MUTE, CM.REPEATED)}

    hud_alert = VISUAL_HUD[c.hudControl.visualAlert.raw]
    snd_beep, snd_chime = AUDIO_HUD[c.hudControl.audibleAlert.raw]

    pcm_accel = int(clip(c.cruiseControl.accelOverride,0,1)*0xc6)

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, \
      c.actuators, \
      c.cruiseControl.speedOverride, \
      c.cruiseControl.override, \
      c.cruiseControl.cancel, \
      pcm_accel, \
      hud_v_cruise, c.hudControl.lanesVisible, \
      hud_show_car = c.hudControl.leadVisible, \
      hud_alert = hud_alert, \
      snd_beep = snd_beep, \
      snd_chime = snd_chime, \
      leftLaneVisible = c.hudControl.leftLaneVisible,\
      rightLaneVisible = c.hudControl.rightLaneVisible)

    self.frame += 1
    return can_sends
