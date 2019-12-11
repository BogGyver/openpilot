from selfdrive.config import Conversions as CV
from cereal import tesla,log
import selfdrive.messaging as messaging
import time

DEBUG = False

class AHBReason():
  HIGH_BEAM_ON = 0
  HIGH_BEAM_OFF_REASON_MOVING_VISION_TARGET = 1
  HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET = 2
  HIGH_BEAM_OFF_REASON_AMBIENT_LIGHT = 3
  HIGH_BEAM_OFF_REASON_HEAD_LIGHT = 4
  HIGH_BEAM_OFF_REASON_SNA = 5

class AHBDecision():
  DAS_HIGH_BEAM_UNDECIDED = 0
  DAS_HIGH_BEAM_OFF = 1
  DAS_HIGH_BEAM_ON = 2
  DAS_HIGH_BEAM_SNA = 3

def _current_time_millis():
  return int(round(time.time() * 1000))

def _debug(message):
  if DEBUG:
    print(message)

class AHBController():

  def __init__(self,carcontroller):
    self.time_last_car_detected = 0
    self.time_last_high_beam_cancel = 0
    self.prev_car_present = False
    self.prev_highLowBeamStatus = 0
    self.prev_highLowBeamReason = 0
    self.prev_light_stalk_position = 0
    self.prev_high_beam_on = False
    self.prev_lights_on = False
    self.ahbInfo = messaging.sub_sock('ahbInfo', conflate=True)
    self.ahbInfoData = None
    self.ahbIsEnabled = False
    self.frameInfo = messaging.sub_sock('frame', conflate=True)
    self.frameInfoData = None
    self.frameInfoGain = 0

  def set_and_return(self,CS,frame,highLowBeamStatus,highLowBeamReason):
    self.prev_light_stalk_position = CS.ahbHighBeamStalkPosition
    if self.ahbInfoData is not None:
      self.prev_car_present = self.ahbInfoData.cameraCarDetected or self.ahbInfoData.radarCarDetected
    self.prev_highLowBeamStatus = highLowBeamStatus
    self.prev_highLowBeamReason = highLowBeamReason
    self.prev_high_beam_on = CS.ahbHiBeamOn
    self.prev_lights_on = CS.ahbLoBeamOn
    self.ahbIsEnabled = self.ahbIsEnabled and CS.ahbEnabled
    return highLowBeamStatus,highLowBeamReason,self.ahbIsEnabled

  def update(self, CS, frame,ahbLead1):
    tms_now = _current_time_millis()
    ahbInfoMsg = self.ahbInfo.receive(non_blocking=True)
    frameInfoMsg = messaging.recv_one_or_none(self.frameInfo)
    if ahbInfoMsg is not None:
      self.ahbInfoData = tesla.AHBinfo.from_bytes(ahbInfoMsg)
    if frameInfoMsg is not None:
      self.frameInfoData = frameInfoMsg.frame
      frameInfoGain = self.frameInfoData.globalGain
      exposureTime = self.frameInfoData.androidCaptureResult.exposureTime
      frameDuration = self.frameInfoData.androidCaptureResult.frameDuration
      if frameInfoGain != self.frameInfoGain:
        self.frameInfoGain = frameInfoGain
      _debug("AHB Camera has new data [ frame - " + str(self.frameInfoData.frameId) + "]  = " + str(frameInfoGain) + ", exposure = " + str(exposureTime) + ", frame duration = " + str(frameDuration))
    #if AHB not enabled, then return OFF
    if CS.ahbEnabled != 1:
      _debug("AHB not enabled in CID")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_SNA
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if stalk is not in the high beam position, return UNDECIDED
    if (CS.ahbHighBeamStalkPosition != 1):
      _debug("High Beam not on")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_SNA
      self.ahbIsEnabled = False
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason) 
    else :
      self.ahbIsEnabled = True
    #if moving below 10mph take no decision, then return undecided
    if CS.v_ego < 4.47:
      _debug("moving too slow for decision")
      highLowBeamStatus = self.prev_highLowBeamStatus
      highLowBeamReason = self.prev_highLowBeamReason
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    if self.ahbInfoData is None:
      _debug("No radar info")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_SNA
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    # if gain less than max, we might see incoming car
    if self.frameInfoGain < 510:
      _debug("OP camera gain < 510")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if lead car detected by radarD, i.e. OP has Lead, then reset timer and return OFF
    if False and ahbLead1 is not None:
      _debug("OP detected car")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if lead car detected by radar, then reset timer and return OFF
    if self.ahbInfoData.radarCarDetected:
      _debug("Radar detected car")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if lead car detected by vision, then reset timer and return OFF
    if self.ahbInfoData.cameraCarDetected:
      _debug("Vision detected car")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_VISION_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if still waiting for the delay after car detected, send off
    if (tms_now - self.time_last_car_detected < 1000 * CS.ahbOffDuration ):
      _debug("Waiting for time delay since last car")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason) 
    #we got here, high beam should be on
    highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
    highLowBeamReason = AHBReason.HIGH_BEAM_ON
    _debug("All conditions met, turn lights ON")
    return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
