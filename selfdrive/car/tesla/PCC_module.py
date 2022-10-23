from common.numpy_fast import clip, interp
from selfdrive.car.tesla.values import CruiseButtons
from selfdrive.car.tesla.tunes import pedal_kpBP, pedal_kpV,pedal_kiBP, pedal_kiV,pedal_kdBP, pedal_kdV, V_PID_FILE,gasMaxBP, gasMaxV, brakeMaxBP, brakeMaxV
from selfdrive.config import Conversions as CV
import time
from cereal import car
from selfdrive.car.modules.CFG_module import load_float_param
from selfdrive.controls.lib.pid_real import  PIDController
import json


_DT = 0.05  # 20Hz in our case, since we don't want to process more than once the same radarState message

ACCEL_MAX = 0.6  #0.6m/s2 * 36 = ~ 0 -> 50mph in 6 seconds
ACCEL_MIN = -0.5 #changed from -3.5 to -4.5 to see if we get better braking with iBooster
MAX_BRAKE_VALUE = 1 #ibooster fully pressed BBTODO determine the exact value we need
BRAKE_LOOKUP_BP = [ACCEL_MIN, interp(0.0,brakeMaxBP,brakeMaxV)]
BRAKE_LOOKUP_V = [MAX_BRAKE_VALUE, 0.]
PID_UNWIND_RATE = 0.6 * _DT
PID_UNWIND_RATE_IBOOSTER = 0.4 * _DT

# TODO: these should end up in values.py at some point, probably variable by trim
# Accel limits
MAX_RADAR_DISTANCE = 120.0  # max distance to take in consideration radar reading
MAX_PEDAL_VALUE = 60
MAX_PEDAL_REGEN_VALUE = -7
PEDAL_HYST_GAP = (
    1.0  # don't change pedal command for small oscilalitons within this value
)
# Cap the pedal to go from 0 to max in 6 seconds
PEDAL_MAX_UP = (MAX_PEDAL_VALUE - MAX_PEDAL_REGEN_VALUE) * _DT / 6
# Cap the pedal to go from max to 0 in 0.4 seconds
PEDAL_MAX_DOWN = (MAX_PEDAL_VALUE - MAX_PEDAL_REGEN_VALUE) * _DT / 0.4

MIN_PCC_V_KPH = 0.0  #
MAX_PCC_V_KPH = 270.0

# Pull the cruise stalk twice in this many ms for a 'double pull'
STALK_DOUBLE_PULL_MS = 750

T_FOLLOW = load_float_param("TinklaFollowDistance",1.45)
PEDAL_PROFILE = int(load_float_param("TinklaPedalProfile",2.0)-1)

class PCCState:
    # Possible state of the PCC system, following the DI_cruiseState naming scheme.
    OFF = 0  # Disabled by UI (effectively never happens since button switches over to ACC mode).
    STANDBY = 1  # Ready to be engaged.
    ENABLED = 2  # Engaged.
    NOT_READY = 9  # Not ready to be engaged due to the state of the car.


def _current_time_millis():
    return int(round(time.time() * 1000))


# this is for the pedal cruise control
class PCCController:

    @staticmethod
    def compute_gb(accel, speed):
        # TODO: is this correct?
        return float(accel) #/ 3.0

    def __init__(self, longcontroller,tesla_can,pedalcan,CP):
        self.LongCtr = longcontroller
        self.CP = CP
        self.carFingerprint = CP.carFingerprint
        self.tesla_can = tesla_can
        self.pcc_available = self.prev_pcc_available = False
        self.pedal_timeout_frame = 0
        self.lead_1 = None
        self.enable_pedal_cruise = False
        self.stalk_pull_time_ms = 0
        self.prev_stalk_pull_time_ms = -1000
        self.prev_cruise_buttons = CruiseButtons.IDLE
        self.pedal_speed_kph = 0.0
        self.speed_limit_kph = 0.0
        self.prev_speed_limit_kph = 0.0
        self.pedal_idx = 0
        self.pedal_steady = 0.0
        self.prev_tesla_pedal = 0.0
        self.prev_tesla_brake = 0.0
        self.v_pid = 0.0
        self.pedalcan = pedalcan
        self.pid = PIDController((pedal_kpBP, pedal_kpV),
          (pedal_kiBP, pedal_kiV),
          (pedal_kdBP,pedal_kdV),
          rate=1/_DT,
          sat_limit=0.8,
          convert=self.compute_gb)
        self.pid.i_unwind_rate = PID_UNWIND_RATE
        self.load_pid()

    def load_pid(self):
        try:
            v_pid_json = open(V_PID_FILE)
            data = json.load(v_pid_json)
            if self.pid:
                self.pid.p = data["p"]
                self.pid.i = data["i"]
                if "d" not in data:
                    self.pid.d = 0.01
                else:
                    self.pid.d = data["d"]
                self.pid.f = data["f"]
            else:
                print("self.pid not initialized!")
        except IOError:
            print("file not present, creating at next reset")

    def save_pid(self):
        data = {}
        data["p"] = self.pid.p
        data["i"] = self.pid.i
        data["d"] = self.pid.d
        data["f"] = self.pid.f
        try:
            with open(V_PID_FILE, "w") as outfile:
                json.dump(data, outfile)
        except IOError:
           print("PDD pid parameters could not be saved to file")

    def reset(self, v_pid):
        """Reset PID controller and change setpoint"""
        self.v_pid = v_pid

    def update_stat(self, CS, frame):
        
        if not self.LongCtr.CP.openpilotLongitudinalControl:
            self.pcc_available = False
            return []

        if not CS.enablePedal:
            self.pcc_available = False
            return []

        self._update_pedal_state(CS, frame)

        can_sends = []
        if not self.pcc_available:
            timed_out = frame >= self.pedal_timeout_frame
            if timed_out or CS.pedal_interceptor_state > 0:
                if frame % 50 == 0:
                    # send reset command
                    idx = self.pedal_idx
                    self.pedal_idx = (self.pedal_idx + 1) % 16
                    can_sends.append(
                        self.tesla_can.create_pedal_command_msg(0, 0, idx, self.pedalcan)
                    )
            return can_sends

        prev_enable_pedal_cruise = self.enable_pedal_cruise
        # disable on brake
        if CS.realBrakePressed and self.enable_pedal_cruise:
            CS.longCtrlEvent = car.CarEvent.EventName.pccDisabled
            self.enable_pedal_cruise = False

        # process any stalk movement
        curr_time_ms = _current_time_millis()
        speed_uom_kph = 1.0
        if CS.speed_units == "MPH":
            speed_uom_kph = CV.MPH_TO_KPH
        if (
            CS.cruise_buttons == CruiseButtons.MAIN
            and self.prev_cruise_buttons != CruiseButtons.MAIN
        ):
            self.prev_stalk_pull_time_ms = self.stalk_pull_time_ms
            self.stalk_pull_time_ms = curr_time_ms
            double_pull = (
                self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms
                < STALK_DOUBLE_PULL_MS
            )
            ready = CS.enablePedal
            if ready and double_pull:
                # A double pull enables ACC. updating the max ACC speed if necessary.
                if not self.enable_pedal_cruise:
                    CS.longCtrlEvent = car.CarEvent.EventName.pccEnabled
                self.enable_pedal_cruise = True
                # Increase PCC speed to match current, if applicable.
                # We round the target speed in the user's units of measurement to avoid jumpy speed readings
                current_speed_kph_uom_rounded = (
                    int(CS.out.vEgo * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
                )
                self.pedal_speed_kph = max(
                    current_speed_kph_uom_rounded, self.speed_limit_kph
                )
        # Handle pressing the cancel button.
        elif CS.cruise_buttons == CruiseButtons.CANCEL:
            if self.enable_pedal_cruise:
                CS.longCtrlEvent = car.CarEvent.EventName.pccDisabled
            self.enable_pedal_cruise = False
            self.pedal_speed_kph = 0.0
            self.stalk_pull_time_ms = 0
            self.prev_stalk_pull_time_ms = -1000
        # Handle pressing up and down buttons.
        elif self.enable_pedal_cruise and CS.cruise_buttons != self.prev_cruise_buttons:
            # Real stalk command while PCC is already enabled. Adjust the max PCC speed if necessary.
            # We round the target speed in the user's units of measurement to avoid jumpy speed readings
            actual_speed_kph_uom_rounded = (
                int(CS.out.vEgo * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
            )
            if CS.cruise_buttons == CruiseButtons.RES_ACCEL:
                self.pedal_speed_kph = (
                    max(self.pedal_speed_kph, actual_speed_kph_uom_rounded)
                    + speed_uom_kph
                )
            elif CS.cruise_buttons == CruiseButtons.RES_ACCEL_2ND:
                self.pedal_speed_kph = (
                    max(self.pedal_speed_kph, actual_speed_kph_uom_rounded)
                    + 5 * speed_uom_kph
                )
            elif CS.cruise_buttons == CruiseButtons.DECEL_SET:
                self.pedal_speed_kph = self.pedal_speed_kph - speed_uom_kph
            elif CS.cruise_buttons == CruiseButtons.DECEL_2ND:
                self.pedal_speed_kph = self.pedal_speed_kph - 5 * speed_uom_kph
            # Clip PCC speed between 0 and 170 KPH.
            self.pedal_speed_kph = clip(
                self.pedal_speed_kph, MIN_PCC_V_KPH, MAX_PCC_V_KPH
            )
        # If something disabled cruise control, disable PCC too
        elif self.enable_pedal_cruise and CS.cruise_state and not CS.enablePedal:
            self.enable_pedal_cruise = False
            CS.longCtrlEvent = car.CarEvent.EventName.pccDisabled
        # A single pull disables PCC (falling back to just steering). Wait some time
        # in case a double pull comes along.
        elif (
            self.enable_pedal_cruise
            and curr_time_ms - self.stalk_pull_time_ms > STALK_DOUBLE_PULL_MS
            and self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms
            > STALK_DOUBLE_PULL_MS
        ):
            self.enable_pedal_cruise = False
            CS.longCtrlEvent = car.CarEvent.EventName.pccDisabled

        # Update prev state after all other actions.
        self.prev_cruise_buttons = CS.cruise_buttons
        if prev_enable_pedal_cruise and not self.enable_pedal_cruise:
            #we just chanceled, save PID
            self.save_pid()

        return can_sends

    def update_pdl(
        self,
        enabled,
        CS,
        frame,
        actuators,
        v_target,
        a_target,
        pcm_override,
        speed_limit_ms,
        set_speed_limit_active,
        speed_limit_offset,
        alca_enabled,
        radar_state
    ):

        if not self.LongCtr.CP.openpilotLongitudinalControl:
            return 0.0, 0.0, -1, -1

        if not CS.enablePedal:
            return 0.0, 0.0, -1, -1

        idx = self.pedal_idx

        if CS.has_ibooster_ecu:
            self.pid.i_unwind_rate = PID_UNWIND_RATE_IBOOSTER

        self.prev_speed_limit_kph = self.speed_limit_kph

        if radar_state is not None:
                self.lead_1 = radar_state.radarState.leadOne

        if set_speed_limit_active and speed_limit_ms > 0:
            self.speed_limit_kph = (speed_limit_ms + speed_limit_offset) * CV.MS_TO_KPH
            if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
                self.pedal_speed_kph = self.speed_limit_kph
        else:  # reset internal speed limit, so double pull doesn't set higher speed than current (e.g. after leaving the highway)
            self.speed_limit_kph = 0.0
        self.pedal_idx = (self.pedal_idx + 1) % 16

        if not self.pcc_available or not enabled:
            self.reset(CS.out.vEgo)
            return 0.0, 0.0, 0, idx

        self.v_pid = v_target

        #TODO: ibstBrakeApplied use that to determine if iBooster is still being pressed
        # to prevent both pedals being on

        prevent_overshoot = not self.CP.stoppingControl and CS.out.vEgo < 1.5 and v_target < 0.7 and v_target < self.v_pid
        deadzone = interp(CS.out.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
        freeze_integrator = prevent_overshoot
        
        gas_max = interp(CS.out.vEgo, gasMaxBP, gasMaxV[PEDAL_PROFILE])
        brake_max = interp(CS.out.vEgo, brakeMaxBP, brakeMaxV)
        if CS.has_ibooster_ecu:
            brake_max = ACCEL_MIN
        self.pid.neg_limit = brake_max
        self.pid.pos_limit = gas_max
        
        if self.enable_pedal_cruise:
            tesla_pedal = self.pid.update(self.v_pid, CS.out.vEgo, speed=CS.out.vEgo, 
                        deadzone=deadzone, feedforward=a_target, 
                        freeze_integrator=freeze_integrator)
            enable_pedal = 1.0
        else:
            tesla_pedal = 0.0
            enable_pedal = 0.0
            self.reset(CS.out.vEgo)
        tesla_brake = clip(tesla_pedal,ACCEL_MIN,interp(CS.out.vEgo, brakeMaxBP, brakeMaxV))
        tesla_pedal = clip(tesla_pedal, interp(CS.out.vEgo, brakeMaxBP, brakeMaxV), gas_max)
        tesla_pedal = int((tesla_pedal -0.07)* 100)

        if abs(CS.out.vEgo * CV.MS_TO_KPH - self.pedal_speed_kph) < 0.5:
            tesla_pedal = self.pedal_hysteresis(tesla_pedal, enable_pedal)
        if CS.out.vEgo < 0.1 and actuators.accel < 0.01:
            #hold brake pressed at when standstill
            #BBTODO: show HOLD indicator in IC with integration
            # for about 14psi to hold a car even on slopes 
            # we need roughty 6.5 mm / 15 = 
            tesla_brake = 0.43
        else:
            tesla_brake = interp(tesla_brake, BRAKE_LOOKUP_BP, BRAKE_LOOKUP_V)

        tesla_pedal = clip(tesla_pedal, self.prev_tesla_pedal - PEDAL_MAX_DOWN, self.prev_tesla_pedal + PEDAL_MAX_UP)

        # if gas pedal pressed, brake should be zero (we alwasys have pedal with ibooster)
        # if CS.has_ibooster_ecu:
        #     if CS.brakeUnavailable:
        #         CS.longCtrlEvent = car.CarEvent.EventName.iBoosterBrakeNotOk
        #     if self.prev_tesla_pedal > 0:
        #         tesla_brake = 0
        #     if self.prev_tesla_brake > 0:
        #         tesla_pedal = MAX_PEDAL_REGEN_VALUE
         
        
        self.prev_tesla_brake = tesla_brake * enable_pedal
        self.prev_tesla_pedal = tesla_pedal * enable_pedal
        #print("pedal=",self.prev_tesla_pedal, "   brake=", self.prev_tesla_brake)
        return self.prev_tesla_pedal, self.prev_tesla_brake, enable_pedal, idx

    def pedal_hysteresis(self, pedal, enabled):
        # for small accel oscillations within PEDAL_HYST_GAP, don't change the command
        if not enabled:
            # send 0 when disabled, otherwise acc faults
            self.pedal_steady = 0.0
        elif pedal > self.pedal_steady + PEDAL_HYST_GAP:
            self.pedal_steady = pedal - PEDAL_HYST_GAP
        elif pedal < self.pedal_steady - PEDAL_HYST_GAP:
            self.pedal_steady = pedal + PEDAL_HYST_GAP
        return self.pedal_steady

    def _update_pedal_state(self, CS, frame):
        if CS.pedal_idx != CS.prev_pedal_idx:
            # time out pedal after 500ms without receiving a new CAN message from it
            self.pedal_timeout_frame = frame + 50
        self.prev_pcc_available = self.pcc_available
        pedal_ready = (
            frame < self.pedal_timeout_frame and CS.pedal_interceptor_state == 0
        )
        #acc_disabled = CS.enablePedal or CruiseState.is_off(CS.cruise_state)
        # Mark pedal unavailable while traditional cruise is on.
        self.pcc_available = pedal_ready and CS.enablePedal
