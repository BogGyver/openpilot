from selfdrive.car.tesla.speed_utils.fleet_speed import FleetSpeed
from common.numpy_fast import clip, interp
from selfdrive.car.tesla.values import CruiseState, CruiseButtons
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.longitudinal_planner import limit_accel_in_turns
import time
import math
from collections import OrderedDict
from common.params import Params
import numpy as np
from cereal import car

ACCEL_MAX = 2.0
ACCEL_MIN = -3.5

# lookup tables VS speed to determine min and max accels in cruise
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MIN_V = [-1.0, -.8, -.67, -.5, -.30]
_A_CRUISE_MIN_BP = [  0.,  5.,  10., 20.,  40.]

# need fast accel at very low speed for stop and go
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MAX_V = [1.2, 1.2, 0.65, .4]
_A_CRUISE_MAX_V_FOLLOWING = [1.6, 1.6, 0.65, .4]
_A_CRUISE_MAX_BP = [0.,  6.4, 22.5, 40.]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def calc_cruise_accel_limits(v_ego, following):
  a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V)

  if following:
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_FOLLOWING)
  else:
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V)
  return np.vstack([a_cruise_min, a_cruise_max])

_DT = 0.05  # 10Hz in our case, since we don't want to process more than once the same radarState message
_DT_MPC = _DT

# Reset the PID completely on disengage of PCC
RESET_PID_ON_DISENGAGE = False

# TODO: these should end up in values.py at some point, probably variable by trim
# Accel limits
MAX_RADAR_DISTANCE = 120.0  # max distance to take in consideration radar reading
MAX_PEDAL_VALUE = 100.0
PEDAL_HYST_GAP = (
    1.0  # don't change pedal command for small oscilalitons within this value
)
# Cap the pedal to go from 0 to max in 2 seconds
PEDAL_MAX_UP = MAX_PEDAL_VALUE * _DT / 2
# Cap the pedal to go from max to 0 in 0.4 seconds
PEDAL_MAX_DOWN = MAX_PEDAL_VALUE * _DT / 0.4

# min safe distance in meters. Roughly 2 car lengths.
MIN_SAFE_DIST_M = 6.0

# BBTODO: move the vehicle variables; maybe make them speed variable
TORQUE_LEVEL_ACC = 0.0
TORQUE_LEVEL_DECEL = -30.0

MIN_PCC_V_KPH = 0.0  #
MAX_PCC_V_KPH = 270.0

ANGLE_STOP_ACCEL = 10.0  # this should be speed dependent

MIN_CAN_SPEED = 0.3  # TODO: parametrize this in car interface

# Pull the cruise stalk twice in this many ms for a 'double pull'
STALK_DOUBLE_PULL_MS = 750

def tesla_compute_gb(accel, speed):
    return float(accel) / 3.0

def max_v_in_mapped_curve_ms(map_data, pedal_set_speed_kph):
    """Use HD map data to limit speed in sharper turns."""
    if map_data and map_data.curvatureValid:
        pedal_set_speed_ms = pedal_set_speed_kph * CV.KPH_TO_MS
        # Max lateral acceleration, used to caclulate how much to slow down in turns
        a_y_max = 1.85  # m/s^2
        curvature = abs(map_data.curvature)
        v_curvature_ms = math.sqrt(a_y_max / max(1e-4, curvature))
        time_to_turn_s = max(0, map_data.distToTurn / max(pedal_set_speed_ms, 1.0))
        v_approaching_turn_ms = OrderedDict(
            [
                # seconds til turn, max allowed velocity
                (0, pedal_set_speed_ms),
                (8, v_curvature_ms),
            ]
        )
        return _interp_map(time_to_turn_s, v_approaching_turn_ms)
    else:
        return None


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
    def __init__(self, longcontroller,tesla_can,pedalcan):
        self.LongCtr = longcontroller
        self.tesla_can = tesla_can
        self.human_cruise_action_time = 0
        self.pcc_available = self.prev_pcc_available = False
        self.pedal_timeout_frame = 0
        self.accelerator_pedal_pressed = self.prev_accelerator_pedal_pressed = False
        self.automated_cruise_action_time = 0
        self.last_angle = 0.0
        self.lead_1 = None
        self.last_update_time = 0
        self.enable_pedal_cruise = False
        self.stalk_pull_time_ms = 0
        self.prev_stalk_pull_time_ms = -1000
        self.prev_cruise_state = 0
        self.prev_cruise_buttons = CruiseButtons.IDLE
        self.pedal_speed_kph = 0.0
        self.speed_limit_kph = 0.0
        self.prev_speed_limit_kph = 0.0
        self.pedal_idx = 0
        self.pedal_steady = 0.0
        self.prev_tesla_accel = 0.0
        self.prev_tesla_pedal = 0.0
        self.torqueLevel_last = 0.0
        self.prev_v_ego = 0.0
        self.PedalForZeroTorque = (
            18.0  # starting number for a S85, adjusts down automatically
        )
        self.lastTorqueForPedalForZeroTorque = TORQUE_LEVEL_DECEL
        self.v_pid = 0.0
        self.a_pid = 0.0
        self.last_output_gb = 0.0
        self.last_speed_kph = None
        # for smoothing the changes in speed
        self.v_acc_start = 0.0
        self.a_acc_start = 0.0
        self.v_acc = 0.0
        self.v_acc_sol = 0.0
        self.v_acc_future = 0.0
        self.a_acc = 0.0
        self.a_acc_sol = 0.0
        self.v_cruise = 0.0
        self.a_cruise = 0.0
 
        # when was radar data last updated?
        self.lead_last_seen_time_ms = 0
        self.continuous_lead_sightings = 0
        self.params = Params()
        average_speed_over_x_suggestions = 6  # 0.3 seconds (20x a second)
        self.fleet_speed = FleetSpeed(average_speed_over_x_suggestions)
        self.pedalcan = pedalcan

    def update_stat(self, CS, frame):
        if not self.LongCtr.CP.openpilotLongitudinalControl:
            return []

        if not self.LongCtr.enablePedal:
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
            ready = (
                (CruiseState.is_off(CS.cruise_state))
                or CS.enablePedal
            )
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

        # Notify if PCC was toggled
        if prev_enable_pedal_cruise and not self.enable_pedal_cruise:
            self.fleet_speed.reset_averager()

        # Update prev state after all other actions.
        self.prev_cruise_buttons = CS.cruise_buttons
        self.prev_cruise_state = CS.cruise_state

        return can_sends

    def update_pdl(
        self,
        enabled,
        CS,
        frame,
        actuators,
        pcm_speed,
        pcm_override,
        speed_limit_ms,
        set_speed_limit_active,
        speed_limit_offset,
        alca_enabled,
        radSt
    ):

        if not self.LongCtr.CP.openpilotLongitudinalControl:
            return 0.0, -1, -1

        if not self.LongCtr.enablePedal:
            return 0.0, -1, -1

        idx = self.pedal_idx

        self.prev_speed_limit_kph = self.speed_limit_kph

        ######################################################################################
        # Determine pedal "zero"
        #
        # save position for cruising (zero acc, zero brake, no torque) when we are above 10 MPH
        ######################################################################################
        if (
            CS.torqueLevel < TORQUE_LEVEL_ACC
            and CS.torqueLevel > TORQUE_LEVEL_DECEL
            and CS.out.vEgo >= 10.0 * CV.MPH_TO_MS
            and abs(CS.torqueLevel) < abs(self.lastTorqueForPedalForZeroTorque)
            and self.prev_tesla_accel > 0.0
        ):
            self.PedalForZeroTorque = self.prev_tesla_accel
            self.lastTorqueForPedalForZeroTorque = CS.torqueLevel
            # print ("Detected new Pedal For Zero Torque at %s" % (self.PedalForZeroTorque))
            # print ("Torque level at detection %s" % (CS.torqueLevel))
            # print ("Speed level at detection %s" % (CS.out.vEgo * CV.MS_TO_MPH))

        if set_speed_limit_active and speed_limit_ms > 0:
            self.speed_limit_kph = (speed_limit_ms + speed_limit_offset) * CV.MS_TO_KPH
            if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
                self.pedal_speed_kph = self.speed_limit_kph
                # reset MovingAverage for fleet speed when speed limit changes
                self.fleet_speed.reset_averager()
        else:  # reset internal speed limit, so double pull doesn't set higher speed than current (e.g. after leaving the highway)
            self.speed_limit_kph = 0.0
        self.pedal_idx = (self.pedal_idx + 1) % 16

        if not self.pcc_available or not enabled:
            return 0.0, 0, idx

        ##############################################################
        # This mode uses the longitudinal MPC built in OP
        #
        # we use the values from actuators.gas and actuators.brake
        ##############################################################
        output_gb = actuators.accel
        self.v_pid = pcm_speed
        MPC_BRAKE_MULTIPLIER = 3.0

        self.last_output_gb = output_gb
        # accel and brake
        apply_accel = clip(
            output_gb, 
            0.0,
            1.0
        )
        apply_brake = -clip(
            output_gb * MPC_BRAKE_MULTIPLIER ,
            _brake_pedal_min(
                CS.out.vEgo, self.v_pid, self.lead_1, CS, self.pedal_speed_kph
            ),
            0.0,
        )
        # if speed is over 5mph, the "zero" is at PedalForZeroTorque; otherwise it is zero
        pedal_zero = 0.0
        if CS.out.vEgo >= 5.0 * CV.MPH_TO_MS:
            pedal_zero = self.PedalForZeroTorque
        tesla_brake = clip((1.0 - apply_brake) * pedal_zero, 0, pedal_zero)
        tesla_accel = clip(
            apply_accel * (MAX_PEDAL_VALUE - pedal_zero),
            0,
            MAX_PEDAL_VALUE - pedal_zero,
        )
        tesla_pedal = tesla_brake + tesla_accel
        tesla_pedal = self.pedal_hysteresis(tesla_pedal, enabled)
        tesla_pedal = clip(
            tesla_pedal,
            self.prev_tesla_pedal - PEDAL_MAX_DOWN,
            self.prev_tesla_pedal + PEDAL_MAX_UP,
        )
        tesla_pedal = (
            clip(tesla_pedal, 0.0, MAX_PEDAL_VALUE) if self.enable_pedal_cruise else 0.0
        )
        enable_pedal = 1.0 if self.enable_pedal_cruise else 0.0

        self.torqueLevel_last = CS.torqueLevel
        self.prev_tesla_pedal = tesla_pedal * enable_pedal
        self.prev_tesla_accel = apply_accel * enable_pedal
        self.prev_v_ego = CS.out.vEgo
        return self.prev_tesla_pedal, enable_pedal, idx

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
        acc_disabled = CS.enablePedal or CruiseState.is_off(CS.cruise_state)
        # Mark pedal unavailable while traditional cruise is on.
        self.pcc_available = pedal_ready and acc_disabled

def _safe_distance_m(v_ego_ms, CS):
    return max(CS.apFollowTimeInS * (v_ego_ms + 1), MIN_SAFE_DIST_M)

def _is_present(lead):
    return bool((not (lead is None)) and (lead.dRel > 0))

def _interp_map(val, val_map):
    """Helper to call interp with an OrderedDict for the mapping. I find
    this easier to read than interp, which takes two arrays."""
    return interp(val, list(val_map.keys()), list(val_map.values()))

def _brake_pedal_min(v_ego, v_target, lead, CS, max_speed_kph):
    # if less than 7 MPH we don't have much left till 5MPH to brake, so full regen
    if v_ego <= 7 * CV.MPH_TO_MS:
        return -1
    # if above speed limit quickly decel
    if v_ego * CV.MS_TO_KPH > max_speed_kph:
        return -0.8
    speed_delta_perc = 100 * (v_ego - v_target) / v_ego
    brake_perc_map = OrderedDict(
        [
            # (perc change, decel)
            (0.0, 0.3),
            (1.5, 0.5),
            (5.0, 0.8),
            (7.0, 1.0),
            (50.0, 1.0),
        ]
    )
    brake_mult1 = _interp_map(speed_delta_perc, brake_perc_map)
    brake_mult2 = 0.0
    if _is_present(lead):
        safe_dist_m = _safe_distance_m(CS.out.vEgo, CS)
        brake_distance_map = OrderedDict(
            [
                # (distance in m, decceleration fraction)
                (0.8 * safe_dist_m, 1.0),
                (1.0 * safe_dist_m, 0.6),
                (3.0 * safe_dist_m, 0.4),
            ]
        )
        brake_mult2 = _interp_map(lead.dRel, brake_distance_map)
    brake_mult = max(brake_mult1, brake_mult2)
    return -brake_mult