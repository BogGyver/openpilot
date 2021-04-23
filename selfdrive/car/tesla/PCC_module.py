from selfdrive.car.tesla import teslacan
from selfdrive.car.tesla.speed_utils.fleet_speed import FleetSpeed
from selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from common.numpy_fast import clip, interp
from selfdrive.car.tesla.values import CruiseState, CruiseButtons
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.speed_smoother import speed_smoother
from selfdrive.controls.lib.longitudinal_planner import (
    calc_cruise_accel_limits,
    limit_accel_in_turns,
)
import cereal.messaging as messaging
import time
import math
from collections import OrderedDict
from common.params import Params
import json


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

V_PID_FILE = "/data/params/pidParams"

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
    def __init__(self, carcontroller):
        self.CC = carcontroller
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


        data = {}
        data["p"] = pid.p
        data["i"] = pid.i
        data["d"] = pid.d
        data["f"] = pid.f
        try:
            with open(V_PID_FILE, "w") as outfile:
                json.dump(data, outfile)
        except IOError:
            print("PDD pid parameters could not be saved to file")

    def update_stat(self, CS, frame):

        self._update_pedal_state(CS, frame)

        can_sends = []
        if not self.pcc_available:
            timed_out = frame >= self.pedal_timeout_frame
            if timed_out or CS.pedal_interceptor_state > 0:
                if frame % 50 == 0:
                    # send reset command
                    idx = self.pedal_idx
                    self.pedal_idx = (self.pedal_idx + 1) % 16
                    pedalcan = 2
                    can_sends.append(
                        teslacan.create_pedal_command_msg(0, 0, idx, pedalcan)
                    )
            return can_sends

        prev_enable_pedal_cruise = self.enable_pedal_cruise
        # disable on brake
        if CS.out.brakePressed and self.enable_pedal_cruise:
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
                or CS.forcePedalOverCC
            )
            if ready and double_pull:
                # A double pull enables ACC. updating the max ACC speed if necessary.
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
        elif self.enable_pedal_cruise and CS.cruise_state and not CS.forcePedalOverCC:
            self.enable_pedal_cruise = False
        # A single pull disables PCC (falling back to just steering). Wait some time
        # in case a double pull comes along.
        elif (
            self.enable_pedal_cruise
            and curr_time_ms - self.stalk_pull_time_ms > STALK_DOUBLE_PULL_MS
            and self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms
            > STALK_DOUBLE_PULL_MS
        ):
            self.enable_pedal_cruise = False

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
        # Alternative speed decision logic that uses the lead car's distance
        # and speed more directly.
        # Bring in the lead car distance from the radarState feed
        if radSt is not None:
            self.lead_1 = radSt.radarState.leadOne
            if _is_present(self.lead_1):
                self.lead_last_seen_time_ms = _current_time_millis()
                self.continuous_lead_sightings += 1
            else:
                self.continuous_lead_sightings = 0

        v_ego = CS.out.vEgo

        following = False
        if self.lead_1:
            following = (
                self.lead_1.status
                and self.lead_1.dRel < MAX_RADAR_DISTANCE
                and self.lead_1.vLeadK > v_ego
                and self.lead_1.aLeadK > 0.0
            )
        accel_limits = [
            float(x) for x in calc_cruise_accel_limits(v_ego, following, is_tesla=True)
        ]

        accel_limits[1] *= _accel_limit_multiplier(CS, self.lead_1)
        accel_limits[0] = _decel_limit(
            accel_limits[0], CS.out.vEgo, self.lead_1, CS, self.pedal_speed_kph
        )
        jerk_limits = [
            min(-0.1, accel_limits[0] / 2.0),
            max(0.1, accel_limits[1] / 2.0),
        ]  # TODO: make a separate lookup for jerk tuning
        accel_limits = limit_accel_in_turns(v_ego, CS.angle_steers, accel_limits, CS.CP)

        output_gb = 0

        ##############################################################
        # This mode uses the longitudinal MPC built in OP
        #
        # we use the values from actuators.gas and actuators.brake
        ##############################################################
        output_gb = actuators.gas - actuators.brake
        self.v_pid = pcm_speed
        MPC_BRAKE_MULTIPLIER = 12.0

        self.last_output_gb = output_gb
        # accel and brake
        apply_accel = clip(
            output_gb, 0.0, 1
        )  # _accel_pedal_max(CS.out.vEgo, self.v_pid, self.lead_1, self.prev_tesla_accel, CS))
        apply_brake = -clip(
            output_gb * MPC_BRAKE_MULTIPLIER,
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
        acc_disabled = CS.forcePedalOverCC or CruiseState.is_off(CS.cruise_state)
        # Mark pedal unavailable while traditional cruise is on.
        self.pcc_available = pedal_ready and acc_disabled


def _visual_radar_adjusted_dist_m(m, CS):
    # visual radar sucks at short distances. It rarely shows readings below 7m.
    # So rescale distances with 7m -> 0m. Maxes out at 1km, if that matters.
    mapping = OrderedDict(
        [
            # (input distance, output distance)
            (7, 0),  # anything below 7m is set to 0m.
            (1000, 1000),
        ]
    )  # values >7m are scaled, maxing out at 1km.
    return _interp_map(m, mapping)


def _safe_distance_m(v_ego_ms, CS):
    return max(CS.apFollowTimeInS * (v_ego_ms + 1), MIN_SAFE_DIST_M)


def _is_present(lead):
    return bool((not (lead is None)) and (lead.dRel > 0))


def _sec_til_collision(lead, CS):
    if _is_present(lead) and lead.vRel < 0:
        if CS.useTeslaRadar:
            # BB: take in consideration acceleration when looking at time to collision.
            return min(
                0.1,
                -4
                + lead.dRel / abs(lead.vRel + min(0, lead.aRel) * CS.apFollowTimeInS),
            )
        else:
            return _visual_radar_adjusted_dist_m(lead.dRel, CS) / abs(
                lead.vRel + min(0, lead.aRel) * CS.apFollowTimeInS
            )
    else:
        return 60.0  # Arbitrary, but better than MAXINT because we can still do math on it.


def _interp_map(val, val_map):
    """Helper to call interp with an OrderedDict for the mapping. I find
    this easier to read than interp, which takes two arrays."""
    return interp(val, list(val_map.keys()), list(val_map.values()))


def _accel_limit_multiplier(CS, lead):
    """Limits acceleration in the presence of a lead car. The further the lead car
    is, the more accel is allowed. Range: 0 to 1, so that it can be multiplied
    with other accel limits."""
    accel_by_speed = OrderedDict(
        [
            # (speed m/s, decel)
            (0.0, 0.95),  #   0 kmh
            (10.0, 0.95),  #  35 kmh
            (20.0, 0.925),  #  72 kmh
            (30.0, 0.875),
        ]
    )  # 107 kmh
    if CS.teslaModel in ["SP", "SPD"]:
        accel_by_speed = OrderedDict(
            [
                # (speed m/s, decel)
                (0.0, 0.985),  #   0 kmh
                (10.0, 0.975),  #  35 kmh
                (20.0, 0.95),  #  72 kmh
                (30.0, 0.9),
            ]
        )  # 107 kmh
    accel_mult = _interp_map(CS.out.vEgo, accel_by_speed)
    if _is_present(lead):
        safe_dist_m = _safe_distance_m(CS.out.vEgo, CS)
        accel_multipliers = OrderedDict(
            [
                # (distance in m, acceleration fraction)
                (0.6 * safe_dist_m, 0.15),
                (1.0 * safe_dist_m, 0.2),
                (3.0 * safe_dist_m, 0.4),
            ]
        )
        vrel_multipliers = OrderedDict(
            [
                # vrel m/s, accel mult
                (0.0, 1.0),
                (10.0, 1.5),
            ]
        )

        return min(
            accel_mult
            * _interp_map(lead.vRel, vrel_multipliers)
            * _interp_map(lead.dRel, accel_multipliers),
            1.0,
        )
    else:
        return min(accel_mult * 0.4, 1.0)


def _decel_limit(accel_min, v_ego, lead, CS, max_speed_kph):
    max_speed_mult = 1.0
    safe_dist_m = _safe_distance_m(v_ego, CS)
    # if above speed limit quickly decel
    if v_ego * CV.MS_TO_KPH > max_speed_kph:
        overshot = v_ego * CV.MS_TO_KPH - max_speed_kph
        if overshot >= 5:
            max_speed_mult = 2.0
        elif overshot >= 2.0:
            max_speed_mult = 1.5
    if _is_present(lead):
        time_to_brake = max(0.1, _sec_til_collision(lead, CS))
        if 0 < lead.dRel < MIN_SAFE_DIST_M:
            return -100.0
        elif (
            lead.vRel >= 0.1 * v_ego
            and lead.aRel < 0.5
            and lead.dRel <= 1.1 * safe_dist_m
        ):
            # going faster but decelerating, reduce with up to the same acceleration
            return -2 + lead.aRel
        elif (
            lead.vRel <= 0.1 * v_ego
            and lead.aLeadK < 0.5
            and lead.dRel <= 1.1 * safe_dist_m
        ):
            # going slower AND decelerating
            accel_to_compensate = min(3 * lead.vRel / time_to_brake, -0.7)
            return -2 + lead.aRel + accel_to_compensate
        elif lead.vRel < -0.1 * v_ego and lead.dRel <= 1.1 * safe_dist_m:
            return -3 + 2 * lead.vRel / time_to_brake
        # if we got here, aLeadK >=0 so use the old logic
        decel_map = OrderedDict(
            [
                # (sec to collision, decel)
                (0, 10.0),
                (4, 1.0),
                (7, 0.5),
                (10, 0.3),
            ]
        )
        decel_speed_map = OrderedDict(
            [
                # (m/s, decel)
                (0, 10.0),
                (4, 5.0),
                (7, 2.50),
                (10, 1.0),
            ]
        )
        return (
            accel_min
            * max_speed_mult
            * _interp_map(_sec_til_collision(lead, CS), decel_map)
            * _interp_map(v_ego, decel_speed_map)
        )
    else:
        # BB: if we don't have a lead, don't do full regen to slow down smoother
        return accel_min * 0.5 * max_speed_mult


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