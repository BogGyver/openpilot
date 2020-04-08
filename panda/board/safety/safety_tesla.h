// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      regen paddle
//      accel rising edge
//      brake rising edge
//      brake > 0mph

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number, bool skip_tx_hook);
// 2m/s are added to be less restrictive
const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_UP = {
    {2., 7., 17.},
    {25., 25., 25.}};

const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_DOWN = {
    {2., 7., 17.},
    {25., 26., 25.}};

const struct lookup_t TESLA_LOOKUP_MAX_ANGLE = {
    {2., 29., 38.},
    {500., 500., 500.}};

const AddrBus TESLA_TX_MSGS[] = {//chassis CAN
{0x045,0}, {0x209,0}, {0x219,0}, {0x229,0}, {0x239,0}, {0x249,0}, 
  {0x2B9,0}, {0x309,0}, {0x329,0}, {0x349,0}, {0x369,0}, {0x379,0}, {0x389,0}, 
  {0x399,0}, {0x3A9,0}, {0x3B1,0}, {0x3D9,0}, {0x3E9,0}, {0x400,0}, {0x488,0}, 
  {0x409,0}, {0x551,0}, {0x539,0}, {0x554,0}, {0x556,0}, {0x557,0}, {0x559,0}, 
  {0x560,0}, {0x5D9,0}, {0x5F9,0}, {0x609,0}, {0x639,0}, {0x659,0}, {0x65A,0}, 
  {0x669,0}, {0x720,0}, 
//radar CAN
{0x101,1}, {0x109,1}, {0x119,1}, {0x129,1}, {0x149,1}, {0x159,1}, {0x169,1}, 
  {0x199,1}, {0x1A9,1}, {0x209,1}, {0x219,1}, {0x2A9,1}, {0x2B9,1}, {0x2D9,1}, 
  {0x641,1}, 
//epas CAN
{0x488,2}, {0x551,2},};  

// TODO: do checksum and counter checks. Add correct timestep, 0.1s for now.
AddrCheckStruct tesla_rx_checks[] = {
  {.addr = {0x00E}, .bus =0, .check_checksum = false, .max_counter = 15U, .expected_timestep = 100000U},
  {.addr = {0x045}, .bus =0, .check_checksum = false, .max_counter = 15U, .expected_timestep = 100000U},
  {.addr = {0x108}, .bus =0, .check_checksum = false, .max_counter = 15U, .expected_timestep = 100000U},
  {.addr = {0x118}, .bus =0, .check_checksum = false, .max_counter = 15U, .expected_timestep = 100000U},
  {.addr = {0x318}, .bus =0, .check_checksum = false, .max_counter = 15U, .expected_timestep = 100000U},
  {.addr = {0x348}, .bus =0, .check_checksum = false, .max_counter = 15U, .expected_timestep = 100000U},
  {.addr = {0x368}, .bus =0, .check_checksum = false, .max_counter = 15U, .expected_timestep = 100000U},
};
const int TESLA_RX_CHECK_LEN = sizeof(tesla_rx_checks) / sizeof(tesla_rx_checks[0]);


const uint32_t TESLA_RT_INTERVAL = 250000; // 250ms between real time checks

struct sample_t tesla_angle_meas; // last 3 steer angles

// state of angle limits
int tesla_desired_angle_last = 0; // last desired steer angle
int16_t tesla_rt_angle_last = 0.; // last real time angle
uint32_t tesla_ts_angle_last = 0;

int tesla_controls_allowed_last = 0;
int steer_allowed = 1;
int EON_is_connected = 0;
int tesla_brake_prev = 0;
int tesla_gas_prev = 0;
int tesla_speed = 0;
int current_car_time = -1;
int time_at_last_stalk_pull = -1;
int eac_status = 0;

/* <-- revB giraffe GPIO */
/*
#include "../drivers/uja1023.h"

uint32_t tesla_ts_brakelight_on_last = 0;
const uint32_t BRAKELIGHT_CLEAR_INTERVAL = 250000; //25ms; needs to be slower than the framerate difference between the DI_torque2 (~100Hz) and DI_state messages (~10hz).
const uint32_t STW_MENU_BTN_HOLD_INTERVAL = 750000; //75ms, how long before we recognize the user is  holding this steering wheel button down

uint32_t stw_menu_btn_pressed_ts = 0;
int stw_menu_current_output_state = 0;
int stw_menu_btn_state_last = 0;
int stw_menu_output_flag = 0;
int high_beam_lever_state = 0;
*/
/* revB giraffe GPIO --> */

int tesla_radar_status = 0; //0-not present, 1-initializing, 2-active
uint32_t tesla_last_radar_signal = 0;
const uint32_t TESLA_RADAR_TIMEOUT = 10000000; // 10s second between real time checks
char radar_VIN[] = "                 "; //leave empty if your radar VIN matches the car VIN
int tesla_radar_vin_complete = 0;
uint8_t tesla_radar_can = 1;
uint8_t tesla_epas_can = 2;
int tesla_radar_trigger_message_id = 0; //not used by tesla, to showcase for other cars
int radarPosition = 0; //0 nosecone, 1 facelift
int radarEpasType = 0; //0/1 bosch, 2-4 mando


//settings from bb_openpilot.cfg


int enable_das_emulation = 1;
int enable_radar_emulation = 1;

//for street signs
int stopSignWarning = 0;
int stopLightWarning = 0;
int streetSignObject_b0 = 0;
int streetSignObject_b1 = 0;
int streetSignObject_b2 = 0;
int streetSignObject_b3 = 0;
int streetSignObject_active = 0;

//fake DAS counters
int DAS_uom = 0; //units of measure, 0 = MPH, 1 = km/h
int DAS_bootID_sent = 0;
int fake_DAS_counter = 0;
int DAS_object_idx = 0;
int DAS_control_idx = 0;
int DAS_pscControl_idx = 0;
int DAS_telemetryPeriodic_idx1 = 0;
int DAS_telemetryPeriodic_idx2 = 0;
int DAS_lanes_idx = 0;
int DAS_status_idx = 0;
int DAS_status2_idx = 0;
int DAS_bodyControls_idx = 0;
int DAS_info_idx = 0;
int DAS_warningMatrix0_idx = 0;
int DAS_warningMatrix1_idx = 0;
int DAS_warningMatrix3_idx = 0;
int DAS_steeringControl_idx = 0;
int DAS_diState_idx = 0;
int DAS_longControl_idx = 0;
int DI_locStatus_idx = 0;
int DAS_carLog_idx = 0;
int DAS_telemetry_idx = 0;


//fake DAS variables
int DAS_longC_enabled = 0;
int DAS_speed_limit_kph = 0;
int DAS_accel_min = 0;
int DAS_accel_max = 0;
int DAS_aeb_event = 0x00;
int DAS_acc_state = 0x00;
int DAS_jerk_min = 0x000;
int DAS_jerk_max = 0x0F;
int DAS_gas_to_resume = 0;
int DAS_206_apUnavailable = 0;
int DAS_haptic_request = 0;
uint8_t EPB_epasControl_idx = 0;

//fake DAS objects
int DAS_LEAD_OBJECT_MLB = 0xFFFFFF00;
int DAS_LEAD_OBJECT_MHB = 0x03FFFF83;
int DAS_LEFT_OBJECT_MLB = 0xFFFFFF01;
int DAS_LEFT_OBJECT_MHB = 0x03FFFF83;
int DAS_RIGHT_OBJECT_MLB = 0xFFFFFF02;
int DAS_RIGHT_OBJECT_MHB = 0x03FFFF83;
//fake DAS for DAS_status and DAS_status2
int DAS_op_status = 1;
int DAS_op_status_last_received = 1;
int DAS_alca_state = 0x05;
int DAS_hands_on_state = 0;
int DAS_forward_collision_warning = 0;
int DAS_units_included = 0;
int DAS_cc_state = 0;
int DAS_acc_speed_limit = 0; //in car speed units
int DAS_acc_speed_kph = 0;
int DAS_collision_warning = 0;
int DAS_ldwStatus = 0;

//lanes and objects
int tLeadDx = 0;
int tLeadDy = 0;
int tLeadClass = 0;
int rLine = 0;
int lLine = 0;
int rLine_qual = 0;
int lLine_qual = 0;
int lWidth = 29;
int curvC0 = 0x64;
int curvC1 = 0x7F;
int curvC2 = 0x7F;
int curvC3 = 0x7F;
int laneRange = 160; // max 160m

//fake DAS for telemetry
int DAS_telLeftMarkerQuality = 3; //3-high, 2-medium, 1-low 0-lowest
int DAS_telRightMarkerQuality = 3; //3-high, 2-medium, 1-low 0-lowest
int DAS_telRightLaneType = 3;//0-undecided, 1-solid, 2-road edge, 3-dashed 4-double 5-botts dots 6-barrier
int DAS_telLeftLaneType = 3; //0-undecided, 1-solid, 2-road edge, 3-dashed 4-double 5-botts dots 6-barrier
int DAS_telRightLaneCrossing = 0; //0-not crossing 1-crossing
int DAS_telLeftLaneCrossing = 0; //0-not crossing 1-crossing
int DAS_telLeftMarkerColor = 1; //0-unknown, 1-white, 2-yellow, 3-blue
int DAS_telRightMarkerColor = 1; //0-unknown, 1-white, 2-yellow, 3-blue
//fake DAS for DAS_bodyControls
int DAS_turn_signal_request = 0;

//fake DAS for DAS_steeringControls
int DAS_steeringAngle = 0x4000;
int DAS_steeringEnabled = 0;

//fake DAS controll
int time_last_DAS_data = -1;
int time_last_EPAS_data = -10;

//fake DAS using pedal
int DAS_usingPedal = 0;

//fake DAS - are we in drive?
int DAS_inDrive = 0;
int DAS_inDrive_prev = 0;
int DAS_present = 0;
int tesla_radar_should_send = 0;
int DAS_noEpasHarness = 0;
int DAS_usesApillarHarness=0;

//fake DAS - last stalk data used to cancel
uint32_t DAS_lastStalkL =0x00;
uint32_t DAS_lastStalkH = 0x00;

//fake DAS - pedal pressed (with Pedal)
int DAS_pedalPressed = 0;

//fake DAS - DI_state spamming
uint32_t DAS_diStateL =0x00;
uint32_t DAS_diStateH = 0x00;

//fake DAS - warning messages
int DAS_211_accNoSeatBelt = 0x00;
int DAS_canErrors = 0x00;
int DAS_202_noisyEnvironment = 0x00;
int DAS_doorOpen = 0x00;
int DAS_notInDrive = 0x00;
int DAS_222_accCameraBlind = 0x00;
int DAS_219_lcTempUnavailableSpeed = 0x00;
int DAS_220_lcTempUnavailableRoad = 0x00;
int DAS_207_lkasUnavailable = 0x00;
int DAS_208_rackDetected = 0x00;
int DAS_025_steeringOverride = 0x00;
int DAS_221_lcAborting = 0x00;

//fake DAS - high beam request
int DAS_high_low_beam_request = 0x00;
int DAS_high_low_beam_reason = 0x00;
int DAS_ahb_is_enabled = 0;

//fake DAS - plain CC condition
int DAS_plain_cc_enabled = 0x00;

//fake DAS - emergency brakes use
int DAS_emergency_brake_request = 0x00;
int DAS_fleet_speed_state = 0x00;


static int add_tesla_crc(uint32_t MLB, uint32_t MHB , int msg_len) {
  //"""Calculate CRC8 using 1D poly, FF start, FF end"""
  int crc_lookup[256] = { 0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB, 
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76, 
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C, 
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1, 
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65, 
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F, 
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D, 
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50, 
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A, 
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7, 
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E, 
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09, 
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4 };
  int crc = 0xFF;
  for (int x = 0; x < msg_len; x++) {
    int v = 0;
    if (x <= 3) {
      v = (MLB >> (x * 8)) & 0xFF;
    } else {
      v = (MHB >> ( (x-4) * 8)) & 0xFF;
    }
    crc = crc_lookup[crc ^ v];
  }
  crc = crc ^ 0xFF;
  return crc;
}

static int add_tesla_cksm(CAN_FIFOMailBox_TypeDef *msg , int msg_id, int msg_len) {
  int cksm = (0xFF & msg_id) + (0xFF & (msg_id >> 8));
  for (int x = 0; x < msg_len; x++) {
    int v = 0;
    if (x <= 3) {
      v = (msg->RDLR >> (x * 8)) & 0xFF;
    } else {
      v = (msg->RDHR >> ( (x-4) * 8)) & 0xFF;
    }
    cksm = (cksm + v) & 0xFF;
  }
  return cksm;
}

static int add_tesla_cksm2(uint32_t dl, uint32_t dh, int msg_id, int msg_len) {
  CAN_FIFOMailBox_TypeDef to_check;
  to_check.RDLR = dl;
  to_check.RDHR = dh;
  return add_tesla_cksm(&to_check,msg_id,msg_len);
} 

// interp function that holds extreme values
float tesla_interpolate(struct lookup_t xy, float x)
{
  int size = sizeof(xy.x) / sizeof(xy.x[0]);
  // x is lower than the first point in the x array. Return the first point
  if (x <= xy.x[0])
  {
    return xy.y[0];
  }
  else
  {
    // find the index such that (xy.x[i] <= x < xy.x[i+1]) and linearly interp
    for (int i = 0; i < size - 1; i++)
    {
      if (x < xy.x[i + 1])
      {
        float x0 = xy.x[i];
        float y0 = xy.y[i];
        float dx = xy.x[i + 1] - x0;
        float dy = xy.y[i + 1] - y0;
        // dx should not be zero as xy.x is supposed ot be monotonic
        if (dx <= 0.)
          dx = 0.0001;
        return dy * (x - x0) / dx + y0;
      }
    }
    // if no such point is found, then x > xy.x[size-1]. Return last point
    return xy.y[size - 1];
  }
}

static uint32_t bitShift(int value, int which_octet, int starting_bit_in_octet) {
  //which octet - 1-4
  //starting_bit_in_octet 1-8
  return ( value << ((starting_bit_in_octet - 1) + (which_octet -1) * 8));
}



static void send_fake_message(uint32_t RIR, uint32_t RDTR,int msg_len, int msg_addr, uint8_t bus_num, uint32_t data_lo, uint32_t data_hi) {
  CAN_FIFOMailBox_TypeDef to_send;
  uint32_t addr_mask = 0x001FFFFF;
  to_send.RIR = (msg_addr << 21) + (addr_mask & (RIR | 1));
  to_send.RDTR = (RDTR & 0xFFFFFFF0) | msg_len;
  to_send.RDLR = data_lo;
  to_send.RDHR = data_hi;
  can_send(&to_send, bus_num, false);
}

static void reset_DAS_data(void) {
  //fake DAS variables
  DAS_present = 0;
  DAS_longC_enabled = 0;
  DAS_speed_limit_kph = 0;
  DAS_accel_min = 0;
  DAS_accel_max = 0;
  DAS_aeb_event = 0x00;
  DAS_acc_state = 0x00;
  DAS_jerk_min = 0x000;
  DAS_jerk_max = 0x0F;
  DAS_gas_to_resume = 0;
  DAS_206_apUnavailable = 0;
  DAS_haptic_request = 0;
  DAS_op_status = 1; //unavailable
  DAS_alca_state = 0x05;
  DAS_hands_on_state = 0;
  DAS_forward_collision_warning = 0;
  DAS_cc_state = 0;
  DAS_acc_speed_limit = 0;
  DAS_acc_speed_kph = 0;
  DAS_collision_warning = 0;
  DAS_telLeftMarkerQuality = 3; //3-high, 2-medium, 1-low 0-lowest
  DAS_telRightMarkerQuality = 3; //3-high, 2-medium, 1-low 0-lowest
  DAS_telRightLaneType = 3; //0-undecided, 1-solid, 2-road edge, 3-dashed 4-double 5-botts dots 6-barrier
  DAS_telLeftLaneType = 3; //0-undecided, 1-solid, 2-road edge, 3-dashed 4-double 5-botts dots 6-barrier
  DAS_telRightLaneCrossing = 0; //0-not crossing 1-crossing
  DAS_telLeftLaneCrossing = 0; //0-not crossing 1-crossing
  DAS_telLeftMarkerColor = 1; //0-unknown, 1-white, 2-yellow, 3-blue
  DAS_telRightMarkerColor = 1; //0-unknown, 1-white, 2-yellow, 3-blue
  DAS_turn_signal_request = 0;
  DAS_high_low_beam_request = 0;
  DAS_high_low_beam_reason = 0;
  DAS_steeringAngle = 0x4000;
  DAS_steeringEnabled = 0;
  DAS_usingPedal = 0;
  DAS_lastStalkL =0x00;
  DAS_lastStalkH = 0x00;
  DAS_pedalPressed = 0;
  DAS_diStateL =0x00;
  DAS_diStateH = 0x00;
  DAS_211_accNoSeatBelt = 0x00;
  DAS_canErrors = 0x00;
  DAS_202_noisyEnvironment = 0x00;
  DAS_doorOpen = 0x00;
  DAS_notInDrive = 0x00;
  DAS_222_accCameraBlind = 0x00;
  DAS_219_lcTempUnavailableSpeed = 0x00;
  DAS_220_lcTempUnavailableRoad = 0x00;
  DAS_207_lkasUnavailable = 0x00;
  DAS_208_rackDetected = 0x00;
  DAS_025_steeringOverride = 0x00;
  DAS_221_lcAborting = 0x00;
  //reset fake DAS objects
  DAS_LEAD_OBJECT_MLB = 0xFFFFFF00;
  DAS_LEAD_OBJECT_MHB = 0x03FFFF83;
  DAS_LEFT_OBJECT_MLB = 0xFFFFFF01;
  DAS_LEFT_OBJECT_MHB = 0x03FFFF83;
  DAS_RIGHT_OBJECT_MLB = 0xFFFFFF02;
  DAS_RIGHT_OBJECT_MHB = 0x03FFFF83;
  DAS_plain_cc_enabled = 0x00;
  DAS_emergency_brake_request = 0x00;
  DAS_fleet_speed_state = 0x00;
}


static void do_fake_DAS(uint32_t RIR, uint32_t RDTR) {

  fake_DAS_counter++;
  fake_DAS_counter = fake_DAS_counter % 300;
  uint32_t MLB = 0;
  uint32_t MHB = 0;

  //check if we got data from OP in the last two seconds
  if (current_car_time - time_last_DAS_data > 2)  {
    //no message in the last 2 seconds, reset all variables
    reset_DAS_data();
    if (EON_is_connected == 1) {
      //reset_DAS_data();
      EON_is_connected  = 0;
    }
  } else {
    EON_is_connected = 1;
  }

  //ldw type of warning... use haptic if we manually steer
  if (DAS_ldwStatus > 0) {
    DAS_haptic_request = 1;
    //if OP not enabled or manual steering, only use haptic
    if (((DAS_219_lcTempUnavailableSpeed == 1) || (DAS_op_status < 5)) && (DAS_ldwStatus > 2)) {
      DAS_ldwStatus -= 2;
    } 
    //if OP enabled and no manual steering, always force audible alert
    if ((DAS_219_lcTempUnavailableSpeed == 0)  && (DAS_op_status == 5) && (DAS_ldwStatus > 0) && (DAS_ldwStatus < 3)) {
      DAS_ldwStatus += 2;
    }
  } else {
    DAS_haptic_request = 0;
  }

  if (fake_DAS_counter % 2 == 0) {
    //send DAS_steeringControl - 0x488
    MHB = 0x00;
    MLB = ((DAS_steeringAngle >> 8) & 0x7F) + (DAS_haptic_request << 7) +
        ((DAS_steeringAngle & 0xFF) << 8) + 
        (((((DAS_steeringEnabled & controls_allowed & DAS_inDrive)  )<< 6) + DAS_steeringControl_idx) << 16);
    int cksm = add_tesla_cksm2(MLB, MHB, 0x488, 3);
    MLB = MLB + (cksm << 24);
    if (DAS_noEpasHarness == 0) {
      send_fake_message(RIR,RDTR,4,0x488,2,MLB,MHB);
    }
    send_fake_message(RIR,RDTR,4,0x488,0,MLB,MHB);
    DAS_steeringControl_idx ++;
    DAS_steeringControl_idx = DAS_steeringControl_idx % 16;
  }


  if (enable_das_emulation == 0) {
    return;
  }

  //if we never sent DAS_bootID message, send it now
  if (DAS_bootID_sent == 0) {
    MLB = 0x00E30055;
    MHB = 0x002003BD;
    send_fake_message(RIR,RDTR,8,0x639,0,MLB,MHB);
    DAS_bootID_sent = 1;
  }

  if (fake_DAS_counter == 299) {
    //send DAS_carLog  0x5D9
    if (DAS_carLog_idx == 0) {
      MLB = 0x00041603;
      MHB = 0x00000000;
    }
    if (DAS_carLog_idx == 1) {
      MLB = 0x00010002;
      MHB = 0x00000000;
    }
    if (DAS_carLog_idx == 2) {
      MLB = 0x00000001;
      MHB = 0x00000000;
    }
    send_fake_message(RIR,RDTR,8,0x5D9,0,MLB,MHB);
    DAS_carLog_idx ++;
    DAS_carLog_idx = DAS_carLog_idx % 3;
  }



  if (fake_DAS_counter % 3 == 0) {
    //send DAS_object - 0x309
    //fix - 0x81,0xC0,0xF8,0xF3,0x43,0x7F,0xFD,0xF1
    //when idx==0 is lead vehicle
    if (DAS_object_idx == 0) {
      MLB = DAS_LEAD_OBJECT_MLB;
      MHB = DAS_LEAD_OBJECT_MHB;
    }
    if (DAS_object_idx == 1) {
      MLB = DAS_LEFT_OBJECT_MLB;
      MHB = DAS_LEFT_OBJECT_MHB;
    }
    if (DAS_object_idx == 2) {
      MLB = DAS_RIGHT_OBJECT_MLB;
      MHB = DAS_RIGHT_OBJECT_MHB;
    }
    if (DAS_object_idx == 3) {
      MLB = 0xFFFFFF03;
      MHB = 0x00000003;
    }
    //when idx == 4 we send street sign if it is valid
    if (DAS_object_idx == 4) {
      if (streetSignObject_active == 1) {
        MLB = (streetSignObject_b3 << 24) + (streetSignObject_b2 << 16) + (streetSignObject_b1 << 8) + (streetSignObject_b0 << 0);
        MHB = 0x00000000;
      } else {
        DAS_object_idx = 5;
      }

    }
    if (DAS_object_idx == 5) {
      MLB = 0xFFFFFF05;
      MHB = 0xFFFFFFFF;
    }
    send_fake_message(RIR,RDTR,8,0x309,0,MLB,MHB);
    DAS_object_idx++;
    DAS_object_idx = DAS_object_idx % 6;
  }
  
  if (fake_DAS_counter % 4 == 0) {
    int acc_speed_kph = 0xFFF;
    int acc_state = 0x00;
    int aeb_event = 0x00;
    int jerk_min = 0x1FF;
    int jerk_max = 0xFF;
    int accel_min = 0x1FF;
    int accel_max = 0x1FF;
    //send DAS_control - 0x2B9
    //when not in drive it should send FF 0F FE FF FF FF XF YY - X counter YY Checksum
    if (DAS_inDrive ==1) {
      acc_state = 0x04;
      if (DAS_longC_enabled > 0) {
        jerk_min = 0x000;
        jerk_max = 0x0F;
        acc_state = 0x04;//bb send HOLD?
        float uom = 1;
        if ((DAS_uom == 0) && (DAS_units_included == 0)) {
          uom = 1.609;
        }
        acc_speed_kph = (int)(DAS_acc_speed_limit * uom * 10.0);
        accel_max = 0x1FE; //(int)((DAS_accel_max + 15 ) / 0.04);
        accel_min = 0x001; //(int)((DAS_accel_min + 15 ) / 0.04);
      }
      MLB = (0xff & acc_speed_kph) + 
        (((acc_state << 4) + ((acc_speed_kph & 0xF00) >> 8)) << 8) +
        ((((jerk_min << 2) & 0xFF) + aeb_event) << 16 ) +
        ((((jerk_max << 3) & 0xFF) +((jerk_min >> 6) & 0x07)) << 24);
      MHB = ((((accel_min << 3) & 0xFF) + ((jerk_max >> 5) & 0x07)) ) +
        ((((accel_max << 4) & 0xFF) + ((accel_min >> 5) & 0x0F)) << 8) +
        (((DAS_control_idx << 5) + ((accel_max >> 4) & 0x1F))<< 16);
    } else {
      MLB = 0xFFFE0FFF;
      MHB = 0x001FFFFF + (DAS_control_idx << 21);
    }
    int cksm = add_tesla_cksm2(MLB, MHB, 0x2B9, 7);
    MHB = MHB + (cksm << 24);
    send_fake_message(RIR,RDTR,8,0x2B9,0,MLB,MHB);
    DAS_control_idx++;
    DAS_control_idx = DAS_control_idx % 8;
    //send DAS_longControl - 0x209
    if (DAS_inDrive == 1) {
      int das_loc_mode = 0x01;//normal
      int das_loc_state = 0x00;//healthy
      int das_loc_request = 0x00; //idle
      jerk_min = 0xFF;
      jerk_max = 0xFF;
      accel_min = 0x1FF;
      accel_max = 0x1FF;
      acc_speed_kph = 0x7FF;
      if (DAS_longC_enabled > 0) {
        acc_state = 0x04;
        jerk_min = 0x01;
        jerk_max = 0xFE;
        acc_speed_kph = (int)(DAS_acc_speed_kph * 10.0);
        das_loc_request = 0x01;//forward
        accel_min = 0x001;
        accel_max = 0x1FE;
      }
      MLB = das_loc_mode + (das_loc_state << 2) + (das_loc_request << 5) +
         (jerk_min << 8) + (jerk_max << 16) + ((acc_speed_kph  & 0xFF) << 24);
      MHB = ((acc_speed_kph  >> 8) & 0x07) + ((accel_min << 3) & 0xFF) +
          ((((accel_min >>5) & 0x0F)  + ((accel_max << 4) & 0xFF))<< 8) +
          ((((accel_max >> 4) & 0x1F) + (DAS_longControl_idx << 5)) << 16);
    } else {
      MLB = 0xFFFFFF00;
      MHB = 0x001FFFFF + (DAS_longControl_idx << 21);
    }
    cksm = add_tesla_cksm2(MLB, MHB, 0x209, 7);
    MHB = MHB + (cksm << 24);
    //send_fake_message(RIR,RDTR,8,0x209,0,MLB,MHB);
    DAS_longControl_idx++;
    DAS_longControl_idx = DAS_longControl_idx % 8;
  }
  if (fake_DAS_counter % 4 == 2) {
    //send DAS_pscControl - 0x219
    // 0x90 + DAS_pscControl_idx,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    uint8_t eacStatus = 1;
    if ((DAS_steeringEnabled & controls_allowed & DAS_inDrive) == 1) {
      eacStatus = 2;
    }
    //BB disabling to see if this is the issue
    eacStatus = 0;
    MLB = 0x90 + DAS_pscControl_idx + (eacStatus << 8);
    MHB = 0x00;
    int cksm = add_tesla_cksm2(MLB, MHB, 0x219, 2);
    MLB = MLB + (cksm << 16);
    send_fake_message(RIR,RDTR,3,0x219,0,MLB,MHB);
    DAS_pscControl_idx++;
    DAS_pscControl_idx = DAS_pscControl_idx % 16;

    //send DAS_telemetryFurniture - 0x3B1
    //NOT SENDING FOR NOW
  }

  if (fake_DAS_counter % 4 == 3) {
    //send DAS_telemetryPeriodic - 0x379
    //static for now
    switch (DAS_telemetryPeriodic_idx2) {
      case 0: //Y0 3A 83 D3 E8 07 6D 11
        MLB = 0xd3833a00;
        MHB = 0x116d07e8;
        break;
      case 1: //Y1 E1 D7 E6 FB BF 7D 00
        MLB = 0xE6D7E101;
        MHB = 0x007DBFFB;
        break;
      case 2: //42 49 F4 27 FF D1 7D 00
        MLB = 0x27F44902;
        MHB = 0x007DD1FF;
        break;
      case 3: //43 D7 AB A7 FF F3 FE C0
        MLB = 0xA7ABD703;
        MHB = 0xC0FEF3FF;
        break;
      case 4: //44 7B FB B7 F1 33 83 FF
        MLB = 0xB7FB7B04;
        MHB = 0xFF8333F1;
        break;
      case 5: //45 EA C7 5F F7 27 83 40
        MLB = 0x5Fc7EA05;
        MHB = 0x408327F7;
        break;
      case 6: //46 01 48 00 00 00 00 00
        MLB = 0x00480106;
        MHB = 0x00000000;
        break;
      case 7: //47 01 48 00 00 00 00 00
        MLB = 0x00480107;
        MHB = 0x00000000;
        break;
      case 8: //48 00 00 00 00 00 00 00
        MLB = 0x00000008;
        MHB = 0x00000000;
        break;
      default: //49 FA 37 00 00 00 00 08
        MLB = 0x0037FA09;
        MHB = 0x08000000;
        break;
    } 
    MLB = MLB + (DAS_telemetryPeriodic_idx2 << 5);
    //send_fake_message(RIR,RDTR,1,0x379,0,MLB,MHB);
    DAS_telemetryPeriodic_idx2++;
    DAS_telemetryPeriodic_idx2 = DAS_telemetryPeriodic_idx2 % 10;
    if (DAS_telemetryPeriodic_idx2 == 0) {
      DAS_telemetryPeriodic_idx1 += 1;
      DAS_telemetryPeriodic_idx1 = DAS_telemetryPeriodic_idx1 % 8;
    }
    
  }

  if (fake_DAS_counter % 6 == 0) {
    //send DAS_integratedSafetyFront - 0x299
    //NOT SENDING FOR NOW

  }

  if (fake_DAS_counter % 10 == 3) {
    //send DAS_lanes - 0x239
    //for now fixed 0x33,0xC8,0xF0,0x7F,0x70,0x70,0x33,(idx << 4)+0x0F
    int fuse = 2;
    MLB = (lWidth << 4) + (rLine << 1) + lLine + (laneRange << 8) + (curvC0 << 16) + (curvC1  << 24);
    MHB = 0x00F00000 + (DAS_lanes_idx << 28) + curvC2 + (curvC3 << 8) + ((((rLine * fuse) << 2) + (lLine * fuse)) << 16); //0x0F000000
    send_fake_message(RIR,RDTR,8,0x239,0,MLB,MHB);
    DAS_lanes_idx ++;
    DAS_lanes_idx = DAS_lanes_idx % 16;

    //send DI_locStatus2 - 0x4F6
    //hoping to show speed for ACC in IC
    int DI_crsState = 0x01;
    int DI_locState = 0x01;
    int DI_locSpeed_kph = 0x00;
    if (DAS_longC_enabled > 0) {
      DI_locSpeed_kph = (int)(DAS_acc_speed_kph * 2.0);
      DI_crsState = 0x02;
      DI_locState = 0x02;
    }
    MLB = 0x00 + ((DI_locStatus_idx << 8)  & 0x0F) + 
        (DI_crsState << 12) +  (DI_locState << 16) +
        ((DI_locSpeed_kph  & 0x1F) << 19) +
        ((DI_locSpeed_kph >> 5) & 0x0F);
    MHB = 0x00;
    int cksm = add_tesla_cksm2(MLB, MHB, 0x4F6, 4);
    MLB = MLB + cksm;
    DI_locStatus_idx += 1;
    DI_locStatus_idx = DI_locStatus_idx % 16;
    //send_fake_message(RIR,RDTR,4,0x4F6,0,MLB,MHB);
  }

  if (fake_DAS_counter % 10 == 4) {
    //send DAS_telemetry - 0x3A9
    //no counter - 00 00 00 00 00 00 00 00
    //ROAD INFO
    //if (DAS_telemetry_idx==0) {
      DAS_telRightMarkerQuality = rLine_qual;
      DAS_telLeftMarkerQuality = lLine_qual;
      if (rLine == 1) {
        DAS_telRightLaneType = 3;
      } else {
        DAS_telRightLaneType = 7;
      }
      if (lLine == 1) {
        DAS_telLeftLaneType = 3;
      } else {
        DAS_telLeftLaneType = 7;
      }
      if (DAS_alca_state == 0x09) {
        //alca in progress, left
        DAS_telRightLaneCrossing = 0;
        DAS_telLeftLaneCrossing = 1;
      } else if (DAS_alca_state == 0x0a) {
        //alca in progress, right
        DAS_telRightLaneCrossing = 1;
        DAS_telLeftLaneCrossing = 0;
      } else {
        DAS_telRightLaneCrossing = 0;
        DAS_telLeftLaneCrossing = 0;
      }
      MLB = 0x00 + ((DAS_telLeftLaneType + (DAS_telRightLaneType << 3) + (DAS_telLeftMarkerQuality  << 6)) << 8 ) +
          ((DAS_telRightMarkerQuality +(DAS_telLeftMarkerColor << 2) + 
          (DAS_telRightMarkerColor << 4) + (DAS_telLeftLaneCrossing << 6) +(DAS_telRightLaneCrossing <<7)) << 16);
      MHB =0x00;
    //}
    //ACC DRIVER MONITORING
    //if (DAS_telemetry_idx==1) {
    //  MLB=0x01;
    //  MHB=0x00;
    //}
    //TRAFFIC SIGNS
    //if (DAS_telemetry_idx==2) {
    //  MLB=0x02;
    //  MHB=0x00;
    //}
    //CSA EVENT
    //if (DAS_telemetry_idx==3) {
    //  MLB=0x07;
    //  MHB=0x00;
    //}
    //STATIONARY OBJECT
    //if (DAS_telemetry_idx==4) {
    //  MLB=0x08;
    //  MHB=0x00;
    //CRUISE STALK
    //if (DAS_telemetry_idx==5) {
    //  MLB=0x09;
    //  MHB=0x00;
    //}
    DAS_telemetry_idx ++;
    DAS_telemetry_idx = DAS_telemetry_idx % 6;
    send_fake_message(RIR,RDTR,8,0x3A9,0,MLB,MHB);
  }

  if (fake_DAS_counter % 10 == 5) {
    //send DAS_chNm - 0x409
    //no counter
    MLB = 0x00;
    MHB = 0x00;
    send_fake_message(RIR,RDTR,1,0x409,0,MLB,MHB);

    //send DAS_telemetryEvent - 0x3D9
    //NOT SENDING FOR NOW

    //send DAS_visualDebug - 0x249
    //no counter - 8A 06 21 1F 00 00 00 00
    //80 04 21 10 40 00 00 01
    //7th octet should be 05 when acc enabled and 01 when not?
    int acc = 0x01;
    if (DAS_cc_state >= 2) {
      acc = 0x05;
    }
    MLB = 0x10210480;
    MHB = 0x01000040 + (acc << 16);
    send_fake_message(RIR,RDTR,8,0x249,0,MLB,MHB);

  }
    

  if (fake_DAS_counter % 50 == 0) {
    //send DAS_status - 0x399
    MLB = DAS_op_status + 0xF0 + (DAS_speed_limit_kph << 8) + (((DAS_collision_warning << 6) + DAS_speed_limit_kph) << 16);
    MHB = ((DAS_fleet_speed_state & 0x03) << 3) + (DAS_ldwStatus << 5) + 
        (((DAS_hands_on_state << 2) + ((DAS_alca_state & 0x03) << 6) + DAS_fleet_speed_state) << 8) +
       ((( DAS_status_idx << 4) + (DAS_alca_state >> 2)) << 16);
    int cksm = add_tesla_cksm2(MLB, MHB, 0x399, 7);
    MHB = MHB + (cksm << 24);
    send_fake_message(RIR,RDTR,8,0x399,0,MLB,MHB);
    DAS_status_idx ++;
    DAS_status_idx = DAS_status_idx % 16;

    //send DAS_status2 - 0x389
    float uom = 1.0;
    if ((DAS_uom == 1) && (DAS_units_included == 1)){
      uom = 1.0/1.609;
    }
    int sl2 = ((int)(DAS_acc_speed_limit * uom / 0.2)) & 0x3FF;
    if (sl2 == 0) {
      sl2 = 0x3FF;
    }
    MLB = sl2;
    int lcw = 0x0F;
    if (DAS_forward_collision_warning > 0) {
        lcw = 0x01;
    }
    int b4 = 0x80;
    int b5 = 0x13; 
    if (DAS_cc_state > 1) { //enabled or hold
        b4 = 0x84;
        b5 = 0x25; //BB lssState 0x 0x03 should be LSS_STATE_ELK enhanced LK
        int _DAS_RobState = 0x02; //active
        int _DAS_radarTelemetry = 0x01; // normal 
        int _DAS_lssState = 0x03; 
        int _DAS_acc_report = 0x01; //ACC_report_target_CIPV
        if (DAS_fleet_speed_state == 2) {
          _DAS_acc_report = 0x12; //ACC_report_fleet_speed
        }
        b4 = (_DAS_acc_report << 2) + ((_DAS_lssState & 0x01) << 7);
        b5 =((_DAS_lssState >> 1) & 0x01) + (_DAS_radarTelemetry << 2) + (_DAS_RobState <<4);
    }
    MLB = MLB + (b4 << 24);
    MHB = 0x8000 + b5 + (DAS_status2_idx << 20) + (lcw << 16);
    cksm = add_tesla_cksm2(MLB, MHB, 0x389, 7);
    MHB = MHB + (cksm << 24);
    send_fake_message(RIR,RDTR,8,0x389,0,MLB,MHB);
    DAS_status2_idx ++;
    DAS_status2_idx = DAS_status2_idx % 16;
  }
  if (fake_DAS_counter % 50 == 9) {
    //send DAS_bodyControls - 0x3E9
    //0xf1,0x0c + turn_signal_request,0x00,0x00,0x00,0x00,(idx << 4)
    MLB = 0x000000F1 + (DAS_turn_signal_request << 8) + (DAS_high_low_beam_request << 10) + (DAS_high_low_beam_reason << 12);
    MHB = 0x00 + (DAS_bodyControls_idx << 20);
    int cksm = add_tesla_cksm2(MLB, MHB, 0x3E9, 7);
    MHB = MHB + (cksm << 24);
    send_fake_message(RIR,RDTR,8,0x3E9,0,MLB,MHB);
    DAS_bodyControls_idx ++;
    DAS_bodyControls_idx = DAS_bodyControls_idx % 16;
  }

  if (fake_DAS_counter % 100 == 0) {
    //send DAS_dtcMatrix - 0x669
    //NOT SENDING FOR NOW

    //send DAS_info - 0x539
    switch (DAS_info_idx) {
      case 0: 
        MLB = 0x0003010a;
        MHB = 0x004e0000;
        break;
      case 1: 
        MLB = 0x0102000b;
        MHB = 0x00000001;
        break;
      case 2:
        MLB = 0x0000000d;
        MHB = 0x3eac8e5b;
        break;
      case 3:
        MLB = 0xc9060010;
        MHB = 0x000021b9;
        break;
      case 4:
        MLB = 0x00000011;
        MHB = 0x00000000;
        break;
      case 5:
        MLB = 0x34577f12;
        MHB = 0x0b89706f;
        break;
      case 6:
        MLB = 0xffff0113;
        MHB = 0x0000fcff;
        break;
      case 7:
        MLB = 0x00000514;
        MHB = 0xc309fce5;
        break;
      case 8:
        MLB = 0xe7700017;
        MHB = 0x000091c4;
        break;
      default:
        MLB = 0xb2d50018;
        MHB = 0x0000786c;
        break;
    } 
    send_fake_message(RIR,RDTR,8,0x539,0,MLB,MHB);
    DAS_info_idx ++;
    DAS_info_idx = DAS_info_idx % 10;
  }
  if (fake_DAS_counter % 100 == 45) {
    //send DAS_warningMatrix0 - 0x329
    MLB = 0x00 + bitShift(DAS_canErrors,4,7) + bitShift(DAS_025_steeringOverride,4,1);
    MHB = 0x00 + bitShift(DAS_notInDrive,2,2);
    send_fake_message(RIR,RDTR,8,0x329,0,MLB,MHB);
    DAS_warningMatrix0_idx ++;
    DAS_warningMatrix0_idx = DAS_warningMatrix0_idx % 16;

    //send DAS_warningMatrix1 - 0x369
    MLB = 0x00 ;
    MHB = 0x00;
    send_fake_message(RIR,RDTR,8,0x369,0,MLB,MHB);
    DAS_warningMatrix1_idx ++;
    DAS_warningMatrix1_idx = DAS_warningMatrix1_idx % 16;

    //send DAS_warningMatrix3 - 0x349
    int ovr = 0;
    int lcAborting = 0;
    int lcUnavailableSpeed = 0;
    if (DAS_alca_state == 0x14) {
      //lcAborting == 1;
    }
    if (DAS_alca_state == 0x05) {
      //lcUnavailableSpeed = 1;
    }
    if ((DAS_cc_state == 2) && (DAS_pedalPressed > 10)) {
      ovr = 1;
    }

    MLB = 0x00 + bitShift(DAS_gas_to_resume,1,2) +  bitShift(DAS_206_apUnavailable,2,6)  + bitShift(ovr,3,8) +
         bitShift(lcAborting,2,1) + bitShift(lcUnavailableSpeed,4,3) + bitShift(DAS_211_accNoSeatBelt,3,3) + bitShift(DAS_202_noisyEnvironment,4,6) +
         bitShift(stopSignWarning,1,4) + bitShift(stopLightWarning,1,5) + bitShift(DAS_207_lkasUnavailable,2,7) + bitShift(DAS_219_lcTempUnavailableSpeed,4,3) +
         bitShift(DAS_220_lcTempUnavailableRoad,4,4) + bitShift(DAS_221_lcAborting,4,5) + bitShift(DAS_222_accCameraBlind,4,6) + bitShift(DAS_208_rackDetected,2,8);
    MHB = 0x00;
    send_fake_message(RIR,RDTR,8,0x349,0,MLB,MHB);
    DAS_warningMatrix3_idx ++;
    DAS_warningMatrix3_idx = DAS_warningMatrix3_idx % 16;
  }
}

static void do_EPB_epasControl(uint32_t RIR, uint32_t RDTR) {
  uint32_t MLB;
  uint32_t MHB; 
  MLB = 0x01 + (EPB_epasControl_idx << 8); 
  MHB = 0x00;
  int cksm = add_tesla_cksm2(MLB, MHB, 0x214, 2);
  MLB = MLB + (cksm << 16);
  if (DAS_noEpasHarness == 0) {
    send_fake_message(RIR,RDTR,3,0x214,2,MLB,MHB);
  } else {
    send_fake_message(RIR,RDTR,3,0x214,0,MLB,MHB);
  }
  EPB_epasControl_idx++;
  EPB_epasControl_idx = EPB_epasControl_idx % 16;
}

static void do_fake_stalk_cancel(uint32_t RIR, uint32_t RDTR) {
  uint32_t MLB;
  uint32_t MHB; 
  if ((DAS_lastStalkL == 0x00) && (DAS_lastStalkH == 0x00)) {
    return;
  }
  MLB = (DAS_lastStalkL & 0xFFFFFFC0) + 0x01;
  MHB = (DAS_lastStalkH & 0x000FFFFF);
  int idx = (DAS_lastStalkH & 0xF00000 ) >> 20;
  idx = ( idx + 1 ) % 16;
  MHB = MHB + (idx << 20);
  int crc = add_tesla_crc(MLB, MHB,7);
  MHB = MHB + (crc << 24);
  DAS_lastStalkH = MHB;
  send_fake_message(RIR,RDTR,8,0x45,0,MLB,MHB);
}



static int tesla_rx_hook(CAN_FIFOMailBox_TypeDef *to_push)
{
  bool valid = false;
  //not much to check yet on these, each tesla messaage has checksum in a different place
  

  if (DAS_noEpasHarness == 0) {
    set_gmlan_digital_output(GMLAN_HIGH);
    reset_gmlan_switch_timeout(); //we're still in tesla safety mode, reset the timeout counter and make sure our output is enabled
  }
  uint32_t ts = TIM2->CNT;

  uint8_t bus_number = (to_push->RDTR >> 4) & 0xFF;
  uint32_t addr;

  if (to_push->RIR & 4)
  {
    // Extended
    // Not looked at, but have to be separated
    // to avoid address collision
    addr = to_push->RIR >> 3;
  }
  else
  {
    // Normal
    addr = to_push->RIR >> 21;
  }
  if (bus_number == 1) {
    //radar is always accepted
    valid = true;
  } else 
  if ((bus_number == 0) || (bus_number == 2)) {
    if ((addr == 0x00E ) || (addr == 0x045 ) || (addr == 0x108 ) || 
    (addr == 0x118 ) || (addr == 0x318 ) || (addr == 0x348 ) || (addr == 0x368))
    {
      valid = addr_safety_check(to_push, tesla_rx_checks, TESLA_RX_CHECK_LEN,
                              NULL, NULL, NULL);
    } else {
      valid = true;
    }

  }

  //let's see if the pedal was pressed
  if ((addr == 0x552) && (bus_number == tesla_epas_can)) {
    //m1 = 0.050796813
    //m2 = 0.101593626
    //d = -22.85856576
    DAS_pedalPressed = (int)((((to_push->RDLR & 0xFF00) >> 8) + ((to_push->RDLR & 0xFF) << 8)) * 0.050796813 -22.85856576);
  }

  //we use 0x108 at 100Hz to detect timing of messages sent by our fake DAS and EPB
  if ((addr == 0x108)  && (bus_number == 0)) {
    if (fake_DAS_counter % 10 == 5) {
      do_EPB_epasControl(to_push->RIR,to_push->RDTR);
    }
    do_fake_DAS(to_push->RIR,to_push->RDTR);
    return valid;
  }

  // Record the current car time in current_car_time (for use with double-pulling cruise stalk)
  if ((addr == 0x318) && (bus_number == 0)) {
    int hour = (to_push->RDLR & 0x1F000000) >> 24;
    int minute = (to_push->RDHR & 0x3F00) >> 8;
    int second = (to_push->RDLR & 0x3F0000) >> 16;
    current_car_time = (hour * 3600) + (minute * 60) + second;
  }

  //we use EPAS_sysStatus 0x370 to determine if the car is off or on 
  if ((addr == 0x370) && (bus_number != 1)) {
    time_last_EPAS_data = current_car_time;
  }

  //we just passed EPAS checkpoint let's see if we have timeout and send car off message
  if (current_car_time - time_last_EPAS_data > 2) {
    //no message in the last 2 seconds, car is off
    // GTW_status
    time_last_EPAS_data = -10;
  } else {

  }

  //see if cruise is enabled [Enabled, standstill or Override] and cancel if using pedal
  if ((addr == 0x368)  && (bus_number == 0)) {
    //first save values for spamming
    DAS_diStateL = to_push->RDLR;
    DAS_diStateH = to_push->RDHR;
    DAS_diState_idx = (DAS_diStateH & 0xF000 ) >> 12;
    int acc_state = ((to_push->RDLR & 0xF000) >> 12);
    DAS_uom = (to_push->RDLR >> 31) & 1;
    //if pedal and CC looks enabled, disable
    if ((DAS_usingPedal == 1) && ( acc_state >= 2) && ( acc_state <= 4)) {
      do_fake_stalk_cancel(to_push->RIR, to_push->RDTR);
    } 
    //if ACC not enabled but CC is enabled, disable if more than 2 seconds since last pull
    if ((EON_is_connected == 1) && (DAS_usingPedal == 0) && (DAS_cc_state != 2) && ( acc_state >= 2) && ( acc_state <= 4)) {
      //disable if more than two seconds since last pull, or there was never a stalk pull
      if (((current_car_time >= time_at_last_stalk_pull + 2) && (current_car_time != -1) && (time_at_last_stalk_pull != -1)) || (time_at_last_stalk_pull == -1)) {
        if (DAS_plain_cc_enabled == 0) {
          do_fake_stalk_cancel(to_push->RIR, to_push->RDTR);
        }
      }
    }
  }

  //looking for radar messages;
  if ((addr == 0x300) && (bus_number == tesla_radar_can)) 
  {
    uint32_t ts = TIM2->CNT;
    uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_last_radar_signal);
    if (tesla_radar_status == 1) {
      tesla_radar_status = 2;
      puts("Tesla Radar Active! \n");
      tesla_last_radar_signal = ts;
    } else
    if ((ts_elapsed > TESLA_RADAR_TIMEOUT) && (tesla_radar_status > 0)) {
      tesla_radar_status = 0;
      puts("Tesla Radar Inactive! (timeout 1) \n");
    } else 
    if ((ts_elapsed <= TESLA_RADAR_TIMEOUT) && (tesla_radar_status == 2)) {
      tesla_last_radar_signal = ts;
    }
  }

  //0x631 is sent by radar to initiate the sync
  if ((addr == 0x631) && (bus_number == tesla_radar_can))
  {
    uint32_t ts = TIM2->CNT;
    uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_last_radar_signal);
    if (tesla_radar_status == 0) {
      tesla_radar_status = 1;
      tesla_last_radar_signal = ts;
      puts("Tesla Radar Initializing... \n");
    } else
    if ((ts_elapsed > TESLA_RADAR_TIMEOUT) && (tesla_radar_status > 0)) {
      tesla_radar_status = 0;
      puts("Tesla Radar Inactive! (timeout 2) \n");
    } else 
    if ((ts_elapsed <= TESLA_RADAR_TIMEOUT) && (tesla_radar_status > 0)) {
      tesla_last_radar_signal = ts;
    }
  }

  if ((addr == 0x45)  && (bus_number == 0)) {
    //first save for future use
    DAS_lastStalkL = to_push->RDLR;
    DAS_lastStalkH = to_push->RDHR;
    // 6 bits starting at position 0
    int ap_lever_position = GET_BYTE(to_push, 0) & 0x3F;
    if (ap_lever_position == 2)
    { // pull forward
      // activate openpilot
      // TODO: uncomment the if to use double pull to activate
      //if (current_car_time <= time_at_last_stalk_pull + 1 && current_car_time != -1 && time_at_last_stalk_pull != -1) {
      controls_allowed = 1;
      //}
      time_at_last_stalk_pull = current_car_time;
    }
    else if (ap_lever_position == 1)
    { // push towards the back
      // deactivate openpilot
      controls_allowed = 0;
    }
    //if using pedal, send a cancel immediately to cancel the pedal
    if ((DAS_usingPedal == 1) && (ap_lever_position > 1)) {
      do_fake_stalk_cancel(to_push->RIR, to_push->RDTR);
    }
    /* <-- revB giraffe GPIO */
    /*
    int turn_signal_lever = (to_push->RDLR >> 16) & 0x3; //TurnIndLvr_Stat : 16|2@1+
    int stw_menu_button = (to_push->RDHR >> 5) & 0x1; //StW_Sw05_Psd : 37|1@1+
    high_beam_lever_state = (to_push->RDLR >> 18) & 0x3; //SG_ HiBmLvr_Stat : 18|2@1+
    
    //TurnIndLvr_Stat 3 "SNA" 2 "RIGHT" 1 "LEFT" 0 "IDLE" ;
    if (turn_signal_lever == 1)
    {
      //Left turn signal is on, turn on output pin 3
      set_uja1023_output_bits(1 << 3);
      //puts(" Left turn on!\n");
    }
    else
    {
      clear_uja1023_output_bits(1 << 3);
    }
    if (turn_signal_lever == 2)
    {
      //Right turn signal is on, turn on output pin 4
      set_uja1023_output_bits(1 << 4);
      //puts(" Right turn on!\n");
    }
    else
    {
      clear_uja1023_output_bits(1 << 4);
    }
    
    if (stw_menu_button == 1)
    {
      //menu button is pushed, if it wasn't last time, set the initial timestamp
      if (stw_menu_btn_state_last == 0)
      {
        stw_menu_btn_state_last = 1;
        stw_menu_btn_pressed_ts = ts;  
      }
      else
      {
        uint32_t stw_ts_elapsed = get_ts_elapsed(ts, stw_menu_btn_pressed_ts);
        if (stw_ts_elapsed > STW_MENU_BTN_HOLD_INTERVAL)
        {
          //user held the button, do stuff!
          if (stw_menu_current_output_state == 0 && stw_menu_output_flag == 0)
          {
            stw_menu_output_flag = 1;
            stw_menu_current_output_state = 1;
            //set_uja1023_output_bits(1 << 5);
            //puts("Menu Button held, setting output 5 HIGH\n");
          }
          else if (stw_menu_current_output_state == 1 && stw_menu_output_flag == 0)
          {
            stw_menu_output_flag = 1;
            stw_menu_current_output_state = 0;
            //clear_uja1023_output_bits(1 << 5);
            //puts("Menu Button held, setting output 5 LOW\n");
          }
        } //held
      }
    } //stw menu button pressed
    else if (stw_menu_button == 0)
    {
      stw_menu_output_flag = 0;
      stw_menu_btn_state_last = 0;
    }
    */
    /* revB giraffe GPIO --> */
  }

  // Detect drive rail on (ignition) (start recording)
  if ((addr == 0x348)  && (bus_number == 0))
  {
    //ALSO use this for radar timeout, this message is always on
    uint32_t ts = TIM2->CNT;
    uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_last_radar_signal);
    if ((ts_elapsed > TESLA_RADAR_TIMEOUT) && (tesla_radar_status > 0)) {
      tesla_radar_status = 0;
      puts("Tesla Radar Inactive! (timeout 3) \n");
    } 
  }


  // exit controls on brake press
  // DI_torque2::DI_brakePedal 0x118

  /* revB giraffe GPIO --> */
  /*
  if ((addr == 0x118)  && (bus_number == 0))
  {
    int drive_state = (to_push->RDLR >> 12) & 0x7; //DI_gear : 12|3@1+
    int brake_pressed = (to_push->RDLR & 0x8000) >> 15;
    int tesla_speed_mph = (((((GET_BYTE(to_push, 3) & 0xF) << 8) + GET_BYTE(to_push, 2)) * 0.05) - 25);
    

    
    //if the car goes into reverse, set UJA1023 output pin 5 to high. If Drive, set pin 1 high
    //DI_gear 7 "DI_GEAR_SNA" 4 "DI_GEAR_D" 3 "DI_GEAR_N" 2 "DI_GEAR_R" 1 "DI_GEAR_P" 0 "DI_GEAR_INVALID" ;
    //UJA1023 pin0 is our output to the camera switcher

    if (drive_state == 2)
    {
      //reverse_state = 1;
      set_uja1023_output_bits(1 << 5);
      
      //if we're in reverse, we always want the rear camera up:
      set_uja1023_output_bits(1 << 0); //show rear camera
      //puts(" Got Reverse\n");
      
    }
    else
    {
      //reverse_state = 0;
      clear_uja1023_output_bits(1 << 5);
      
      //if we're in not in reverse and button state is 0, set output low (show front camera)
      if (stw_menu_current_output_state == 0)
      {
        clear_uja1023_output_bits(1 << 0); //show front camera
      }
      
      //if we're not in reverse and button state is 1, set the output high (show the rear camera)
      else
      {
        set_uja1023_output_bits(1 << 0); //show rear camera
      } 
    }
    
    if (drive_state == 4)
    {
      set_uja1023_output_bits(1 << 1);
      //puts(" Got Drive\n");
    }
    else
    {
      clear_uja1023_output_bits(1 << 1);
    }

    if (brake_pressed == 1)
    {
      //disable break cancel by commenting line below
      //controls_allowed = 0;
    
      set_uja1023_output_bits(1 << 2);
      //puts(" Brake on!\n");
      tesla_ts_brakelight_on_last = ts;
    }
    else
    {
      uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_ts_brakelight_on_last);
      if (ts_elapsed > BRAKELIGHT_CLEAR_INTERVAL)
      {
        clear_uja1023_output_bits(1 << 2);
        //puts(" Brakelight off!\n");
      } 
    }
  }
  */
    /* revB giraffe GPIO --> */
  if ((addr == 0x118)  && (bus_number == 0))
  {
    int tesla_speed_mph = (((((GET_BYTE(to_push, 3) & 0xF) << 8) + GET_BYTE(to_push, 2)) * 0.05) - 25);
    
    
    //get vehicle speed in m/2. Tesla gives MPH
    tesla_speed = (tesla_speed_mph*1.609/3.6);
    if (tesla_speed < 0)
    {
      tesla_speed = 0;
    }
    DAS_inDrive_prev = DAS_inDrive;
    if (((to_push->RDLR & 0x7000 ) >> 12) == 4) {
      DAS_inDrive = 1;
    } else {
      DAS_inDrive = 0;
    }
    if ((DAS_inDrive == 0) && (DAS_inDrive_prev == 1)) {
      reset_DAS_data();
    }
  }

  // exit controls on EPAS error
  // EPAS_sysStatus::EPAS_eacStatus 0x370
  //BB this was on bus_number 1 which is wrong, but not tested on 2 yet
  if ((addr == 0x370)  && (bus_number == tesla_epas_can))
  {
    // if EPAS_eacStatus is not 1 or 2, disable control
    eac_status = (GET_BYTE(to_push, 6) >> 5) & 0x7;
    // For human steering override we must not disable controls when eac_status == 0
    // Additional safety: we could only allow eac_status == 0 when we have human steerign allowed
    if ((controls_allowed == 1) && (eac_status != 0) && (eac_status != 1) && (eac_status != 2))
    {
      controls_allowed = 0;
      puts("EPAS error! \n");
    }
  }
  //get latest steering wheel angle
  if ((addr == 0x00E)  && (bus_number == 0))
  {
    int angle_meas_now = (int)(((((GET_BYTE(to_push, 0) & 0x3F) << 8) + GET_BYTE(to_push, 1)) * 0.1) - 819.2);
    //uint32_t ts = TIM2->CNT;
    uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_ts_angle_last);

    // *** angle real time check
    // add 1 to not false trigger the violation and multiply by 25 since the check is done every 250 ms and steer angle is updated at     100Hz
    int rt_delta_angle_up = ((int)((tesla_interpolate(TESLA_LOOKUP_ANGLE_RATE_UP, tesla_speed) * 25. + 1.)));
    int rt_delta_angle_down = ((int)((tesla_interpolate(TESLA_LOOKUP_ANGLE_RATE_DOWN, tesla_speed) * 25. + 1.)));
    int highest_rt_angle = tesla_rt_angle_last + (tesla_rt_angle_last > 0 ? rt_delta_angle_up : rt_delta_angle_down);
    int lowest_rt_angle = tesla_rt_angle_last - (tesla_rt_angle_last > 0 ? rt_delta_angle_down : rt_delta_angle_up);

    if ((ts_elapsed > TESLA_RT_INTERVAL) || (controls_allowed && !tesla_controls_allowed_last))
    {
      tesla_rt_angle_last = angle_meas_now;
      tesla_ts_angle_last = ts;
    }

    // update array of samples
    update_sample(&tesla_angle_meas, angle_meas_now);

    // check for violation;
    if (max_limit_check(angle_meas_now, highest_rt_angle, lowest_rt_angle))
    {
      // We should not be able to STEER under these conditions
      // Other sending is fine (to allow human override)
      steer_allowed = 0;
      puts("WARN: RT Angle - No steer allowed! \n");
    }
    else
    {
      steer_allowed = 1;
    }

    tesla_controls_allowed_last = controls_allowed;
  }
  
    /* <-- revB giraffe GPIO */
  /*
  //BO_ 1001 DAS_bodyControls: 8 XXX
  if ((addr == 0x3e9)  && (bus_number == 0)) {
    int high_beam_decision = (to_push->RDLR >> 10) & 0x3; //DAS_highLowBeamDecision : 10|2@1+
    // highLowBeamDecision:
    //0: Undecided (Car off)
    //1: Off
    //2: On
    //3: Auto High Beam is disabled
    //VAL_ 69 HiBmLvr_Stat 3 "SNA" 2 "HIBM_FLSH_ON_PSD" 1 "HIBM_ON_PSD" 0 "IDLE" ;

    //If the lever is in either high beam position and auto high beam is off or indicates highs should be on
    if ((high_beam_decision == 3 && (high_beam_lever_state == 2 || high_beam_lever_state == 1))
    || (high_beam_decision == 2 && (high_beam_lever_state == 2 || high_beam_lever_state == 1)))
    {
      //high beams are on. Set the output 6 high
      set_uja1023_output_bits(1 << 6);
      //puts("High Beam on!\n");
    } //high beams on!
    else
    {
      //high beams are off. Set the output 6 low
      clear_uja1023_output_bits(1 << 6);
      //puts("High Beam off!\n");
    } //high beams off
  } //DAS_bodyControls
  
  //BO_ 872 DI_state: 8 DI
  if ((addr == 0x368)  && (bus_number == 0)) {
    int regen_brake_light = (to_push->RDLR >> 8) & 0x1; //DI_regenLight : 8|1@1+
    //if the car's brake lights are on, set pin 2 to high
    if (regen_brake_light == 1)
    {
      set_uja1023_output_bits(1 << 2);
      //puts(" Regen Brake Light on!\n");
      tesla_ts_brakelight_on_last = ts;
    }
    else
    {
      uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_ts_brakelight_on_last);
      if (ts_elapsed > BRAKELIGHT_CLEAR_INTERVAL)
      {
        clear_uja1023_output_bits(1 << 2);
        //puts(" Brakelight off!\n");
      }
    }
  }*/
  /* revB giraffe GPIO --> */
  return valid;
}

// all commands: gas/regen, friction brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int tesla_tx_hook(CAN_FIFOMailBox_TypeDef *to_send)
{

  uint32_t addr;
  int desired_angle;

  addr = to_send->RIR >> 21;

  //capture message for radarVIN and settings
  if (addr == 0x560) {
    int id = (to_send->RDLR & 0xFF);
    int radarVin_b1 = ((to_send->RDLR >> 8) & 0xFF);
    int radarVin_b2 = ((to_send->RDLR >> 16) & 0xFF);
    int radarVin_b3 = ((to_send->RDLR >> 24) & 0xFF);
    int radarVin_b4 = (to_send->RDHR & 0xFF);
    int radarVin_b5 = ((to_send->RDHR >> 8) & 0xFF);
    int radarVin_b6 = ((to_send->RDHR >> 16) & 0xFF);
    int radarVin_b7 = ((to_send->RDHR >> 24) & 0xFF);
    if (id == 0) {
      tesla_radar_should_send = (radarVin_b2 & 0x01);
      radarPosition =  ((radarVin_b2 >> 1) & 0x03);
      radarEpasType = ((radarVin_b2 >> 3) & 0x07);
      tesla_radar_trigger_message_id = (radarVin_b3 << 8) + radarVin_b4;
      tesla_radar_can = radarVin_b1;
      radar_VIN[0] = radarVin_b5;
      radar_VIN[1] = radarVin_b6;
      radar_VIN[2] = radarVin_b7;
      tesla_radar_vin_complete = tesla_radar_vin_complete | 1;
    }
    if (id == 1) {
      radar_VIN[3] = radarVin_b1;
      radar_VIN[4] = radarVin_b2;
      radar_VIN[5] = radarVin_b3;
      radar_VIN[6] = radarVin_b4;
      radar_VIN[7] = radarVin_b5;
      radar_VIN[8] = radarVin_b6;
      radar_VIN[9] = radarVin_b7;
      tesla_radar_vin_complete = tesla_radar_vin_complete | 2;
    }
    if (id == 2) {
      radar_VIN[10] = radarVin_b1;
      radar_VIN[11] = radarVin_b2;
      radar_VIN[12] = radarVin_b3;
      radar_VIN[13] = radarVin_b4;
      radar_VIN[14] = radarVin_b5;
      radar_VIN[15] = radarVin_b6;
      radar_VIN[16] = radarVin_b7;
      tesla_radar_vin_complete = tesla_radar_vin_complete | 4;
    }
    return false;
  }
  //capture message for fake DAS street sign object
  if (addr == 0x556) {
    streetSignObject_b0 = (to_send->RDLR & 0xFF);
    streetSignObject_b1 = ((to_send->RDLR >> 8) & 0xFF);
    streetSignObject_b2 = ((to_send->RDLR >> 16) & 0xFF);
    streetSignObject_b3 = ((to_send->RDLR >> 24) & 0xFF);
    streetSignObject_active = 0;
    int signType = ((streetSignObject_b0 >> 6) & 0x03) + ((streetSignObject_b1 << 2) & 0xFF);
    if (signType < 0xFF) {
       streetSignObject_active = 1;
    }
    return false;
  }

  //capture message for DAS objects
  if (addr == 0x559) {
    int lane =  (to_send->RDLR & 0x03);
    if (lane == 0) {
      DAS_LEAD_OBJECT_MLB = to_send->RDLR;
      DAS_LEAD_OBJECT_MHB = to_send->RDHR;
      if(((to_send->RDLR >> 8) & 0xFF) == 0)  {
        DAS_LEAD_OBJECT_MLB = 0xFFFFFF00;
        DAS_LEAD_OBJECT_MHB = 0x03FFFF83;
      }
    }
    if (lane == 1) {
      DAS_LEFT_OBJECT_MLB = to_send->RDLR;
      DAS_LEFT_OBJECT_MHB = to_send->RDHR;
      if(((to_send->RDLR >> 8) & 0xFF) == 0)  {
        DAS_LEFT_OBJECT_MLB = 0xFFFFFF01;
        DAS_LEFT_OBJECT_MHB = 0x03FFFF83;
      }
    }
    if (lane == 2) {
      DAS_RIGHT_OBJECT_MLB = to_send->RDLR;
      DAS_RIGHT_OBJECT_MHB = to_send->RDHR;
      if(((to_send->RDLR >> 8) & 0xFF) == 0)  {
        DAS_RIGHT_OBJECT_MLB = 0xFFFFFF02;
        DAS_RIGHT_OBJECT_MHB = 0x03FFFF83;
      }
    }
    return false;
  }

  //capture message for fake DAS lane and object info
  if (addr == 0x557) {
    tLeadDx = (to_send->RDLR & 0xFF);
    tLeadDy = ((to_send->RDLR >> 8) & 0xFF);
    rLine_qual = ((to_send->RDLR >> 16) & 0x03);
    lLine_qual = ((to_send->RDLR >> 18) & 0x03);
    if (rLine_qual > 1) {
      rLine = 1;
    } else {
      rLine = 0;
    }
    if (lLine_qual > 1) {
      lLine = 1;
    } else {
      lLine = 0;
    }
    lWidth = ((to_send->RDLR >> 20) & 0x0F);
    curvC0 = ((to_send->RDLR >> 24) & 0xFF);
    curvC1 = (to_send->RDHR & 0xFF);
    curvC2 = ((to_send->RDHR >> 8) & 0xFF);
    curvC3 = ((to_send->RDHR >> 16) & 0xFF);
    laneRange = ((to_send->RDHR >> 24) & 0x3F) * 4;
    tLeadClass = ((to_send->RDHR >> 30) & 0x03);
    tLeadClass ++;
    //we used 4 - bicycle for 5-pedestrian
    if (tLeadClass == 4) {
      tLeadClass = 5;
    }
    return false;
  }


  //capture message for fake DAS warning
  if (addr == 0x554) {
    int b0 = (to_send->RDLR & 0xFF); 
    int b1 = ((to_send->RDLR >> 8) & 0xFF);
    int b2 = ((to_send->RDLR >> 16) & 0xFF);
    DAS_211_accNoSeatBelt = ((b0 >> 4) & 0x01);
    DAS_canErrors = ((b0 >> 3) & 0x01);
    DAS_202_noisyEnvironment = ((b0 >> 2) & 0x01);
    DAS_doorOpen = ((b0 >> 1) & 0x01);
    DAS_notInDrive = ((b0 >> 0) & 0x01);
    enable_das_emulation = ((b0 >> 5) & 0x01);
    enable_radar_emulation = ((b0 >> 6) & 0x01);
    stopLightWarning = ((b0 >> 7) & 0x01);
    stopSignWarning = ((b1 >> 0) & 0x01);
    DAS_222_accCameraBlind = ((b1 >> 1) & 0x01); //we will not override the one set in 0x553
    DAS_219_lcTempUnavailableSpeed = ((b1 >> 2) & 0x01);
    DAS_220_lcTempUnavailableRoad = ((b1 >> 3) & 0x01);
    DAS_221_lcAborting = ((b1 >> 4) & 0x01);
    DAS_207_lkasUnavailable = ((b1 >> 5) & 0x01);
    DAS_208_rackDetected = ((b1 >> 6) & 0x01);
    DAS_025_steeringOverride = ((b1 >> 7) & 0x01);
    DAS_ldwStatus = (b2 & 0x07);
    //FLAG NOT USED = ((b2 >> 3) & 0x01);
    DAS_noEpasHarness = ((b2 >> 4) & 0x01);
    DAS_usesApillarHarness = ((b2 >> 5) & 0x01);
    if (DAS_noEpasHarness == 1) {
      tesla_epas_can = 0;
    } else {
      tesla_epas_can = 2;
    }
    return false;
  }

  //capture message for AHB and parse 
  if (addr == 0x65A) {
    int b0 = (to_send->RDLR & 0xFF);
    int b1 = ((to_send->RDLR >> 8) & 0xFF);
    int b2 = ((to_send->RDLR >> 16) & 0xFF);
    DAS_high_low_beam_request = b0;
    DAS_high_low_beam_reason = b1;
    DAS_ahb_is_enabled = b2 & 0x01;
    DAS_fleet_speed_state = (b2 >> 1) & 0x03;
    //intercept and do not forward
    return false;
  }

  //capture message for fake DAS and parse
  if (addr == 0x659) { //0x553) {
    int b0 = (to_send->RDLR & 0xFF);
    int b1 = ((to_send->RDLR >> 8) & 0xFF);
    int b2 = ((to_send->RDLR >> 16) & 0xFF);
    int b3 = ((to_send->RDLR >> 24) & 0xFF);
    int b4 = (to_send->RDHR & 0xFF);
    int b5 = ((to_send->RDHR >> 8) & 0xFF);
    int b6 = ((to_send->RDHR >> 16) & 0xFF);
    int b7 = ((to_send->RDHR >> 24) & 0xFF);

    DAS_acc_speed_kph = b1;
    DAS_acc_speed_limit = b4;
    DAS_longC_enabled = ((b0 & 0x80) >> 7);
    DAS_gas_to_resume = ((b0 & 0x40) >> 6);
    DAS_206_apUnavailable = ((b0 & 0x20) >> 5);
    DAS_collision_warning = ((b0 & 0x10) >> 4);
    DAS_op_status = (b0 & 0x0F);
    DAS_turn_signal_request = ((b2 & 0xC0) >> 6);
    DAS_forward_collision_warning = ((b2 & 0x10) >> 4);
    DAS_units_included = ((b2 & 0x20) >> 5);
    if (((b2 >> 3) & 0x01) == 0) {
      DAS_plain_cc_enabled = 1;
    } else {
      DAS_plain_cc_enabled = 0;
    }
    DAS_hands_on_state = (b2 & 0x07);
    DAS_cc_state = ((b3 & 0xC0)>>6);
    DAS_usingPedal = ((b3 & 0x20) >> 5);
    DAS_alca_state = (b3 & 0x1F);
    DAS_speed_limit_kph = (b5 & 0x1F);
    DAS_emergency_brake_request = ((b5 & 0x20)  >> 5);
    time_last_DAS_data = current_car_time;
    DAS_present = 1;
    DAS_steeringAngle = ((b7 << 8) + b6) & 0x7FFF;
    DAS_steeringEnabled = (b7 >> 7);

    desired_angle = DAS_steeringAngle * 0.1 - 1638.35;
    //bool violation = 0;

    if (DAS_steeringEnabled == 0) {
      //steering is not enabled, do not check angles and do send
      tesla_desired_angle_last = desired_angle;
    } else
    if (controls_allowed)
    {
      // add 1 to not false trigger the violation
      int delta_angle_up = (int)(tesla_interpolate(TESLA_LOOKUP_ANGLE_RATE_UP, tesla_speed) * 25. + 1.);
      int delta_angle_down = (int)(tesla_interpolate(TESLA_LOOKUP_ANGLE_RATE_DOWN, tesla_speed) * 25. + 1.);
      int highest_desired_angle = tesla_desired_angle_last + (tesla_desired_angle_last > 0 ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = tesla_desired_angle_last - (tesla_desired_angle_last > 0 ? delta_angle_down : delta_angle_up);
      int TESLA_MAX_ANGLE = (int)(tesla_interpolate(TESLA_LOOKUP_MAX_ANGLE, tesla_speed) + 1.);

      if (max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle))
      {
        //violation = 1;
        controls_allowed = 0;
        puts("Angle limit - delta! \n");
      }
      if (max_limit_check(desired_angle, TESLA_MAX_ANGLE, -TESLA_MAX_ANGLE))
      {
        //violation = 1;
        controls_allowed = 0;
        puts("Angle limit - max! \n");
      }
    }
    tesla_desired_angle_last = desired_angle;

    return true; //we have to send this to the spamIC 
  }

  return true;
}

static void tesla_init(int16_t param)
{
  UNUSED(param);
  if (DAS_noEpasHarness == 0) {
    gmlan_switch_init(1); //init the gmlan switch with 1s timeout enabled
  }
  //uja1023_init();
}


static void tesla_fwd_to_radar_as_is(uint8_t bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  if ((enable_radar_emulation == 0) || (tesla_radar_vin_complete !=7) || (tesla_radar_should_send==0) ) {
    return;
  }
  CAN_FIFOMailBox_TypeDef to_send;
  to_send.RIR = to_fwd->RIR | 1; // TXRQ
  to_send.RDTR = to_fwd->RDTR;
  to_send.RDLR = to_fwd->RDLR;
  to_send.RDHR = to_fwd->RDHR;
  can_send(&to_send, bus_num, true);
}


static uint32_t radar_VIN_char(int pos, int shift) {
  return (((int)radar_VIN[pos]) << (shift * 8));
}


static void tesla_fwd_to_radar_modded(uint8_t bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  if ((enable_radar_emulation == 0) || (tesla_radar_vin_complete !=7) || (tesla_radar_should_send==0) ) {
    return;
  }
  int32_t addr = to_fwd->RIR >> 21;
  CAN_FIFOMailBox_TypeDef to_send;
  to_send.RIR = to_fwd->RIR | 1; // TXRQ
  to_send.RDTR = to_fwd->RDTR;
  to_send.RDLR = to_fwd->RDLR;
  to_send.RDHR = to_fwd->RDHR;
  uint32_t addr_mask = 0x001FFFFF;
  //now modd
  if (addr == 0x405 )
  {
    to_send.RIR = (0x2B9 << 21) + (addr_mask & (to_fwd->RIR | 1));
    if (((to_send.RDLR & 0x10) == 0x10) && (sizeof(radar_VIN) >= 4))
    {
      int rec = to_send.RDLR &  0xFF;
      if (rec == 0x10) {
        to_send.RDLR = 0x00000000 | rec;
        to_send.RDHR = radar_VIN_char(0,1) | radar_VIN_char(1,2) | radar_VIN_char(2,3);
      }
      if (rec == 0x11) {
        to_send.RDLR = rec | radar_VIN_char(3,1) | radar_VIN_char(4,2) | radar_VIN_char(5,3);
        to_send.RDHR = radar_VIN_char(6,0) | radar_VIN_char(7,1) | radar_VIN_char(8,2) | radar_VIN_char(9,3);
      }
      if (rec == 0x12) {
        to_send.RDLR = rec | radar_VIN_char(10,1) | radar_VIN_char(11,2) | radar_VIN_char(12,3);
        to_send.RDHR = radar_VIN_char(13,0) | radar_VIN_char(14,1) | radar_VIN_char(15,2) | radar_VIN_char(16,3);
      }
    }
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x398 )
  {
    //change frontradarHW = 1 and dashw = 1
    //SG_ GTW_dasHw : 7|2@0+ (1,0) [0|0] ""  NEO
    //SG_ GTW_parkAssistInstalled : 11|2@0+ (1,0) [0|0] ""  NEO
    to_send.RDHR = to_send.RDHR | 0x100; //TODO if this is Park Assist, it should be RDLR not RDHR
    //resend on CAN 0 first
    to_send.RIR = (to_fwd->RIR | 1);
    //can_send(&to_send,0, true);
     
    
    to_send.RDLR = to_send.RDLR & 0xFFFFF33F;
    to_send.RDLR = to_send.RDLR | 0x440;
    // change the autopilot to 1
    to_send.RDHR = to_fwd->RDHR & 0xCFFF0F0F;
    to_send.RDHR = to_send.RDHR | 0x10000000 | (radarPosition << 4) | (radarEpasType << 12);
    
    if ((sizeof(radar_VIN) >= 4) && (((int)(radar_VIN[7]) == 0x32) || ((int)(radar_VIN[7]) == 0x34))) {
        //also change to AWD if needed (most likely) if manual VIN and if position 8 of VIN is a 2 (dual motor)
        to_send.RDLR = to_send.RDLR | 0x08;
    }
    //now change address and send to radar
    to_send.RIR = (0x2A9 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x00E )
  {
    to_send.RIR = (0x199 << 21) + (addr_mask & (to_fwd->RIR | 1));
    //check if angular speed sends SNA (0x3FFF)
    if (((to_send.RDLR >> 16) & 0xFF3F) == 0xFF3F) {
      //if yes replace 0x3FFFF with 0x2000 which is 0 angular change
      to_send.RDLR = (to_send.RDLR & 0x00C0FFFF) | (0x0020 << 16);
      //if this is the case, most likely we need to change the model too
      //so remove CRC and StW_AnglHP_Sens_Id (1st octet of RDHR)
      to_send.RDHR = to_send.RDHR & 0x00FFFFF0;
      //force StW_AnglHP_Sens_Id to DELPHI (0x04 1st octet of RDHR)
      to_send.RDHR = to_send.RDHR | 0x00000004;
      //compute new CRC
      int crc = add_tesla_crc(to_send.RDLR, to_send.RDHR,7);
      //Add new CRC
      to_send.RDHR = to_send.RDHR | (crc << 24);
    }
    can_send(&to_send, bus_num, true);
    return;
  }
  if (addr == 0x20A )
  {
    to_send.RIR = (0x159 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x115 )
  {
    
    int counter = ((to_fwd->RDHR & 0xF0) >> 4 ) & 0x0F;

    to_send.RIR = (0x129 << 21) + (addr_mask & (to_fwd->RIR | 1));
    int cksm = (0x16 + (counter << 4)) & 0xFF;
    can_send(&to_send, bus_num, true);

    //we don't get 0x148 DI_espControl so send as 0x1A9 on CAN1 and also as 0x148 on CAN0
    to_send.RDTR = (to_fwd->RDTR & 0xFFFFFFF0) | 0x05;
    to_send.RIR = (0x148 << 21) + (addr_mask & (to_fwd->RIR | 1));
    to_send.RDLR = 0x000C0000 | (counter << 28);
    cksm = (0x38 + 0x0C + (counter << 4)) & 0xFF;
    to_send.RDHR = cksm;
    //can_send(&to_send, 0, true);

    to_send.RIR = (0x1A9 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

    return;
  }

  if (addr == 0x145) 
  {
    to_send.RIR = (0x149 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

    return;
  }
  
  /*if (addr == 0x175)
  {
    to_send.RIR = (0x169 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);
    return;
  }*/

  if (addr == 0x118 )
  {
    to_send.RIR = (0x119 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

     //we don't get 0x175 ESP_wheelSpeeds so send as 0x169 on CAN1 and also as 0x175 on CAN0
    int counter = to_fwd->RDHR  & 0x0F;
    to_send.RIR = (0x169 << 21) + (addr_mask & (to_fwd->RIR | 1));
    to_send.RDTR = (to_fwd->RDTR & 0xFFFFFFF0) | 0x08;
    int32_t speed_kph = (((0xFFF0000 & to_send.RDLR) >> 16) * 0.05 -25) * 1.609;
    if (speed_kph < 0) {
      speed_kph = 0;
    }
    // is AHB is enabled, use low apeed to spread radar angle
    if ((speed_kph > 2 ) && (DAS_ahb_is_enabled == 1)) {
    //  speed_kph = 2;
    }
    if (((0xFFF0000 & to_send.RDLR) >> 16) == 0xFFF) {
      speed_kph = 0x1FFF; //0xFFF is signal not available for DI_Torque2 speed 0x118; should be SNA or 0x1FFF for 0x169
    } else {
      speed_kph = (int)(speed_kph/0.04) & 0x1FFF;
    }
    to_send.RDLR = (speed_kph | (speed_kph << 13) | (speed_kph << 26)) & 0xFFFFFFFF;
    to_send.RDHR = ((speed_kph  >> 6) | (speed_kph << 7) | (counter << 20)) & 0x00FFFFFF;
    int cksm = 0x76;
    cksm = (cksm + (to_send.RDLR & 0xFF) + ((to_send.RDLR >> 8) & 0xFF) + ((to_send.RDLR >> 16) & 0xFF) + ((to_send.RDLR >> 24) & 0xFF)) & 0xFF;
    cksm = (cksm + (to_send.RDHR & 0xFF) + ((to_send.RDHR >> 8) & 0xFF) + ((to_send.RDHR >> 16) & 0xFF) + ((to_send.RDHR >> 24) & 0xFF)) & 0xFF;
    to_send.RDHR = to_send.RDHR | (cksm << 24);
    can_send(&to_send, bus_num, true);

    //to_send.RIR = (0x175 << 21) + (addr_mask & (to_fwd->RIR | 1));
    //can_send(&to_send, 0, true);
    
    return;
  }
  if (addr == 0x108 )
  {
    to_send.RIR = (0x109 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x308 )
  {
    to_send.RIR = (0x209 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x45 )
  {
    to_send.RIR = (0x219 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x148 )
  {
    to_send.RIR = (0x1A9 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num,true);

    return;
  }
  if (addr == 0x30A)
  {
    to_send.RIR = (0x2D9 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);

    return;
  }
  
}

static int tesla_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd)
{

  int32_t addr = to_fwd->RIR >> 21;
  int ret_val = -1;

  //first let's deal with the messages we need to send to radar
  if ((bus_num == 0) || ((bus_num == 2 ) && (DAS_usesApillarHarness == 1)))
  {
    
    //compute return value
    if (bus_num == 0) {
      if (DAS_noEpasHarness == 0) {
        ret_val=2;
      } else {
        ret_val=-1;
      }
    } else if (bus_num == 2) {
      ret_val=0;
    }

    //check all messages we need to also send to radar, moddified, after we receive 0x631 from radar
    //148 does not exist, we use 115 at the same frequency to trigger and pass static vals
    //175 does not exist, we use 118 at the same frequency to trigger and pass vehicle speed
    if ((tesla_radar_status > 0 ) && (enable_radar_emulation == 1) && ((addr == 0x20A ) || (addr == 0x118 ) || (addr == 0x108 ) ||  
    (addr == 0x115 ) ||  (addr == 0x148 ) || (addr == 0x145)))
    {
      tesla_fwd_to_radar_modded(tesla_radar_can, to_fwd);
      return ret_val;
    }

    //check all messages we need to also send to radar, moddified, all the time
    if  (((addr == 0xE ) || (addr == 0x308 ) || (addr == 0x45 ) || (addr == 0x398 ) ||
    (addr == 0x405 ) ||  (addr == 0x30A)) && (enable_radar_emulation == 1))  {
      tesla_fwd_to_radar_modded(tesla_radar_can, to_fwd);
      return ret_val;
    }

    //forward to radar unmodded the UDS messages 0x641
    if  (addr == 0x641 ) {
      tesla_fwd_to_radar_as_is(tesla_radar_can, to_fwd);
      return ret_val;
    }
    
  }   
  //now let's deal with CAN0 alone
  if (bus_num == 0) {

    if (DAS_noEpasHarness == 0) {
      ret_val=2;
    } else {
      ret_val=-1;
    }
    
    // change inhibit of GTW_epasControl and enabled haptic for LDW
    if (addr == 0x101)
    {
      to_fwd->RDLR = GET_BYTES_04(to_fwd) | 0x4000 | 0x1000; 
      // 0x4000: WITH_ANGLE, 0xC000: WITH_BOTH (angle and torque)
      // 0x1000: enabled LDW for haptic
      int checksum = (GET_BYTE(to_fwd, 1) + GET_BYTE(to_fwd, 0) + 2) & 0xFF;
      to_fwd->RDLR = GET_BYTES_04(to_fwd) & 0xFFFF;
      to_fwd->RDLR = GET_BYTES_04(to_fwd) + (checksum << 16);
      return ret_val;
    }

    if (addr == 0x214) {
      //inhibit ibooster epas kill signal
      return -1;
    }

    //forward everything else to CAN 2 unless claiming no harness
    return ret_val;
  }

  //now let's deal with CAN1 - Radar
  if (bus_num == tesla_radar_can) {
    //send radar 0x531 and 0x651 from Radar CAN to CAN0
    if ((addr == 0x531) || (addr == 0x651)){ 
      return 0;
    }

    //block everything else from radar
    return -1;
  }

  //now let's deal with CAN2 
  if ((bus_num == tesla_epas_can) && (bus_num > 0))
  {

    // remove Pedal in forwards
    if ((addr == 0x551) || (addr == 0x552)) {
      return -1;
    }

    //forward everything else to CAN 0 unless claiming no harness
    return 0;
  }
  return -1;
}

const safety_hooks tesla_hooks = {
  .init = tesla_init,
  .rx = tesla_rx_hook,
  .tx = tesla_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = tesla_fwd_hook,
  .addr_check = tesla_rx_checks,
  .addr_check_len = sizeof(tesla_rx_checks) / sizeof(tesla_rx_checks[0]),
};
