void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);

const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_UP = {
    {2., 7., 17.},
    {8., .8, .25}};

const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_DOWN = {
    {2., 7., 17.},
    {5., 3.5, .8}};

const int TESLA_DEG_TO_CAN = 10;

static uint8_t len_to_dlc(uint8_t len) {
  if (len <= 8) {
    return len;
  }
  if (len <= 24) {
    return 8 + ((len - 8) / 4) + ((len % 4) ? 1 : 0);
  } else {
    return 11 + (len / 16) + ((len % 16) ? 1 : 0);
  }
}

const uint32_t TIME_TO_ENGAGE = 500000; //0.5s 1swait for AP status @ 2Hz
const uint32_t TIME_TO_HIDE_ERRORS = 4000000; //4s to hide potential fake DAS errors after disengage

uint32_t time_cruise_engaged = 0;
uint32_t time_op_disengaged = 0;
const float TESLA_MAX_ACCEL = 2.0;  // m/s^2
const float TESLA_MIN_ACCEL = -3.5; // m/s^2

//for safetyParam parsing
const uint16_t FLAG_TESLA_POWERTRAIN = 1;
const uint16_t FLAG_TESLA_LONG_CONTROL = 2;
const uint16_t FLAG_TESLA_RADAR_BEHIND_NOSECONE = 4;
const uint16_t FLAG_TESLA_HAS_IC_INTEGRATION = 8;
const uint16_t FLAG_TESLA_HAS_AP = 16;
const uint16_t FLAG_TESLA_NEED_RADAR_EMULATION = 32;
const uint16_t FLAG_TESLA_ENABLE_HAO = 64;
const uint16_t FLAG_TESLA_HAS_IBOOSTER = 128;


bool has_ap_hardware = false;
bool has_ibooster = false;
bool has_ibooster_ecu = false;
bool has_acc = false;
bool has_hud_integration = false;
bool has_body_controls = false;
bool do_radar_emulation = false;
bool enable_hao = false;
bool tesla_longitudinal = false;
bool tesla_powertrain = false;  // Are we the second panda intercepting the powertrain bus?

bool bosch_radar_vin_learn = false;

int last_acc_status = -1;
int prev_controls_allowed = 0;

//pedal pressed (with Pedal)
int pedalPressed = 0;
int pedalCan = -1;
int pedalEnabled = 0;

//use for preAP IC integration
int IC_send_counter = 0;
int DAS_bodyControls_idx = 0;
int DAS_control_idx = 0;
int DAS_status_idx = 0;
int DAS_status2_idx = 0;
int DAS_lanes_idx = 0;
int DAS_steeringControl_idx = 0;
uint32_t DAS_lastStalkL = 0;
uint32_t DAS_lastStalkH = 0;
int EPB_epasControl_idx = 0;
int time_at_last_stalk_pull = -1;
int current_car_time = -1;


//use for Bosch radar
int tesla_radar_status = 0; //0-not present, 1-initializing, 2-active
uint32_t tesla_last_radar_signal = 0;
const uint32_t TESLA_RADAR_TIMEOUT = 10000000; // 10s second between real time checks
uint8_t tesla_radar_can = 1;
uint8_t tesla_epas_can = 0;
int tesla_radar_trigger_message_id = 0; //not used by tesla, to showcase for other cars
int radarPosition = 0; //0 nosecone, 1 facelift
int radarEpasType = 0; //0/1 bosch, 2-4 mando

//TESLA WITH AUTOPILOT DEFS
const CanMsg TESLA_AP_TX_MSGS[] = {
    {0x488, 0, 4},  // DAS_steeringControl - Lat Control
    {0x2B9, 0, 8},  // DAS_control - Long Control
    {0x209, 0, 8},  // DAS_longControl - Long Control
    {0x45,  0, 8},  // STW_ACTN_RQ - ACC Control
    {0x45,  2, 8},  // STW_ACTN_RQ - ACC Control
    {0x399, 0, 8},  // DAS_status - HUD
    {0x389, 0, 8},  // DAS_status2 - HUD
    {0x239, 0, 8},  // DAS_lanes - HUD
    {0x309, 0, 8},  // DAS_object - HUD
    {0x3A9, 0, 8},  // DAS_telemetry - HUD
    {0x3E9, 0, 8},  // DAS_bodyControls - Car Integration for turn signal on ALCA
    {0x329, 0, 8},  // DAS_warningMatrix0
    {0x369, 0, 8},  // DAS_warningMatrix1
    {0x349, 0, 8},  // DAS_warningMatrix3
  };
#define TESLA_AP_TX_LEN (sizeof(TESLA_AP_TX_MSGS) / sizeof(TESLA_AP_TX_MSGS[0]))

const CanMsg TESLA_PT_TX_MSGS[] = {
  {0x2bf, 0, 8},  // DAS_control
};
#define TESLA_PT_TX_LEN (sizeof(TESLA_PT_TX_MSGS) / sizeof(TESLA_PT_TX_MSGS[0]))

AddrCheckStruct  TESLA_AP_RX_CHECKS[] = {
    {.msg = {{0x370, 0, 8, .expected_timestep = 40000U}}},   // EPAS_sysStatus (25Hz)
    {.msg = {{0x108, 0, 8, .expected_timestep = 10000U}}},   // DI_torque1 (100Hz)
    {.msg = {{0x118, 0, 6, .expected_timestep = 10000U}}},   // DI_torque2 (100Hz)
    {.msg = {{0x155, 0, 8, .expected_timestep = 20000U}}},   // ESP_B (50Hz)
    {.msg = {{0x20a, 0, 8, .expected_timestep = 20000U}}},   // BrakeMessage (50Hz)
    {.msg = {{0x368, 0, 8, .expected_timestep = 100000U}}},  // DI_state (10Hz)
    {.msg = {{0x318, 0, 8, .expected_timestep = 100000U}}},  // GTW_carState (10Hz)
    {.msg = {{0x399, 2, 8, .expected_timestep = 500000U}}},  // AutopilotStatus (2Hz)
  };
#define TESLA_AP_RX_LEN (sizeof(TESLA_AP_RX_CHECKS) / sizeof(TESLA_AP_RX_CHECKS[0]))
addr_checks tesla_rx_checks = {TESLA_AP_RX_CHECKS, TESLA_AP_RX_LEN};

AddrCheckStruct TESLA_PT_RX_CHECKS[] = {
  {.msg = {{0x106, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},   // DI_torque1 (100Hz)
  {.msg = {{0x116, 0, 6, .expected_timestep = 10000U}, { 0 }, { 0 }}},   // DI_torque2 (100Hz)
  {.msg = {{0x1f8, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},   // BrakeMessage (50Hz)
  {.msg = {{0x256, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},  // DI_state (10Hz)
};
#define TESLA_PT_RX_LEN (sizeof(TESLA_PT_RX_CHECKS) / sizeof(TESLA_PT_RX_CHECKS[0]))
addr_checks tesla_pt_rx_checks = {TESLA_PT_RX_CHECKS, TESLA_PT_RX_LEN};

CanMsgFwd  TESLA_AP_FWD_MODDED[] = {
    //used for control
    {.msg = {0x488,2,4},.fwd_to_bus=0,.expected_timestep = 50000U,.counter_mask_H=0x00000000,.counter_mask_L=0x000F0000}, // DAS_steeringControl - Lat Control - 20Hz
    {.msg = {0x2B9,2,8},.fwd_to_bus=0,.expected_timestep = 25000U,.counter_mask_H=0x00E00000,.counter_mask_L=0x00000000}, // DAS_control - Long Control - 40Hz
    {.msg = {0x209,2,8},.fwd_to_bus=0,.expected_timestep = 25000U,.counter_mask_H=0x00E00000,.counter_mask_L=0x00000000}, // DAS_longControl - Long Control - 40Hz
    //for DAS_bodyControls Mask all but checksum, turnSignalRequest, turnSignalReason and hazardRequest
    {.msg = {0x3E9,2,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00FFFFF,.counter_mask_L=0xFFF0FCF3}, // DAS_bodyControls - Control Body - 2Hz - 
    //used for IC integration
    {.msg = {0x399,2,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00F8031F,.counter_mask_L=0xFF3FFFF0}, // DAS_status - Status - 2Hz
    {.msg = {0x389,2,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00F0FF3F,.counter_mask_L=0xFFFF3FFF}, // DAS_status2 - Status - 2Hz
    {.msg = {0x329,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix0 - Status - 1Hz - nocounter/nochecksum
    {.msg = {0x369,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix1 - Status - 1Hz - nocounter/nochecksum
    {.msg = {0x349,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix3 - Status - 1Hz - nocounter/nochecksum
    {.msg = {0x3A9,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_telemetry - Lane Type - 10Hz - nocounter/nochecksum
    {.msg = {0x3B1,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_telemetryFurniture - Lane Type - 25Hz - nocounter/nochecksum
    {.msg = {0x309,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_object - Lead Car - 30Hz - nocounter/nochecksum
    {.msg = {0x239,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0xF0000000,.counter_mask_L=0x00000000}, // DAS_lanes - Path/Lanes - 10Hz - nochecksum
  };

//TESLA WITHOUT AUTOPILOT DEFS
const CanMsg TESLA_PREAP_TX_MSGS[] = {
    {0x488, 0, 4},  // DAS_steeringControl - Lat Control
    {0x2B9, 0, 8},  // DAS_control - Long Control
    {0x209, 0, 8},  // DAS_longControl - Long Control
    {0x45,  0, 8},  // STW_ACTN_RQ - ACC Control
    {0x399, 0, 8},  // DAS_status - HUD
    {0x389, 0, 8},  // DAS_status2 - HUD
    {0x239, 0, 8},  // DAS_lanes - HUD
    {0x309, 0, 8},  // DAS_object - HUD
    {0x3A9, 0, 8},  // DAS_telemetry - HUD
    {0x3E9, 0, 8},  // DAS_bodyControls - Car Integration for turn signal on ALCA
    {0x329, 0, 8},  // DAS_warningMatrix0
    {0x369, 0, 8},  // DAS_warningMatrix1
    {0x349, 0, 8},  // DAS_warningMatrix3
    {0x659, 0, 8},  // DAS_uds used for IC into TB 
    {0x214, 0, 3},  // EPB_epasControl 
    //used for radar integration
    {0x109, 1, 8},  //DI_torque1
    {0x119, 1, 6},  //DI_torque2
    {0x129, 1, 6},  //ESP_115h
    {0x149, 1, 8},  //ESP_145h
    {0x159, 1, 8},  //ESP_C
    {0x169, 1, 8},  //ESP_wheelSpeed
    {0x199, 1, 8},  //STW_ANGLHP_STAT
    {0x1A9, 1, 5},  //DI_espControl
    {0x209, 1, 8},  //GTW_odo
    {0x219, 1, 8},  //STW_ACTN_RQ
    {0x2A9, 1, 8},  //GTW_carConfig
    {0x2B9, 1, 8},  //VIP_405HS
    {0x2D9, 1, 8},  //BC_status
    {0x560, 1, 8},  //radar VIN fake message
    {0x641, 1, 8},  //UDS message to radar CAN0 0x671 -> 0x641 on CAN1
    {0x681, 0, 8},  //UDS answer from CAN1 0x651 -> 0x681 on CAN0
    //pedal
    {0x551, 0, 6}, //GAS_INTERCEPTOR command can0
    {0x551, 2, 6}, //GAS_INTERCEPTOR command can2
    {0x200, 0, 6}, //old code for pedal
    {0x200, 2, 6}, //old code for pedal can2
    //brake wipe request
    {0x208, 0, 4}, //GTW_ESP1 spammed 
    //iBooster command
    {0x553, 0, 6}, //Brake command for iBooster
    //ibooster vacuum switch
    {0x555, 0, 6}, //iBooster vacuum switch fake code
  };
#define TESLA_PREAP_TX_LEN (sizeof(TESLA_PREAP_TX_MSGS) / sizeof(TESLA_PREAP_TX_MSGS[0]))

AddrCheckStruct  TESLA_PREAP_RX_CHECKS[] = {
    {.msg = {{0x370, 0, 8, .expected_timestep = 40000U}}},   // EPAS_sysStatus (25Hz)
    {.msg = {{0x108, 0, 8, .expected_timestep = 10000U}}},   // DI_torque1 (100Hz)
    {.msg = {{0x118, 0, 6, .expected_timestep = 10000U}}},   // DI_torque2 (100Hz)
    {.msg = {{0x155, 0, 8, .expected_timestep = 20000U}}},   // ESP_B (50Hz)
    {.msg = {{0x20a, 0, 8, .expected_timestep = 20000U}}},   // BrakeMessage (50Hz)
    {.msg = {{0x368, 0, 8, .expected_timestep = 100000U}}},  // DI_state (10Hz)
    {.msg = {{0x318, 0, 8, .expected_timestep = 100000U}}},  // GTW_carState (10Hz)
    {.msg = {{0x45, 0, 8, .expected_timestep = 100000U}}},  // STW_ACTN_RQ (10Hz)
  };
#define TESLA_PREAP_RX_LEN (sizeof(TESLA_PREAP_RX_CHECKS) / sizeof(TESLA_PREAP_RX_CHECKS[0]))
addr_checks tesla_preap_rx_checks = {TESLA_PREAP_RX_CHECKS, TESLA_PREAP_RX_LEN};

CanMsgFwd TESLA_PREAP_FWD_MODDED[] = {
  //steering
  {.msg = {0x488,0,4},.fwd_to_bus=0,.expected_timestep = 100000U,.counter_mask_H=0x00000000,.counter_mask_L=0x000F0000}, // DAS_steeringControl - Lat Control - 20Hz
  //used for control
  {.msg = {0x3E9,0,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00F00000,.counter_mask_L=0x00000000}, // DAS_bodyControls - Control Body - 2Hz
  //used for IC integration
  {.msg = {0x399,0,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00F00000,.counter_mask_L=0x00000000}, // DAS_status - Status - 2Hz
  {.msg = {0x2B9,0,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00E00000,.counter_mask_L=0x00000000}, // DAS_control - Long Control 
  {.msg = {0x389,0,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00F00000,.counter_mask_L=0x00000000}, // DAS_status2 - Status - 2Hz
  {.msg = {0x329,0,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix0 - Status - 1Hz - nocounter/nochecksum
  {.msg = {0x369,0,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix1 - Status - 1Hz - nocounter/nochecksum
  {.msg = {0x349,0,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix3 - Status - 1Hz - nocounter/nochecksum
  {.msg = {0x3A9,0,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_telemetry - Lane Type - 10Hz - nocounter/nochecksum
  {.msg = {0x3B1,0,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_telemetryFurniture - Lane Type - 25Hz - nocounter/nochecksum
  {.msg = {0x309,0,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_object - Lead Car - 30Hz - nocounter/nochecksum
  {.msg = {0x239,0,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0xF0000000,.counter_mask_L=0x00000000}, // DAS_lanes - Path/Lanes - 10Hz - nochecksum
}; 

bool autopilot_enabled = false;
bool eac_enabled = false;
bool autopark_enabled = false;
bool epas_inhibited = false;

static uint8_t tesla_compute_checksum(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = (uint8_t)(addr) + (uint8_t)((unsigned int)(addr) >> 8U);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)GET_BYTE(to_push, i);
  }
  return checksum;
}

static uint8_t tesla_compute_crc(uint32_t MLB, uint32_t MHB , int msg_len) {
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

static bool tesla_compute_fwd_checksum(CANPacket_t *to_fwd) {
    uint8_t checksum = tesla_compute_checksum(to_fwd); 
    bool valid = false;
    int addr = GET_ADDR(to_fwd);

    if (addr == 0x488) {
      WORD_TO_BYTE_ARRAY(&to_fwd->data[0],(GET_BYTES_04(to_fwd)  | (checksum <<  24)));
      valid = true;
    }

    if ((addr == 0x209) || (addr == 0x2B9)) {
      WORD_TO_BYTE_ARRAY(&to_fwd->data[4],(GET_BYTES_48(to_fwd)  | (checksum << 24)));
      valid = true;
    }

    //DAS_bodyControls DAS_status DAS_status2
    if ((addr == 0x3E9) || (addr == 0x399) || (addr == 0x389)) {
      WORD_TO_BYTE_ARRAY(&to_fwd->data[4],(GET_BYTES_48(to_fwd)  | (checksum << 24)));
      valid = true;
    }

    //DAS_lanes DAS_warningMatrix0 DAS_warningMatrix1 DAS_warningMatrix3
    if ((addr == 0x239) || (addr == 0x329) || (addr == 0x369) || (addr == 0x349)) {
      valid = true;
    }

    // DAS_telemetry DAS_telemetryFurniture DAS_objects
    if ((addr == 0x3A9) || (addr == 0x3B1) || (addr == 0x309)) {
      valid = true;
    }

    return valid;
}

static bool tesla_compute_fwd_should_mod(CANPacket_t *to_fwd) {
    bool valid = false;
    int addr = GET_ADDR(to_fwd);

    if (!has_ap_hardware) {
      return valid;
    }

    if (addr == 0x488) {
      valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    if (addr == 0x209) {
      valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    if (addr == 0x2B9) {
      valid = !(autopilot_enabled || eac_enabled || autopark_enabled);      
    }

    //check the ones for IC integration
    //DAS_bodyControls
    if (addr == 0x3E9) {
      valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    //DAS_status - send as long as the timestamp is correct
    if (addr == 0x399) {
      valid = true; //!(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    //DAS_status2 - send as long as the timestamp is correct
    if (addr == 0x389) {
      valid = true; //!(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    //DAS_lanes
    if (addr == 0x239) {
      valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    //DAS_warningMatrix0
    if (addr == 0x329) {
      valid = true; //!(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    //DAS_warningMatrix1
    if (addr == 0x369) {
      valid = true; //!(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    //DAS_warningMatrix3
    if (addr == 0x349) {
      valid = true; //!(autopilot_enabled || eac_enabled || autopark_enabled);
    }

    // DAS_telemetry
    if (addr == 0x3A9) {
      if (has_ap_hardware) {
        int mux = GET_BYTES_04(to_fwd) & 0xFF;
        valid = ((!(autopilot_enabled || eac_enabled || autopark_enabled)) && (mux == 0));
      } else {
        valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
      }      
    }

    // DAS_telemetryFurniture
    if (addr == 0x3B1) {
      if (has_ap_hardware) {
        int mux = GET_BYTES_04(to_fwd) & 0x0F;
        valid = ((!(autopilot_enabled || eac_enabled || autopark_enabled)) && (mux == 0));
      } else {
        valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
      }      
    }

    // DAS_objects
    if (addr == 0x309) {
      if (has_ap_hardware) {
        int mux = GET_BYTES_04(to_fwd) & 0x07;
        valid = ((!(autopilot_enabled || eac_enabled || autopark_enabled)) && (mux == 0));
      } else {
        valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
      }      
    }

    return valid;
}

static void teslaPreAp_fwd_to_radar_as_is(uint8_t bus_num, CANPacket_t *to_fwd, uint16_t addr) {
  if (has_ap_hardware) {
    return;
  }
  if (!do_radar_emulation) {
    return;
  }
  CANPacket_t to_send;
  to_send.returned = 0U;
  to_send.rejected = 0U;
  to_send.extended = to_fwd->extended;
  to_send.addr = addr;
  to_send.bus = bus_num;
  to_send.data_len_code = to_fwd->data_len_code;
  uint32_t RDLR = GET_BYTES_04(to_fwd);
  uint32_t RDHR = GET_BYTES_48(to_fwd);
  WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
  WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
  can_send(&to_send, bus_num, true);
}

static void teslaPreAp_fwd_to_radar_modded(uint8_t bus_num, CANPacket_t *to_fwd) {
  if (has_ap_hardware) {
    return;
  }
  if (!do_radar_emulation) {
    return;
  }
  int32_t addr = GET_ADDR(to_fwd);
  CANPacket_t to_send;
  to_send.returned = 0U;
  to_send.rejected = 0U;
  to_send.extended = to_fwd->extended;
  to_send.bus = bus_num;
  to_send.data_len_code = to_fwd->data_len_code;

  uint32_t RDLR = GET_BYTES_04(to_fwd);
  uint32_t RDHR = GET_BYTES_48(to_fwd);
  //now modd messages as needed
  if (addr == 0x405 )
  {
    to_send.addr = (0x2B9 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);
    return;
  }
  if (addr == 0x398 )
  {
    //change frontradarHW = 1  and dashw = 1
    //SG_ GTW_dasHw : 7|2@0+ (1,0) [0|0] ""  NEO
    //SG_ GTW_parkAssistInstalled : 11|2@0+ (1,0) [0|0] ""  NEO

    RDLR = RDLR & 0xFFFFF33F;
    RDLR = RDLR | 0x100; //Park Assist
    RDLR = RDLR | 0x440; //forwardRadarHw, dasHw
    // change the autopilot to 1
    RDHR = RDHR & 0xCFFF0F0F; //take out values for autopilot, radarPosition and epasType
    RDHR = RDHR | 0x10000000 | (radarPosition << 4) | (radarEpasType << 12);
    
    //now change address and send to radar
    to_send.addr = (0x2A9 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x00E )
  {
    to_send.addr = (0x199);
    //check if angular speed sends SNA (0x3FFF)
    if (((RDLR >> 16) & 0xFF3F) == 0xFF3F) {
      //if yes replace 0x3FFFF with 0x2000 which is 0 angular change
      RDLR = (RDLR & 0x00C0FFFF) | (0x0020 << 16);
      //if this is the case, most likely we need to change the model too
      //so remove CRC and StW_AnglHP_Sens_Id (1st octet of RDHR)
      RDHR = RDHR & 0x00FFFFF0;
      //force StW_AnglHP_Sens_Id to DELPHI (0x04 1st octet of RDHR)
      RDHR = RDHR | 0x00000004;
      //compute new CRC
      int crc = tesla_compute_crc(RDLR, RDHR,7);
      //Add new CRC
      RDHR = RDHR | (crc << 24);
    }
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);
    return;
  }

  if (addr == 0x20A )
  {
    to_send.addr = (0x159 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);
    return;
  }
  if ((addr == 0x148) && (has_ibooster)) {
    to_send.addr = (0x1A9);
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);
    return;
  }
  if (addr == 0x115 )
  {
    to_send.addr = (0x129 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);

    //we don't get 0x148 DI_espControl so send as 0x1A9 on CAN1 and also as 0x148 on CAN0
    if (!has_ibooster) {
      to_send.returned = 0U;
      to_send.rejected = 0U;
      to_send.data_len_code = len_to_dlc(0x05);
      int counter = ((RDHR & 0xF0) >> 4 ) & 0x0F;
      RDLR = 0x000C0000 | (counter << 28);
      int cksm = (0x38 + 0x0C + (counter << 4)) & 0xFF;
      RDHR = cksm;
      WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
      WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
      can_send(&to_send, 0, true);
      to_send.addr = (0x1A9 );
      can_send(&to_send, bus_num, true);
    }
    return;
  }

  if (addr == 0x145) 
  {
    to_send.addr = (0x149);
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);

    return;
  }
  if ((addr == 0x175) && (has_ibooster)){
    to_send.addr = (0x169);
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);
    return;
  }
  if (addr == 0x118 )
  {
    to_send.addr = (0x119 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);
    //we don't get 0x175 ESP_wheelSpeeds so send as 0x169 on CAN1 and also as 0x175 on CAN0
    if (!has_ibooster) {
      int counter = GET_BYTES_48(to_fwd)  & 0x0F;
      to_send.addr = (0x169 );
      to_send.returned = 0U;
      to_send.rejected = 0U;
      to_send.data_len_code = len_to_dlc(0x08);
      int32_t speed_kph = (((0xFFF0000 & RDLR) >> 16) * 0.05 -25) * 1.609;
      if (speed_kph < 0) {
        speed_kph = 0;
      }
      if (((0xFFF0000 & RDLR) >> 16) == 0xFFF) {
        speed_kph = 0x1FFF; //0xFFF is signal not available for DI_Torque2 speed 0x118; should be SNA or 0x1FFF for 0x169
      } else {
        speed_kph = (int)(speed_kph/0.04) & 0x1FFF;
      }
      RDLR = (speed_kph | (speed_kph << 13) | (speed_kph << 26)) & 0xFFFFFFFF;
      RDHR = ((speed_kph  >> 6) | (speed_kph << 7) | (counter << 20)) & 0x00FFFFFF;
      int cksm = 0x76;
      cksm = (cksm + (RDLR & 0xFF) + ((RDLR >> 8) & 0xFF) + ((RDLR >> 16) & 0xFF) + ((RDLR >> 24) & 0xFF)) & 0xFF;
      cksm = (cksm + (RDHR & 0xFF) + ((RDHR >> 8) & 0xFF) + ((RDHR >> 16) & 0xFF) + ((RDHR >> 24) & 0xFF)) & 0xFF;
      RDHR = RDHR | (cksm << 24);
      WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
      WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
      can_send(&to_send, bus_num, true);
    }
    return;
  }
  if (addr == 0x108 )
  {
    to_send.addr = (0x109 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x308 )
  {
    to_send.addr = (0x209 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x45 )
  {
    to_send.addr = (0x219 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);

    return;
  }
  if (addr == 0x30A)
  {
    to_send.addr = (0x2D9 );
    WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
    WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
    can_send(&to_send, bus_num, true);

    return;
  }
  
}

static void teslaPreAp_generate_message(int id) {
  int index = get_addr_index(id, 0, TESLA_PREAP_FWD_MODDED, sizeof(TESLA_PREAP_FWD_MODDED)/sizeof(TESLA_PREAP_FWD_MODDED[0]),true);
  if (index == -1) {
    return;
  }
  
  //is the data valid to process?
  if (!TESLA_PREAP_FWD_MODDED[index].is_valid) {
    //return;
  }
  //create message
  CANPacket_t to_send;

  to_send.returned = 0U;
  to_send.rejected = 0U;
  to_send.extended = 0U;
  to_send.addr = id;
  to_send.bus = TESLA_PREAP_FWD_MODDED[index].fwd_to_bus;
  to_send.data_len_code = len_to_dlc(TESLA_PREAP_FWD_MODDED[index].msg.len);
  uint32_t RDLR = TESLA_PREAP_FWD_MODDED[index].dataL;
  uint32_t RDHR = TESLA_PREAP_FWD_MODDED[index].dataH;
  
  //these messages need counter added
  //0x3E9 0x399 0x389 0x239 0x488 0x2B9
  if (id == 0x488) {
    RDLR = RDLR | (DAS_steeringControl_idx << 16);
    DAS_steeringControl_idx = (DAS_steeringControl_idx + 1) % 16;
  }
  if (id == 0x2B9) {
    RDHR = RDHR | (DAS_control_idx << 21);
    DAS_control_idx = (DAS_control_idx + 1) % 7;
  }
  if (id == 0x3E9) {
    RDHR = RDHR | (DAS_bodyControls_idx << 20);
    DAS_bodyControls_idx = (DAS_bodyControls_idx + 1) % 16;
  }
  if (id == 0x399) {
    RDHR = RDHR | (DAS_status_idx << 20);
    DAS_status_idx = (DAS_status_idx + 1) % 16;
  }
  if (id == 0x389) {
    RDHR = RDHR | (DAS_status2_idx << 20);
    DAS_status2_idx = (DAS_status2_idx + 1) % 16;
  }
  if (id == 0x239) {
    RDHR = RDHR | (DAS_lanes_idx << 28);
    DAS_lanes_idx = (DAS_lanes_idx + 1) % 16;
  }
  //now do the checksums
  WORD_TO_BYTE_ARRAY(&to_send.data[0],RDLR);
  WORD_TO_BYTE_ARRAY(&to_send.data[4],RDHR);
  tesla_compute_fwd_checksum(&to_send);
  //send message
  can_send(&to_send, TESLA_PREAP_FWD_MODDED[index].fwd_to_bus, true);
}

static void send_fake_message(int msg_len, int msg_addr, uint8_t bus_num, uint32_t data_lo, uint32_t data_hi) {
  CANPacket_t to_send;
  to_send.returned = 0U;
  to_send.rejected = 0U;
  to_send.extended = 0U;
  to_send.addr = msg_addr;
  to_send.bus = bus_num;
  to_send.data_len_code = len_to_dlc(msg_len);
  WORD_TO_BYTE_ARRAY(&to_send.data[0],data_lo);
  WORD_TO_BYTE_ARRAY(&to_send.data[4],data_hi);
  can_send(&to_send, bus_num, true);
}

static void do_EPB_epasControl(void) {
  if (has_ibooster) {
    return;
  }
  uint32_t MLB;
  uint32_t MHB; 
  MLB = 0x01 + (EPB_epasControl_idx << 8) + ((0x17 + EPB_epasControl_idx) << 16); 
  MHB = 0x00;
  send_fake_message(3,0x214,0,MLB,MHB);
  EPB_epasControl_idx++;
  EPB_epasControl_idx = EPB_epasControl_idx % 16;
}

static void do_fake_stalk_cancel(void) {
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
  int crc = tesla_compute_crc(MLB, MHB,7);
  MHB = MHB + (crc << 24);
  DAS_lastStalkH = MHB;
  send_fake_message(8,0x45,0,MLB,MHB);
}

static void teslaPreAp_send_IC_messages(void) {
  //generate everything at higher rate than 10Hz
  //DAS_telemetry
  teslaPreAp_generate_message(0x3A9);
  //DAS_telemetryFurniture
  teslaPreAp_generate_message(0x3B1);
  //DAS_object
  teslaPreAp_generate_message(0x309);
  //DAS_lane
  teslaPreAp_generate_message(0x239);
  //EPB_epasControl 
  do_EPB_epasControl();
  //DAS_steeringControl
  teslaPreAp_generate_message(0x488);
  //generate everything at 2Hz
  if ((IC_send_counter == 1) || (IC_send_counter == 6)){
    //DAS_bodyControls
    teslaPreAp_generate_message(0x3E9);
    //DAS_status
    teslaPreAp_generate_message(0x399);
    //DAS_status2
    teslaPreAp_generate_message(0x389);
    //DAS_control
    teslaPreAp_generate_message(0x2B9);
  }
  //generate everything at 1Hz
  if (IC_send_counter == 3) {
    //DAS_warningMatrix0
    teslaPreAp_generate_message(0x329);
    //DAS_warningMatrix1
    teslaPreAp_generate_message(0x369);
    //DAS_warningMatrix3
    teslaPreAp_generate_message(0x349);
  }
  IC_send_counter = (IC_send_counter + 1) % 10;
}

static int tesla_rx_hook(CANPacket_t *to_push) {
  //update gmlan for giraffe control
  if ((hw_type == HW_TYPE_WHITE_PANDA) || (hw_type == HW_TYPE_WHITE_PANDA))
  {
    //we're still in tesla safety mode, reset the timeout counter and make sure our output is enabled
    set_gmlan_digital_output(0); //GMLAN_HIGH
    reset_gmlan_switch_timeout(); 
  };

  bool valid = false;
  if (has_ap_hardware) {
    valid = addr_safety_check(to_push, tesla_powertrain ? (&tesla_pt_rx_checks) : (&tesla_rx_checks),
                                 NULL, NULL, NULL);
  } else {
    valid = addr_safety_check(to_push, (&tesla_preap_rx_checks),
                                 NULL, NULL, NULL);
  }

  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (((bus == 0) && (addr == 0x671)) || 
      ((bus == tesla_radar_can) && (addr = 0x651))) {
      valid = true;
  }

  if ((bus == 0) && (addr == 0x39D) && (!has_ibooster_ecu)) {
    //found IBST_status, it has official ibooster
    has_ibooster = true;
  }

  //looking for radar messages to see if we have a timeout
  if ((addr == 0x300) && (bus == tesla_radar_can)) 
  {
    uint32_t ts = microsecond_timer_get();
    uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_last_radar_signal);
    if (tesla_radar_status == 1) {
      tesla_radar_status = 2;
      tesla_last_radar_signal = ts;
    } else
    if ((ts_elapsed > TESLA_RADAR_TIMEOUT) && (tesla_radar_status > 0)) {
      tesla_radar_status = 0;
    } else 
    if ((ts_elapsed <= TESLA_RADAR_TIMEOUT) && (tesla_radar_status == 2)) {
      tesla_last_radar_signal = ts;
    }
  }

  //0x631 is sent by radar to initiate the sync
  if ((addr == 0x631) && (bus == tesla_radar_can))
  {
    uint32_t ts = microsecond_timer_get();
    uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_last_radar_signal);
    if (tesla_radar_status == 0) {
      tesla_radar_status = 1;
      tesla_last_radar_signal = ts;
    } else
    if ((ts_elapsed > TESLA_RADAR_TIMEOUT) && (tesla_radar_status > 0)) {
      tesla_radar_status = 0;
    } else 
    if ((ts_elapsed <= TESLA_RADAR_TIMEOUT) && (tesla_radar_status > 0)) {
      tesla_last_radar_signal = ts;
    }
  }

  if(valid) {
    if(bus == 0) {
      if (!tesla_powertrain) {
        if ((addr == 0x348) && (!has_ap_hardware)) {
          //use GTW_status at 10Hz to generate the IC messages for nonAP cars
          teslaPreAp_send_IC_messages();
          //ALSO use this for radar timeout, this message is always on
          uint32_t ts = microsecond_timer_get();
          uint32_t ts_elapsed = get_ts_elapsed(ts, tesla_last_radar_signal);
          if ((ts_elapsed > TESLA_RADAR_TIMEOUT) && (tesla_radar_status > 0)) {
            tesla_radar_status = 0;
          } 
        }

        if (addr == 0x318) {
          int hour = (GET_BYTES_04(to_push) & 0x1F000000) >> 24;
          int minute = (GET_BYTES_48(to_push) & 0x3F00) >> 8;
          int second = (GET_BYTES_04(to_push) & 0x3F0000) >> 16;
          current_car_time = (hour * 3600) + (minute * 60) + second;
        }

        if (addr == 0x45)  {
          //first save for future use
          DAS_lastStalkL = GET_BYTES_04(to_push);
          DAS_lastStalkH = GET_BYTES_48(to_push);
          // 6 bits starting at position 0
          if (!has_ap_hardware) {
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
            //if using pedal, send a cancel immediately to cancel the CC
            if ((pedalCan != -1) && (pedalEnabled == 1) && (ap_lever_position > 1)) {
              do_fake_stalk_cancel();
            }
          }
        }
  
        if (addr == 0x214) {
          //has ibooser or otherwise we don't get EPB_epasControl
          if (has_ap_hardware) {
            epas_inhibited = (GET_BYTES_04(to_push) & 0x07) == 0;
          }
        }

        if (addr == 0x370) {
          // Steering angle: (0.1 * val) - 819.2 in deg.
          // Store it 1/10 deg to match steering request
          int angle_meas_new = (((GET_BYTE(to_push, 4) & 0x3F) << 8) | GET_BYTE(to_push, 5)) - 8192;
          update_sample(&angle_meas, angle_meas_new);
        }

        if(addr == 0x155) {
          // Vehicle speed: (0.01 * val) * KPH_TO_MPS
          vehicle_speed = ((GET_BYTE(to_push, 5) << 8) | (GET_BYTE(to_push, 6))) * 0.01 / 3.6;
          vehicle_moving = vehicle_speed > 0.;
        }
      }

      if(addr == (tesla_powertrain ? 0x106 : 0x108)) {
        // Gas pressed - only for ACC for now
        if (has_ap_hardware) {
          gas_pressed = ((GET_BYTE(to_push, 6) != 0) && (!enable_hao));
        }
      }

      if(addr == (tesla_powertrain ? 0x1f8 : 0x20a)) {
        // Brake pressed - only for ACC for now
        if (has_ap_hardware) {
          brake_pressed = ((GET_BYTE(to_push, 0) & 0x0C) >> 2 != 1);
        }
      }

      if(addr == (tesla_powertrain ? 0x256 : 0x368)) {
        // Cruise state
        int cruise_state = (GET_BYTE(to_push, 1) >> 4);
        bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                              (cruise_state == 3) ||  // STANDSTILL
                              (cruise_state == 4) ||  // OVERRIDE
                              (cruise_state == 6) ||  // PRE_FAULT
                              (cruise_state == 7);    // PRE_CANCEL
        if (has_ap_hardware) {
          if(cruise_engaged && !cruise_engaged_prev && !(autopilot_enabled || eac_enabled || autopark_enabled) && !epas_inhibited) {
            time_cruise_engaged = microsecond_timer_get();
          }
          
          if((time_cruise_engaged !=0) && (get_ts_elapsed(microsecond_timer_get(),time_cruise_engaged) >= TIME_TO_ENGAGE)) {
            if (cruise_engaged && !(autopilot_enabled || eac_enabled || autopark_enabled) && !epas_inhibited) {
              controls_allowed = 1;
            }
            time_cruise_engaged = 0;
          }
          
          if(!cruise_engaged || epas_inhibited) {
            controls_allowed = 0;
          }
        }
        cruise_engaged_prev = cruise_engaged;
      }
    }

    if (bus == 2) {
      if ((addr == 0x399) && (has_ap_hardware)) {
        // Autopilot status
        int autopilot_status = (GET_BYTE(to_push, 0) & 0xF);
        autopilot_enabled = (autopilot_status == 3) ||  // ACTIVE_1
                            (autopilot_status == 4);// ||  // ACTIVE_2
                            //(autopilot_status == 5);    // ACTIVE_NAVIGATE_ON_AUTOPILOT
        if (autopilot_enabled || eac_enabled || autopark_enabled) {
          controls_allowed = 0;
        }
      }

      if ((addr == 0x219) && (has_ap_hardware)) {
        //autopark and eac status
        int psc_status = ((GET_BYTE(to_push, 0) & 0xF0) >> 4);
        int eac_status = (GET_BYTE(to_push, 1) & 0x07);
        eac_enabled = (eac_status == 1);
        autopark_enabled = (psc_status == 14) || ((psc_status >= 1) && (psc_status <=8));
        if (autopilot_enabled || eac_enabled || autopark_enabled) {
          controls_allowed = 0;
        }
      }

      if (addr == 0x2B9) {
        //AP1 DAS_control
        last_acc_status = ((GET_BYTE(to_push, 1)>> 4) & 0xF);
      }
    }

    if ((addr == 0x552) && ((bus == 2) || (bus == 0))) {
      pedalPressed = (int)((((GET_BYTES_04(to_push) & 0xFF00) >> 8) + ((GET_BYTES_04(to_push) & 0xFF) << 8)) * 0.050796813 -22.85856576);
      if (pedalCan == -1) {
        pedalCan = bus;
      }
    }


    if (tesla_powertrain) {
      // 0x2bf: DAS_control should not be received on bus 0
      generic_rx_checks((addr == 0x2bf) && (bus == 0));
    } else {
      // 0x488: DAS_steeringControl should not be received on bus 0
      generic_rx_checks((addr == 0x488) && (bus == 0));
    }
  }

  return valid;
}

static int tesla_tx_hook(CANPacket_t *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  if (has_ap_hardware) {
    if(!msg_allowed(to_send, 
                  tesla_powertrain ? TESLA_PT_TX_MSGS : TESLA_AP_TX_MSGS,
                  tesla_powertrain ? TESLA_PT_TX_LEN : TESLA_AP_TX_LEN)) {
      tx = 0;
    }
  } else {
    if(!msg_allowed(to_send, TESLA_PREAP_TX_MSGS, TESLA_PREAP_TX_LEN)) {
      tx = 0;
    }
  }

  if(relay_malfunction) {
    tx = 0;
  }

  if (addr == 0x659) {
    pedalEnabled = ((GET_BYTE(to_send, 5) >> 5) & 0x01);
  }

  //do not allow long control if not enabled
  if ((!tesla_longitudinal) && ((addr == 0x2B9) || (addr == 0x209))) {
    //{0x2B9, 0, 8},  // DAS_control - Long Control
    //{0x209, 0, 8},  // DAS_longControl - Long Control
    tx = 0;
  }

  //do not allow body controls if not enabled
  if ((!has_body_controls) && (addr == 0x3E9)) {
    //{0x3E9, 0, 8},  // DAS_bodyControls
    tx = 0;
  }

  //do not allow hud integration messages if not enabled
  if ((!has_hud_integration) && ((addr == 0x399) || (addr == 0x389) || (addr == 0x239) ||(addr == 0x309) || (addr == 0x3A9) || (addr == 0x329) || (addr == 0x369) || (addr == 0x349))) {
    //{0x399, 0, 8},  // DAS_status - HUD
    //{0x389, 0, 8},  // DAS_status2 - HUD
    //{0x239, 0, 8},  // DAS_lanes - HUD
    //{0x309, 0, 8},  // DAS_object - HUD
    //{0x3A9, 0, 8},  // DAS_telemetry - HUD
    //{0x329, 0, 8},  // DAS_warningMatrix0 - HUD
    //{0x369, 0, 8},  // DAS_warningMatrix1 - HUD
    //{0x349, 0, 8},  // DAS_warningMatrix3 - HUD
    tx = 0;
  }

  if(!tesla_powertrain && (addr == 0x488)) {
    // Steering control: (0.1 * val) - 1638.35 in deg.
    // We use 1/10 deg as a unit here
    int raw_angle_can = (((GET_BYTE(to_send, 0) & 0x7F) << 8) | GET_BYTE(to_send, 1));
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = GET_BYTE(to_send, 2) >> 6;
    bool steer_control_enabled = (steer_control_type != 0) &&  // NONE
                                 (steer_control_type != 3);    // DISABLED
    /*
    // Rate limit while steering
    if(controls_allowed && steer_control_enabled) {
      // Add 1 to not false trigger the violation
      float delta_angle_float;
      delta_angle_float = (interpolate(TESLA_LOOKUP_ANGLE_RATE_UP, vehicle_speed) * TESLA_DEG_TO_CAN) + 1.;
      int delta_angle_up = (int)(delta_angle_float);
      delta_angle_float =  (interpolate(TESLA_LOOKUP_ANGLE_RATE_DOWN, vehicle_speed) * TESLA_DEG_TO_CAN) + 1.;
      int delta_angle_down = (int)(delta_angle_float);
      int highest_desired_angle = desired_angle_last + ((desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = desired_angle_last - ((desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

      // Check for violation;
      violation |= max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);
    }
    */
    
    desired_angle_last = desired_angle;

    // Angle should be the same as current angle while not steering
    if(!controls_allowed && ((desired_angle < (angle_meas.min - 1)) || (desired_angle > (angle_meas.max + 1)))) {
      violation = true;
    }

    // No angle control allowed when controls are not allowed
    if(!controls_allowed && steer_control_enabled) {
      violation = true;
    }
  }

  if(!tesla_powertrain && (addr == 0x45)) {
    // No button other than cancel can be sent by us when we have AP
    if (has_ap_hardware) {
      int control_lever_status = (GET_BYTE(to_send, 0) & 0x3F);
      if((control_lever_status != 0) && (control_lever_status != 1)) {
        violation = true;
      }
    }
  }

  if(addr == (tesla_powertrain ? 0x2bf : 0x2b9)) {
    // DAS_control: longitudinal control message
    if (tesla_longitudinal) {
      // No AEB events may be sent by openpilot
      int aeb_event = GET_BYTE(to_send, 2) & 0x03U;
      if (aeb_event != 0) {
        violation = true;
      }

      // Don't allow any acceleration limits above the safety limits
      int raw_accel_max = ((GET_BYTE(to_send, 6) & 0x1FU) << 4) | (GET_BYTE(to_send, 5) >> 4);
      int raw_accel_min = ((GET_BYTE(to_send, 5) & 0x0FU) << 5) | (GET_BYTE(to_send, 4) >> 3);
      float accel_max = (0.04 * raw_accel_max) - 15;
      float accel_min = (0.04 * raw_accel_min) - 15;

      if ((accel_max > TESLA_MAX_ACCEL) || (accel_min > TESLA_MAX_ACCEL)){
        violation = true;
      }

      if ((accel_max < TESLA_MIN_ACCEL) || (accel_min < TESLA_MIN_ACCEL)){
        violation = true;
      }
    } else {
      violation = true;
    }
  }

  if(violation) {
    controls_allowed = 0;
    tx = 0;
  }

  if (has_ap_hardware) {
    if (fwd_data_message(to_send,TESLA_AP_FWD_MODDED,sizeof(TESLA_AP_FWD_MODDED)/sizeof(TESLA_AP_FWD_MODDED[0]),violation)) {
      //do not send if the message is in the forwards
      tx = 0;
    }
  } else {
    if (fwd_data_message(to_send,TESLA_PREAP_FWD_MODDED,sizeof(TESLA_PREAP_FWD_MODDED)/sizeof(TESLA_PREAP_FWD_MODDED[0]),violation)) {
      //do not send if the message is in the forwards
      tx = 0;
    }
  }

  return tx;
}

static int tesla_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  int fwd_modded = -2;

  /* TODOBB: add this for long to mod not to block and resend
  if(bus_num == 0) {
    // Chassis/PT to autopilot
    bus_fwd = 2;
  }

  if(bus_num == 2) {
    // Autopilot to chassis/PT
    int das_control_addr = (tesla_powertrain ? 0x2bf : 0x2b9);

    bool block_msg = false;
    if (!tesla_powertrain && (addr == 0x488)) {
      block_msg = true;
    }

    if (tesla_longitudinal && (addr == das_control_addr)) {
      block_msg = true;
    }

    if(!block_msg) {
      bus_fwd = 0;
    }
    */

  //check for disengagement
  if ((prev_controls_allowed != controls_allowed) && (controls_allowed == 0)) {
    time_op_disengaged = microsecond_timer_get();
  }
  prev_controls_allowed = controls_allowed;

  //do not forward pedal messages 0x551 and 0x552
  if (((addr == 0x551) || (addr == 0x552)) && ((pedalCan == bus_num) || (pedalCan == -1))) {
      return -1;
  }

  if ((!has_ap_hardware) && (do_radar_emulation)){
      //forward 0x671 on can0 to 0x641 on can1 radar UDS
      if ((bus_num == 0) && (addr == 0x671)) {
        //change addr
        teslaPreAp_fwd_to_radar_as_is(tesla_radar_can, to_fwd, 0x641);
        return -1;
      }
      //forward 0x651 on can1 to 0x681 on can0 radar UDS
      if ((bus_num == tesla_radar_can) && (addr == 0x651)) {
        //change addr
        teslaPreAp_fwd_to_radar_as_is(0, to_fwd, 0x681);
        return -1;
      }
  }

  if (has_ap_hardware) {
  //we check to see first if these are modded forwards
    fwd_modded = fwd_modded_message(to_fwd,TESLA_AP_FWD_MODDED,sizeof(TESLA_AP_FWD_MODDED)/sizeof(TESLA_AP_FWD_MODDED[0]),
            tesla_compute_fwd_should_mod,tesla_compute_fwd_checksum);
  } else {
    fwd_modded = fwd_modded_message(to_fwd,TESLA_PREAP_FWD_MODDED,sizeof(TESLA_PREAP_FWD_MODDED)/sizeof(TESLA_PREAP_FWD_MODDED[0]),
            tesla_compute_fwd_should_mod,tesla_compute_fwd_checksum);
  }
  if (fwd_modded != -2) {
    //it's a forward modded message, so just forward now
    return fwd_modded;
  }

  //radar emulation triggers
  if ((!has_ap_hardware) && (do_radar_emulation) && (bus_num == 0)) {
    //check all messages we need to also send to radar, moddified, after we receive 0x631 from radar
    //148 does not exist, we use 115 at the same frequency to trigger and pass static vals
    //175 does not exist, we use 118 at the same frequency to trigger and pass vehicle speed
    if (((tesla_radar_status >= 0 ) || (bosch_radar_vin_learn)) && ((addr == 0x20A ) || (addr == 0x118 ) || (addr == 0x108 ) ||  
    (addr == 0x115 ) ||  (addr == 0x145)))
    {
      teslaPreAp_fwd_to_radar_modded(tesla_radar_can, to_fwd);
      return -1;
    }

    //check all messages we need to also send to radar, moddified, all the time
    if  (((addr == 0xE ) || (addr == 0x308 ) || (addr == 0x45 ) || (addr == 0x398 ) ||
    (addr == 0x405 ) ||  (addr == 0x30A) ||  (addr == 0x175) ||  (addr == 0x148)))  {
      teslaPreAp_fwd_to_radar_modded(tesla_radar_can, to_fwd);
      return -1;
    }
  }

  if(bus_num == 0) {
    if (has_ap_hardware) {
      // Chassis to autopilot

      //we need to modify EPAS_sysStatus->EPAS_eacStatus from 2 to 1 otherwise we can never 
      //engage AutoPilot. Once we send the steering commands from OP the status
      //changes from 1-AVAILABLE to 2-ACTIVE and AutoPilot becomes unavailable
      //The condition has to be:
      // IF controls_allowed AND EPAS_eacStatus = 2 THEN EPAS_eacStatus = 1
      if ((addr == 0x370) && (controls_allowed == 1) && (!(autopilot_enabled || eac_enabled || autopark_enabled))) {
        int epas_eacStatus = ((GET_BYTE(to_fwd, 6) & 0xE0) >> 5);
        //we only change from 2 to 1 leaving all other values alone
        if (epas_eacStatus == 2) {
          WORD_TO_BYTE_ARRAY(&to_fwd->data[4],(GET_BYTES_48(to_fwd) & 0x001FFFFF) | 0X00200000);
          WORD_TO_BYTE_ARRAY(&to_fwd->data[4], GET_BYTES_48(to_fwd) | (tesla_compute_checksum(to_fwd) << 24));
        }
      }
      //do not forward IC integration stuff from 0 -> 2 because they should not even be there
      if ((addr == 0x399) || (addr == 0x389) || (addr == 0x239) ||(addr == 0x309) || (addr == 0x3A9) || (addr == 0x329) || (addr == 0x369) || (addr == 0x349)) {
        return -1;
      }
    } else {
      //no AP hardware so nothing to forward
      return -1;
    }
    bus_fwd = 2;
  }

  if(bus_num == 2) {
    if (has_ap_hardware) {
      //we take care of what needs to be modded via fwd_modded method
      //so make sure anything else is sent from 2 to 0

      //if disengage less than 3 seconds ago, 
      if ((controls_allowed == 0) && (get_ts_elapsed(microsecond_timer_get(),time_op_disengaged) <= TIME_TO_HIDE_ERRORS)) {
        //make DAS_status2->DAS_activationFailureStatus 0
        if (addr ==0x389) {
          WORD_TO_BYTE_ARRAY(&to_fwd->data[0],(GET_BYTES_04(to_fwd) & 0xFFFF3FFF)); 
          WORD_TO_BYTE_ARRAY(&to_fwd->data[4],(GET_BYTES_48(to_fwd)  & 0x00FFFFFF));
          WORD_TO_BYTE_ARRAY(&to_fwd->data[4],(GET_BYTES_48(to_fwd)| (tesla_compute_checksum(to_fwd) << 24)));
        } 
        

        //make DAS_status->DAS_autopilotState 2 so we don't trigger warnings
        if (addr == 0x399) {
          WORD_TO_BYTE_ARRAY(&to_fwd->data[0],(GET_BYTES_04(to_fwd)  & 0xFFFFFFF0) | 2); 
          WORD_TO_BYTE_ARRAY(&to_fwd->data[4],(GET_BYTES_48(to_fwd) & 0x00FFFFFF));
          WORD_TO_BYTE_ARRAY(&to_fwd->data[4],(GET_BYTES_48(to_fwd) | (tesla_compute_checksum(to_fwd) << 24)));
        } 

        //if disengage less than 3 seconds ago, hide warningMatrix values
        if ((addr == 0x329) || (addr == 0x349) || (addr == 0x369))  {
          WORD_TO_BYTE_ARRAY(&to_fwd->data[4],0x00);
          WORD_TO_BYTE_ARRAY(&to_fwd->data[4],0x00);
        } 
      }
    } else {
      return -1;
    }
      bus_fwd = 0;
  }

  if(relay_malfunction) {
    bus_fwd = -1;
  }

  return bus_fwd;
}

static const addr_checks* tesla_init(int16_t param) {
  if (param < 0) {
    bosch_radar_vin_learn = true;
    param = - param;
  }
  tesla_powertrain = GET_FLAG(param, FLAG_TESLA_POWERTRAIN);
  tesla_longitudinal = GET_FLAG(param, FLAG_TESLA_LONG_CONTROL);
  has_ap_hardware = GET_FLAG(param, FLAG_TESLA_HAS_AP);
  has_ibooster = GET_FLAG(param, FLAG_TESLA_HAS_AP);
  has_ibooster_ecu = GET_FLAG(param, FLAG_TESLA_HAS_IBOOSTER);
  has_acc = GET_FLAG(param, FLAG_TESLA_HAS_AP);
  has_hud_integration = GET_FLAG(param,FLAG_TESLA_HAS_IC_INTEGRATION);
  if GET_FLAG(param,FLAG_TESLA_RADAR_BEHIND_NOSECONE) {
    radarPosition = 1;
  }
  has_body_controls = true;
  do_radar_emulation = GET_FLAG(param, FLAG_TESLA_NEED_RADAR_EMULATION);
  enable_hao = GET_FLAG(param, FLAG_TESLA_ENABLE_HAO);
  controls_allowed = 0;
  //init gmlan for giraffe control
  if ((hw_type == HW_TYPE_WHITE_PANDA) || (hw_type == HW_TYPE_WHITE_PANDA))
  {
    gmlan_switch_init(1);
  };
  relay_malfunction_reset();
  

  if (tesla_powertrain) {
      return &tesla_pt_rx_checks;
  } else {
    if (has_ap_hardware) {
      return &tesla_rx_checks;
    } else {
      return &tesla_preap_rx_checks;
    }
  }
}

const safety_hooks tesla_hooks = {
  .init = tesla_init,
  .rx = tesla_rx_hook,
  .tx = tesla_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = tesla_fwd_hook,
};
