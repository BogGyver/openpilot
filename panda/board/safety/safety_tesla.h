#include "safety_forwards.h"

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number, bool skip_tx_hook);


const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_UP = {
    {2., 7., 17.},
    {5., .8, .25}};

const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_DOWN = {
    {2., 7., 17.},
    {5., 3.5, .8}};

const int TESLA_DEG_TO_CAN = 10;

const uint32_t TIME_TO_ENGAGE = 500000; //0.5s 1swait for AP status @ 2Hz
const uint32_t TIME_TO_HIDE_ERRORS = 4000000; //4s to hide potential fake DAS errors after disengage

uint32_t time_cruise_engaged = 0;
uint32_t time_op_disengaged = 0;

//for safetyParam parsing
const uint16_t TESLA_HAS_AP_HARDWARE = 1;
const uint16_t TESLA_HAS_ACC = 2;
const uint16_t TESLA_OP_LONG_CONTROL = 4;
const uint16_t TESLA_HUD_INTEGRATION = 8;
const uint16_t TESLA_BODY_CONTROLS = 16;
const uint16_t TESLA_RADAR_EMULATION = 32;
const uint16_t TESLA_ENABLE_HAO = 64;

bool has_ap_hardware = false;
bool has_acc = false;
bool has_op_long_control = false;
bool has_hud_integration = false;
bool has_body_controls = false;
bool do_radar_emulation = false;
bool enable_hao = false;
int last_acc_status = -1;
int prev_controls_allowed = 0;

//pedal pressed (with Pedal)
int pedalPressed = 0;
int pedalCan = -1;

//use for preAP IC integration
int IC_send_counter = 0;
int DAS_bodyControls_idx = 0;
int DAS_status_idx = 0;
int DAS_status2_idx = 0;
int DAS_lanes_idx = 0;
uint32_t DAS_lastStalkL = 0;
uint32_t DAS_lastStalkH = 0;
int time_at_last_stalk_pull = -1;
int current_car_time = -1;

//use for Bosch radar
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
int tesla_radar_should_send = 0;

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
    {0x488, 2, 4},  // DAS_steeringControl - Lat Control
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
  };

AddrCheckStruct  TESLA_PREAP_RX_CHECKS[] = {
    {.msg = {{0x370, 2, 8, .expected_timestep = 40000U}}},   // EPAS_sysStatus (25Hz)
    {.msg = {{0x108, 0, 8, .expected_timestep = 10000U}}},   // DI_torque1 (100Hz)
    {.msg = {{0x118, 0, 6, .expected_timestep = 10000U}}},   // DI_torque2 (100Hz)
    {.msg = {{0x155, 0, 8, .expected_timestep = 20000U}}},   // ESP_B (50Hz)
    {.msg = {{0x20a, 0, 8, .expected_timestep = 20000U}}},   // BrakeMessage (50Hz)
    {.msg = {{0x368, 0, 8, .expected_timestep = 100000U}}},  // DI_state (10Hz)
    {.msg = {{0x318, 0, 8, .expected_timestep = 100000U}}},  // GTW_carState (10Hz)
  };

CanMsgFwd TESLA_PREAP_FWD_MODDED[] = {
    //used for control
    {.msg = {0x3E9,2,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00F00000,.counter_mask_L=0x00000000}, // DAS_bodyControls - Control Body - 2Hz
    //used for IC integration
    {.msg = {0x399,2,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00F00000,.counter_mask_L=0x00000000}, // DAS_status - Status - 2Hz
    {.msg = {0x389,2,8},.fwd_to_bus=0,.expected_timestep = 500000U,.counter_mask_H=0x00F00000,.counter_mask_L=0x00000000}, // DAS_status2 - Status - 2Hz
    {.msg = {0x329,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix0 - Status - 1Hz - nocounter/nochecksum
    {.msg = {0x369,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix1 - Status - 1Hz - nocounter/nochecksum
    {.msg = {0x349,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_warningMatrix3 - Status - 1Hz - nocounter/nochecksum
    {.msg = {0x3A9,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_telemetry - Lane Type - 10Hz - nocounter/nochecksum
    {.msg = {0x3B1,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_telemetryFurniture - Lane Type - 25Hz - nocounter/nochecksum
    {.msg = {0x309,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0x00000000,.counter_mask_L=0x00000000}, // DAS_object - Lead Car - 30Hz - nocounter/nochecksum
    {.msg = {0x239,2,8},.fwd_to_bus=0,.expected_timestep = 1000000U,.counter_mask_H=0xF0000000,.counter_mask_L=0x00000000}, // DAS_lanes - Path/Lanes - 10Hz - nochecksum
  }; 

bool autopilot_enabled = false;
bool eac_enabled = false;
bool autopark_enabled = false;
bool epas_inhibited = false;

static uint8_t tesla_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
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

static bool tesla_compute_fwd_checksum(CAN_FIFOMailBox_TypeDef *to_fwd) {
    uint8_t checksum = tesla_compute_checksum(to_fwd); 
    bool valid = false;
    int addr = GET_ADDR(to_fwd);

    if (addr == 0x488) {
      to_fwd->RDLR = (to_fwd->RDLR | (checksum << 24));
      valid = true;
    }

    if ((addr == 0x209) || (addr == 0x2B9)) {
      to_fwd->RDHR = (to_fwd->RDHR | (checksum << 24));
      valid = true;
    }

    //DAS_bodyControls DAS_status DAS_status2
    if ((addr == 0x3E9) || (addr == 0x399) || (addr == 0x389)) {
      to_fwd->RDHR = (to_fwd->RDHR | (checksum << 24));
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

static bool tesla_compute_fwd_should_mod(CAN_FIFOMailBox_TypeDef *to_fwd) {
    bool valid = false;
    int addr = GET_ADDR(to_fwd);

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
        int mux = to_fwd->RDLR & 0xFF;
        valid = ((!(autopilot_enabled || eac_enabled || autopark_enabled)) && (mux == 0));
      } else {
        valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
      }      
    }

    // DAS_telemetryFurniture
    if (addr == 0x3B1) {
      if (has_ap_hardware) {
        int mux = to_fwd->RDLR & 0x0F;
        valid = ((!(autopilot_enabled || eac_enabled || autopark_enabled)) && (mux == 0));
      } else {
        valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
      }      
    }

    // DAS_objects
    if (addr == 0x309) {
      if (has_ap_hardware) {
        int mux = to_fwd->RDLR & 0x07;
        valid = ((!(autopilot_enabled || eac_enabled || autopark_enabled)) && (mux == 0));
      } else {
        valid = !(autopilot_enabled || eac_enabled || autopark_enabled);
      }      
    }

    return valid;
}

static void teslaPreAp_fwd_to_radar_as_is(uint8_t bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (has_ap_hardware) {
    return;
  }
  if ((!do_radar_emulation) || (tesla_radar_vin_complete !=7) || (tesla_radar_should_send==0) ) {
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

static void teslaPreAp_fwd_to_radar_modded(uint8_t bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (has_ap_hardware) {
    return;
  }
  if ((!do_radar_emulation) || (tesla_radar_vin_complete !=7) || (tesla_radar_should_send==0) ) {
    return;
  }
  int32_t addr = to_fwd->RIR >> 21;
  CAN_FIFOMailBox_TypeDef to_send;
  to_send.RIR = to_fwd->RIR | 1; // TXRQ
  to_send.RDTR = to_fwd->RDTR;
  to_send.RDLR = to_fwd->RDLR;
  to_send.RDHR = to_fwd->RDHR;
  uint32_t addr_mask = 0x001FFFFF;
  //now modd messages as needed
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
    //change frontradarHW = 1  and dashw = 1
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
      int crc = tesla_compute_crc(to_send.RDLR, to_send.RDHR,7);
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
  /*
  if ((addr == 0x148) && (DAS_noEpasHarness == 1)) 
  {
    to_send.RIR = (0x1A9 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);
    return;
  }
  */
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
  /*
  if ((addr == 0x175) && (DAS_noEpasHarness == 1)) 
  {
    to_send.RIR = (0x169 << 21) + (addr_mask & (to_fwd->RIR | 1));
    can_send(&to_send, bus_num, true);
    return;
  }
  */
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

static void teslaPreAp_generate_message(int id,uint32_t RIR, uint32_t RDTR) {
  int index = get_addr_index(id, 0, TESLA_PREAP_FWD_MODDED, sizeof(TESLA_PREAP_FWD_MODDED)/sizeof(TESLA_PREAP_FWD_MODDED[0]),true);
  if (index == -1) {
    return;
  }
  
  //is the data valid to process?
  if (!TESLA_PREAP_FWD_MODDED[index].is_valid) {
    return;
  }
  //create message
  CAN_FIFOMailBox_TypeDef to_send;
  uint32_t addr_mask = 0x001FFFFF;
  to_send.RIR = (id << 21) + (addr_mask & (RIR | 1));
  to_send.RDTR = (RDTR & 0xFFFFFFF0) | TESLA_PREAP_FWD_MODDED[index].msg.len;
  to_send.RDLR = TESLA_PREAP_FWD_MODDED[index].dataH;
  to_send.RDHR = TESLA_PREAP_FWD_MODDED[index].dataL;
  //these messages need counter added
  //0x3E9 0x399 0x389 0x239
  if (id == 0x3E9) {
    to_send.RDHR = to_send.RDHR | (DAS_bodyControls_idx << 20);
    DAS_bodyControls_idx = (DAS_bodyControls_idx + 1) % 16;
  }
  if (id == 0x399) {
    to_send.RDHR = to_send.RDHR | (DAS_status_idx << 20);
    DAS_status_idx = (DAS_status_idx + 1) % 16;
  }
  if (id == 0x389) {
    to_send.RDHR = to_send.RDHR | (DAS_status2_idx << 20);
    DAS_status2_idx = (DAS_status2_idx + 1) % 16;
  }
  if (id == 0x239) {
    to_send.RDHR = to_send.RDHR | (DAS_lanes_idx << 20);
    DAS_lanes_idx = (DAS_lanes_idx + 1) % 16;
  }
  //now do the checksums
  tesla_compute_fwd_checksum(&to_send);
  //send message
  can_send(&to_send, TESLA_PREAP_FWD_MODDED[index].fwd_to_bus, false);
}

static void teslaPreAp_send_IC_messages(uint32_t RIR, uint32_t RDTR) {
  //generate everything at higher rate than 10Hz
  //DAS_telemetry
  teslaPreAp_generate_message(0x3A9,RIR,RDTR);
  //DAS_telemetryFurniture
  teslaPreAp_generate_message(0x3B1,RIR,RDTR);
  //DAS_object
  teslaPreAp_generate_message(0x309,RIR,RDTR);
  //DAS_lane
  teslaPreAp_generate_message(0x239,RIR,RDTR);
  //generate everything at 2Hz
  if ((IC_send_counter == 1) || (IC_send_counter == 6)){
    //DAS_bodyControls
    teslaPreAp_generate_message(0x3E9,RIR,RDTR);
    //DAS_status
    teslaPreAp_generate_message(0x399,RIR,RDTR);
    //DAS_status2
    teslaPreAp_generate_message(0x389,RIR,RDTR);
  }
  //generate everything at 1Hz
  if (IC_send_counter == 3) {
    //DAS_warningMatrix0
    teslaPreAp_generate_message(0x329,RIR,RDTR);
    //DAS_warningMatrix1
    teslaPreAp_generate_message(0x369,RIR,RDTR);
    //DAS_warningMatrix3
    teslaPreAp_generate_message(0x349,RIR,RDTR);
  }
  IC_send_counter = (IC_send_counter + 1) % 10;
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
  int crc = tesla_compute_crc(MLB, MHB,7);
  MHB = MHB + (crc << 24);
  DAS_lastStalkH = MHB;
  send_fake_message(RIR,RDTR,8,0x45,0,MLB,MHB);
}

static int tesla_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  //update gmlan for giraffe control
  if ((hw_type == HW_TYPE_WHITE_PANDA) || (hw_type == HW_TYPE_WHITE_PANDA))
  {
    //we're still in tesla safety mode, reset the timeout counter and make sure our output is enabled
    set_gmlan_digital_output(0); //GMLAN_HIGH
    reset_gmlan_switch_timeout(); 
  };

  bool valid = false;
  if (has_ap_hardware) {
    valid = addr_safety_check(to_push, TESLA_AP_RX_CHECKS, sizeof(TESLA_AP_RX_CHECKS)/sizeof(TESLA_AP_RX_CHECKS[0]),
                                 NULL, NULL, NULL);
  } else {
    valid = addr_safety_check(to_push, TESLA_PREAP_RX_CHECKS, sizeof(TESLA_PREAP_RX_CHECKS)/sizeof(TESLA_PREAP_RX_CHECKS[0]),
                                 NULL, NULL, NULL);
  }

  if(valid) {
    int bus = GET_BUS(to_push);
    int addr = GET_ADDR(to_push);

    if(bus == 0) {
      if ((addr == 0x348) && (!has_ap_hardware)) {
        //use GTW_status at 10Hz to generate the IC messages for nonAP cars
        teslaPreAp_send_IC_messages(to_push->RIR, to_push->RDTR);
      }

      if (addr == 0x318) {
        int hour = (to_push->RDLR & 0x1F000000) >> 24;
        int minute = (to_push->RDHR & 0x3F00) >> 8;
        int second = (to_push->RDLR & 0x3F0000) >> 16;
        current_car_time = (hour * 3600) + (minute * 60) + second;
      }

      if ((addr == 0x370) && (has_ap_hardware)) {
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

      if(addr == 0x108) {
        // Gas pressed - only for ACC for now
        if (has_ap_hardware) {
          gas_pressed = ((GET_BYTE(to_push, 6) != 0) && (!enable_hao));
        }
      }

      if(addr == 0x20a) {
        // Brake pressed - only for ACC for now
        if (has_ap_hardware) {
          brake_pressed = ((GET_BYTE(to_push, 0) & 0x0C) >> 2 != 1);
        }
      }

      if(addr == 0x368) {
        // Cruise state
        int cruise_state = (GET_BYTE(to_push, 1) >> 4);
        bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                              (cruise_state == 3) ||  // STANDSTILL
                              (cruise_state == 4) ||  // OVERRIDE
                              (cruise_state == 6) ||  // PRE_FAULT
                              (cruise_state == 7);    // PRE_CANCEL
        if (has_ap_hardware) {
          if(cruise_engaged && !cruise_engaged_prev && !(autopilot_enabled || eac_enabled || autopark_enabled) && !epas_inhibited) {
            time_cruise_engaged = TIM2->CNT;
          }
          
          if((time_cruise_engaged !=0) && (get_ts_elapsed(TIM2->CNT,time_cruise_engaged) >= TIME_TO_ENGAGE)) {
            if (cruise_engaged && !(autopilot_enabled || eac_enabled || autopark_enabled) && !epas_inhibited) {
              controls_allowed = 1;
            }
            time_cruise_engaged = 0;
          }
          
          if(!cruise_engaged || epas_inhibited) {
            controls_allowed = 0;
          }
        } else {
          if (cruise_engaged && !epas_inhibited) {
              controls_allowed = 1;
          }
        }
        cruise_engaged_prev = cruise_engaged;
      }

      if (addr == 0x45)  {
        //first save for future use
        DAS_lastStalkL = to_push->RDLR;
        DAS_lastStalkH = to_push->RDHR;
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
          if ((pedalCan != -1) && (ap_lever_position > 1)) {
            do_fake_stalk_cancel(to_push->RIR, to_push->RDTR);
          }
        }
      }
 
      if (addr == 0x214) {
        epas_inhibited = (to_push->RDLR & 0x07) == 0;
      }
    }

    if (bus == 2) {
      if ((addr == 0x370) && (!has_ap_hardware)) {
        // Steering angle: (0.1 * val) - 819.2 in deg.
        // Store it 1/10 deg to match steering request
        int angle_meas_new = (((GET_BYTE(to_push, 4) & 0x3F) << 8) | GET_BYTE(to_push, 5)) - 8192;
        update_sample(&angle_meas, angle_meas_new);
      }

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

    if ((addr == 0x552) && ((bus == 2) || (bus == 3))) {
      pedalPressed = (int)((((to_push->RDLR & 0xFF00) >> 8) + ((to_push->RDLR & 0xFF) << 8)) * 0.050796813 -22.85856576);
      if (pedalCan == -1) {
        pedalCan = bus;
      }
    }


    // 0x488: DAS_steeringControl should not be received on bus 0
    generic_rx_checks((addr == 0x488) && (bus == 0));
  }

  return valid;
}

static int tesla_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  if (has_ap_hardware) {
    if(!msg_allowed(to_send, TESLA_AP_TX_MSGS, sizeof(TESLA_AP_TX_MSGS) / sizeof(TESLA_AP_TX_MSGS[0]))) {
      tx = 0;
    }
  } else {
    if(!msg_allowed(to_send, TESLA_PREAP_TX_MSGS, sizeof(TESLA_PREAP_TX_MSGS) / sizeof(TESLA_PREAP_TX_MSGS[0]))) {
      tx = 0;
    }
  }

  if(relay_malfunction) {
    tx = 0;
  }

  //capture message for radarVIN and settings
  if ((!has_ap_hardware) && (do_radar_emulation) && (addr == 0x560)) {
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

  //do not allow long control if not enabled
  if ((!has_op_long_control) && ((addr == 0x2B9) || (addr == 0x209))) {
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

  if(addr == 0x488) {
    // Steering control: (0.1 * val) - 1638.35 in deg.
    // We use 1/10 deg as a unit here
    int raw_angle_can = (((GET_BYTE(to_send, 0) & 0x7F) << 8) | GET_BYTE(to_send, 1));
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = GET_BYTE(to_send, 2) >> 6;
    bool steer_control_enabled = (steer_control_type != 0) &&  // NONE
                                 (steer_control_type != 3);    // DISABLED

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

  if(addr == 0x45) {
    // No button other than cancel can be sent by us when we have AP
    if (has_ap_hardware) {
      int control_lever_status = (GET_BYTE(to_send, 0) & 0x3F);
      if((control_lever_status != 0) && (control_lever_status != 1)) {
        violation = true;
      }
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

static int tesla_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  int fwd_modded = -2;

  //check for disengagement
  if ((prev_controls_allowed != controls_allowed) && (controls_allowed == 0)) {
    time_op_disengaged = TIM2->CNT;
  }
  prev_controls_allowed = controls_allowed;

  //do not forward pedal messages 0x551 and 0x552
  if (((addr == 0x551) || (addr == 0x552)) && ((pedalCan == bus_num) || (pedalCan == -1))) {
      return -1;
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
  if ((!has_ap_hardware) && (do_radar_emulation)) {
    //check all messages we need to also send to radar, moddified, after we receive 0x631 from radar
    //148 does not exist, we use 115 at the same frequency to trigger and pass static vals
    //175 does not exist, we use 118 at the same frequency to trigger and pass vehicle speed
    if ((tesla_radar_status > 0 ) && ((addr == 0x20A ) || (addr == 0x118 ) || (addr == 0x108 ) ||  
    (addr == 0x115 ) ||  (addr == 0x148 ) || (addr == 0x145)))
    {
      teslaPreAp_fwd_to_radar_modded(tesla_radar_can, to_fwd);
    }

    //check all messages we need to also send to radar, moddified, all the time
    if  (((addr == 0xE ) || (addr == 0x308 ) || (addr == 0x45 ) || (addr == 0x398 ) ||
    (addr == 0x405 ) ||  (addr == 0x30A)))  {
      teslaPreAp_fwd_to_radar_modded(tesla_radar_can, to_fwd);
    }

    //forward to radar unmodded the UDS messages 0x641
    if  (addr == 0x641 ) {
      teslaPreAp_fwd_to_radar_as_is(tesla_radar_can, to_fwd);
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
          to_fwd->RDHR = (to_fwd->RDHR & 0x001FFFFF) | 0X00200000;
          to_fwd->RDHR = (to_fwd->RDHR | (tesla_compute_checksum(to_fwd) << 24));
        }
      }
      //do not forward IC integration stuff from 0 -> 2 because they should not even be there
      if ((addr == 0x399) || (addr == 0x389) || (addr == 0x239) ||(addr == 0x309) || (addr == 0x3A9) || (addr == 0x329) || (addr == 0x369) || (addr == 0x349)) {
        return -1;
      }
    } else {
      //no AP hardware so we need to mod EPAS Controls
      //there is no hurting in doing so because steering does not happen
      //unless we send the steering control message
      if (addr == 0x101)
      {
        to_fwd->RDLR = (to_fwd->RDLR & 0x0000FFFF) | 0x4000 | 0x1000; 
        to_fwd->RDHR = 0x00;
        // 0x4000: WITH_ANGLE, 0xC000: WITH_BOTH (angle and torque)
        // 0x1000: enabled LDW for haptic
        to_fwd->RDLR = (to_fwd->RDLR | (tesla_compute_checksum(to_fwd) << 16));
      }

      if (addr == 0x214) {
        to_fwd->RDLR = (to_fwd->RDLR & 0x0000FFF8) | 0x01; 
        to_fwd->RDHR = 0x00;
        // 0x01 - EPB_ALLOW_EAC
        to_fwd->RDLR = (to_fwd->RDLR | (tesla_compute_checksum(to_fwd) << 16));
      }
    }
    bus_fwd = 2;
  }

  if(bus_num == 2) {
      //we take care of what needs to be modded via fwd_modded method
      //so make sure anything else is sent from 2 to 0

      //if disengage less than 3 seconds ago, 
      if ((controls_allowed == 0) && (get_ts_elapsed(TIM2->CNT,time_op_disengaged) <= TIME_TO_HIDE_ERRORS)) {
        //make DAS_status2->DAS_activationFailureStatus 0
        if (addr ==0x389) {
          to_fwd->RDLR = (to_fwd->RDLR & 0xFFFF3FFF); 
          to_fwd->RDHR = (to_fwd->RDHR & 0x00FFFFFF);
          to_fwd->RDHR = (to_fwd->RDHR | (tesla_compute_checksum(to_fwd) << 24));
        } 
        

        //make DAS_status->DAS_autopilotState 2 so we don't trigger warnings
        if (addr == 0x399) {
          to_fwd->RDLR = ((to_fwd->RDLR & 0xFFFFFFF0) | 2); 
          to_fwd->RDHR = (to_fwd->RDHR & 0x00FFFFFF);
          to_fwd->RDHR = (to_fwd->RDHR | (tesla_compute_checksum(to_fwd) << 24));
        } 

        //if disengage less than 3 seconds ago, hide warningMatrix values
        if ((addr == 0x329) || (addr == 0x349) || (addr == 0x369))  {
          to_fwd->RDLR = (to_fwd->RDLR & 0x00000000);
          to_fwd->RDHR = (to_fwd->RDHR & 0x00000000);
        } 
      }
      bus_fwd = 0;
  }

  if(relay_malfunction) {
    bus_fwd = -1;
  }

  return bus_fwd;
}

static void tesla_init(int16_t param) {
  controls_allowed = 0;
  //init gmlan for giraffe control
  if ((hw_type == HW_TYPE_WHITE_PANDA) || (hw_type == HW_TYPE_WHITE_PANDA))
  {
    gmlan_switch_init(1);
  };
  relay_malfunction_reset();
  has_ap_hardware = GET_FLAG(param, TESLA_HAS_AP_HARDWARE);
  has_acc = GET_FLAG(param, TESLA_HAS_ACC);
  has_op_long_control = GET_FLAG(param, TESLA_OP_LONG_CONTROL);
  has_hud_integration = GET_FLAG(param, TESLA_HUD_INTEGRATION);
  has_body_controls = GET_FLAG(param, TESLA_BODY_CONTROLS);
  do_radar_emulation = GET_FLAG(param, TESLA_RADAR_EMULATION);
  enable_hao = GET_FLAG(param, TESLA_ENABLE_HAO);
}

const safety_hooks tesla_hooks = {
  .init = tesla_init,
  .rx = tesla_rx_hook,
  .tx = tesla_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = tesla_fwd_hook,
  .addr_check = TESLA_AP_RX_CHECKS,
  .addr_check_len = sizeof(TESLA_AP_RX_CHECKS)/sizeof(TESLA_AP_RX_CHECKS[0]),
};

const safety_hooks tesla_preap_hooks = {
  .init = tesla_init,
  .rx = tesla_rx_hook,
  .tx = tesla_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = tesla_fwd_hook,
  .addr_check = TESLA_PREAP_RX_CHECKS,
  .addr_check_len = sizeof(TESLA_PREAP_RX_CHECKS)/sizeof(TESLA_PREAP_RX_CHECKS[0]),
};
