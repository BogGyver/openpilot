// ********************* Includes *********************
#include "../config.h"
#include "libc.h"

#include "main_declarations.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

// uncomment for usb debugging via debug_console.py
#define TGW_USB
#define DEBUG_CAN
#define DEBUG_CTRL

#ifdef TGW_USB
  #include "drivers/uart.h"
  #include "drivers/usb.h"
#else
  // no serial either
  void puts(const char *a) {
    UNUSED(a);
  }
  void puth(unsigned int i) {
    UNUSED(i);
  }
  void puth2(unsigned int i) {
    UNUSED(i);
  }
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

#ifdef TGW_USB

#include "gateway/can.h"

// ********************* usb debugging *********************
// TODO: neuter this if we are not debugging
void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  CAN_FIFOMailBox_TypeDef *reply = (CAN_FIFOMailBox_TypeDef *)usbdata;
  int ilen = 0;
  while (ilen < MIN(len/0x10, 4) && can_pop(&can_rx_q, &reply[ilen])) {
    ilen++;
  }
  return ilen*0x10;
}
// send on serial, first byte to select the ring
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  uint8_t *usbdata8 = (uint8_t *)usbdata;
  uart_ring *ur = get_ring_by_number(usbdata8[0]);
  if ((len != 0) && (ur != NULL)) {
    if ((usbdata8[0] < 2U)) {
      for (int i = 1; i < len; i++) {
        while (!putc(ur, usbdata8[i])) {
          // wait
        }
      }
    }
  }
}
// send on CAN
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete() {
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_BULK_TRANSFER)) {
    usb_outep3_resume_if_paused();
  }
}

void usb_cb_enumeration_complete() {
  puts("USB enumeration complete\n");
  is_enumerated = 1;
}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      // so it's blocked over wifi
      switch (setup->b.wValue.w) {
        case 0:
          // only allow bootloader entry on debug builds
          #ifdef ALLOW_DEBUG
            if (hardwired) {
              puts("-> entering bootloader\n");
              enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
              NVIC_SystemReset();
            }
          #endif
          break;
        case 1:
          puts("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          puts("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (setup->b.wValue.w == 0xFFFFU) {
        puts("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (setup->b.wValue.w < BUS_MAX) {
        puts("Clearing CAN Tx queue\n");
        can_clear(can_queues[setup->b.wValue.w]);
      } else {
        puts("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf2: Clear UART ring buffer.
    case 0xf2:
      {
        uart_ring * rb = get_ring_by_number(setup->b.wValue.w);
        if (rb != NULL) {
          puts("Clearing UART queue.\n");
          clear_uart_buff(rb);
        }
        break;
      }
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** can port *****************************

// Toyota Checksum algorithm
uint8_t toyota_checksum(int addr, uint8_t *dat, int len){
  int cksum = 0;
  for(int ii = 0; ii < (len - 1); ii++){
    cksum = (cksum + dat[ii]); 
  }
  cksum += len;
  cksum += ((addr >> 8U) & 0xFF); // idh
  cksum += ((addr) & 0xFF); // idl
  return cksum & 0xFF;
}

#define CAN_UPDATE  0xF0 //bootloader
#define COUNTER_CYCLE 0xFU
uint8_t counter = 0;

void CAN1_TX_IRQ_Handler(void) {
  process_can(0);
}

void CAN2_TX_IRQ_Handler(void) {
  process_can(1);
}

void CAN3_TX_IRQ_Handler(void) {
  process_can(2);
}

#define MAX_TIMEOUT 50U
#define MAX_TIMEOUT2 200U
uint32_t timeout = 0;
uint32_t timeout_f1 = 0;
uint32_t timeout_f2 = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
#define STATE_AEB_CTRL 7U

uint8_t state = FAULT_STARTUP;

#define MODE_ACC_CTRL 1U
#define MODE_AEB_CTRL 2U
uint8_t ctrl_mode = 0;

bool send = 0;

//------------- BUS 1 - PTCAN ------------//

#define ACC_CTRL 0xF1
int acc_cmd = 0;
bool enable_acc = 0;
bool mini_car = 0;
bool distance = 0;
bool release_standstill = 0;

#define AEB_CTRL 0xF2
bool enable_aeb_control = 0;

int aeb_cmd = 0; // (dat0 << 2) | (dat1 >> 6);
uint8_t ds1stbk2 = 0; //dat1 & 7U
uint8_t ds1stat2 = 0; //(dat1 >> 3) & 7U
bool pcsopr = 0; //dat2 & 1U
bool pcsalm = 0; //(dat2 >> 1) & 1U
bool pcswar = 0; //(dat2 >> 2) & 1U
uint8_t irlt_req = 0; //dat3 & 3U
bool clextrgr = 0; //(dat3 >> 2) & 1U
bool ibtrgr = 0; //(dat3 >> 3) & 1U
bool pptrgr = 0; //(dat3 >> 4) & 1U
uint8_t PBATRGR = 0; //(dat3 >> 5) & 3U
bool pcsabk = 0; //(dat3 >> 7) & 1U
bool pbrtrgr = 0; //dat4 & 1U
bool prefill = 0; //(dat4 >> 1) & 1U
uint8_t VGRSTRGR = 0; //(dat4 >> 2) & 3U
bool AVSTRGR = 0; //(dat4 >> 4) & 1U
bool brkhld = 0; //(dat4 >> 5U) & 1U
bool pbprepmp = 0; //dat5 & 1U
bool pcsdis = 0; //(dat5 >> 3) & 1U

#define PCS_CTRL 0xF3
bool enable_pcs_ctrl = 0;

//------------- BUS 2 - DSU -------------//

#define DSU_ACC_CONTROL 0x343
bool acc_cancel = 0;

#define DSU_AEB_CMD 0x344
bool stock_aeb_active = 0;

#define ACC_HUD 0x411

#define DSU_PRECOLLISION 0x283 // 7 bytes
uint8_t pcs_counter = 0; // byte 0
int16_t pcs_force = 0; // byte2 | byte3
uint8_t DSS1STAT = 0; //(byte4 >> 2) &7U pcs status
uint8_t DSS1STBK = 0; //byte4 >> 5 brake status
bool pcs_active = 0; //(byte5 >> 1) & 1U
// toyota dbc values that need to be understood
uint8_t DSSFTRQD = 0; //byte1 & FU
bool PBRTRGR2 = 0; //(byte1 >> 4) &1U
bool DSSTPBZ = 0; //(byte1 >> 6U) &1U
bool DSLCCW1 = 0; //(byte1 >> 7U) &1U
bool PPTRGR2 = 0; //byte4 &1U
bool DSBHOK = 0; //(byte4 >> 1U) &1U
bool PCSABK2 = 0; //(byte5 >> 4) &1U
bool IBTRGR2 = 0; //(byte5 >> 5) &1U
uint8_t DSRBQH = 0; //(byte5 >> 6) &3U

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN1->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN1->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN1->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN1->sFIFOMailBox[0].RDHR;

    uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;

    #ifdef DEBUG_CAN
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #endif

    // CAN data buffer
    uint8_t dat[8];
    for (int i=0; i<8; i++) {
      dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
    }

    switch (address) {
      case CAN_UPDATE:
        if (GET_BYTES_04(&CAN1->sFIFOMailBox[0]) == 0xdeadface) {
          if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x0ab00b1e) {
            enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
            NVIC_SystemReset();
          } else if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x02b00b1e) {
            enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
            NVIC_SystemReset();
          } else {
            puts("Failed entering Softloader or Bootloader\n");
          }
        }
        break;
      case ACC_CTRL:
        // send this EXACTLY how ACC_CONTROL is sent
        if (dat[7] == toyota_checksum(address, dat, 8)){
          enable_acc = 1; // TODO: set this somewhere else.. 1D2? do we need this?
          distance = (dat[2] >> 4U) & 1U;
          acc_cmd = (dat[0] << 8U) | dat[1]; // ACC_CMD
          mini_car = (dat[2] >> 5U) & 1U;
          acc_cancel = (dat[3] & 1U);
          release_standstill = (dat[3] >> 7U) & 1U;
          // reset the timer
          timeout_f1 = 0;
          ctrl_mode = MODE_ACC_CTRL; // set ACC_CTRL mode bit
          #ifdef DEBUG_CTRL
            puts("GOT ACC_CTRL");
          #endif
        } else {
          #ifdef DEBUG_CTRL
            for(int ii = 0; ii<8; ii++){ 
              puth2(dat[ii]);
            }
            puts("\n expected: ");
            puth2(toyota_checksum(address, dat, 8));
            puts(" got: ");
            puth2(dat[7]);
            puts("\n");
          #endif
          state = FAULT_BAD_CHECKSUM;
          enable_acc = 0;
          acc_cmd = 0;
          distance = 0;
          mini_car = 0;
          acc_cancel = 0;
          release_standstill = 0;
        }
        to_fwd.RIR &= 0xFFFFFFFE; // do not fwd
        break;
      case AEB_CTRL:
        // send this EXACTLY how PRE_COLLISION2 is sent
        if (dat[7] == toyota_checksum(address, dat, 8)){
          // an emergency maneuver is being requested
          enable_aeb_control = 1;

          // collect all the vairables
          aeb_cmd = (dat[0] << 2) | ((dat[1] >> 6) & 3U);
          ds1stat2 = (dat[1] >> 3) & 7U;
          ds1stbk2 = (dat[1] & 7U);
          pcsopr = (dat[2] & 1U);
          pcsalm = (dat[2] >> 1) & 1U;
          pcswar = (dat[2] >> 2) & 1U;
          irlt_req = dat[3] & 3U;
          clextrgr = (dat[3] >> 2) & 1U;
          ibtrgr = (dat[3] >> 3) & 1U;
          pptrgr = (dat[3] >> 4) & 1U;
          PBATRGR = (dat[3] >> 5) & 3U;
          pcsabk = (dat[3] >> 7) & 1U;
          pbrtrgr = dat[4] & 1U;
          prefill = (dat[4] >> 1) & 1U;
          VGRSTRGR = (dat[4] >> 2) & 3U;
          AVSTRGR = (dat[4] >> 4) & 1U;
          brkhld = (dat[4] >> 5) & 1U; 
          pbprepmp = dat[5] & 1U;
          pcsdis = (dat[5] >> 3) & 1U;
          // reset the timer
          timeout_f2 = 0;
          ctrl_mode = MODE_AEB_CTRL; // set AEB_CTRL mode bit
          state = STATE_AEB_CTRL;
          #ifdef DEBUG_CTRL
            puts("GOT AEB_CTRL");
          #endif
        } else {
          #ifdef DEBUG_CTRL
            for(int ii = 0; ii<8; ii++){ 
              puth2(dat[ii]);
            }
            puts("\n expected: ");
            puth2(toyota_checksum(address, dat, 8));
            puts(" got: ");
            puth2(dat[7]);
            puts("\n");
          #endif
          enable_aeb_control = 0;
          aeb_cmd = 0;
          state = FAULT_BAD_CHECKSUM;
        }
        to_fwd.RIR &= 0xFFFFFFFE; // do not fwd
        break;
      default:
        // FWD as-is
        break;
    }
    // send to CAN3
    can_send(&to_fwd, 2, false);
    // next
    can_rx(0);
    // CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

// CAN2 not used for now.. chip shortage means don't populate on the board

// void CAN2_RX0_IRQ_Handler(void) {
//   while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
//     uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;
//     #ifdef DEBUG_CAN
//     puts("CAN3 RX: ");
//     puth(address);
//     puts("\n");
//     #else
//     UNUSED(address);
//     #endif

//     // next
//     can_rx(1);
//   }
// }

// void CAN2_SCE_IRQ_Handler(void) {
//   state = FAULT_SCE;
//   can_sce(CAN2);
//   llcan_clear_send(CAN2);
// }

void CAN3_RX0_IRQ_Handler(void) {
  // From the DSU
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN3->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN3->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN3->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN3->sFIFOMailBox[0].RDHR;

    uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;
    
    #ifdef DEBUG_CAN
    puts("CAN3 RX: ");
    puth(address);
    puts("\n");
    #endif
    
    // CAN data buffer
    uint8_t dat[8];

    switch (address) {
      case DSU_ACC_CONTROL: // ACC_CTRL
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN3->sFIFOMailBox[0], i);
        }
        if(dat[7] == toyota_checksum(address, dat, 8)) {
          // add permit_braking and recompute the checksum
          dat[2] &= 0x3F; // mask off the top 2 bits
          dat[2] |= (1 << 6U); // SET_ME_X01
          dat[3] |= (1 << 6U); // permit_braking
          if (enable_acc){ 
            // modify this before sending to the car only if requested
            dat[0] = (acc_cmd >> 8U);
            dat[1] = (acc_cmd & 0xFF);
            dat[2] |= ((mini_car << 5U) | (distance << 4U));
            dat[3] |= (release_standstill << 7U) | (acc_cancel << 0U);
          }
          dat[7] = toyota_checksum(address, dat, 8);  // do the checksum after modding data
          to_fwd.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
          to_fwd.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
          // reset the timer for seeing the DSU
          timeout = 0;
          state = NO_FAULT;
        } else {
          // bad checksum
          state = FAULT_BAD_CHECKSUM;
          #ifdef DEBUG_CAN
            for(int ii = 0; ii<8; ii++){ 
              puth2(dat[ii]);
            }
            puts("\n expected: ");
            puth2(toyota_checksum(address, dat, 8));
            puts(" got: ");
            puth2(dat[7]);
            puts("\n");
          #endif
        }
        break;
      case DSU_AEB_CMD: // AEB BRAKING
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN3->sFIFOMailBox[0], i);
        }
        uint16_t stock_aeb = ((dat[0] << 8U) | dat[1]) >> 6U;
        stock_aeb_active = (stock_aeb != 0);

        if(dat[7] == toyota_checksum(address, dat, 8)) {
          if (enable_aeb_control & !stock_aeb_active){ 
            // modify this message before sending to the car only if requested and stock AEB is NOT active
            dat[0] = (aeb_cmd >> 2U); // 10 bit msg
            dat[1] = ((aeb_cmd & 3U) << 6U) | ((ds1stat2 & 7U) << 3U) | ((ds1stbk2 & 7U) << 0U);
            dat[2] = (pcswar << 2) | (pcsalm << 1) | pcsopr;
            dat[3] = (pcsabk << 7) | ((PBATRGR & 3U) << 5) | (pptrgr << 4U) | (ibtrgr << 3) | (clextrgr << 2) | (irlt_req & 3U); 
            dat[4] = (brkhld << 5U) | (AVSTRGR << 4) | (VGRSTRGR << 2) | (prefill << 1U) | pbrtrgr;
            dat[7] = toyota_checksum(address, dat, 8);
            
            to_fwd.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
            to_fwd.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
          }
        } else {
          // bad checksum
          state = FAULT_BAD_CHECKSUM;
          #ifdef DEBUG_CAN
            for(int ii = 0; ii<8; ii++){ 
              puth2(dat[ii]);
            }
            puts("\n expected: ");
            puth2(toyota_checksum(address, dat, 8));
            puts(" got: ");
            puth2(dat[7]);
            puts("\n");
          #endif
        }
        break;
      case DSU_PRECOLLISION:
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN3->sFIFOMailBox[0], i);
        }
        // bool stock_pcs_active = ((dat[5] >> 1) & 1U);
        // if (enable_aeb_control & !stock_pcs_active){ 

        //   DSS1STAT = ds1stat2;
        //   DSS1STBK = 0;
        //   PPTRGR2 = 0;
        //   PBRTRGR2 = 0;
        //   PCSABK2 = 0;

        //   pcs_force = -100;

        //   //todo - check this
        //   pcs_active = 0;

        //   dat[0] = pcs_counter;
        //   dat[1] = (DSLCCW1 << 7) | (DSSTPBZ << 6) | (PBRTRGR2 << 4) | (DSSFTRQD & 7U);
        //   dat[2] = (pcs_force >> 8);
        //   dat[3] = pcs_force & 0xFF;
        //   dat[4] = (DSS1STBK << 5) | (DSS1STAT << 2) | (DSBHOK << 1) | PPTRGR2;
        //   dat[5] = ((DSRBQH << 6) & 3U) | (IBTRGR2 << 5) | (PCSABK2 << 4) | (pcs_active << 1);
        //   dat[6] = toyota_checksum(address, dat, 7);
          
        //   // load into the buffer
        //   to_fwd.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
        //   to_fwd.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);

        //   // increase the PCS counter
        //   pcs_counter += 1;
        //   pcs_counter &= 0xFF;
        // }
        break;
      default:
        // FWD as-is
        break;
    }
    // send to CAN1
    can_send(&to_fwd, 0, false);
    // next
    can_rx(2);
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

void TIM3_IRQ_Handler(void) {

  //send to EON. cap this to 50Hz
  if (send){
    if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
      uint8_t dat[4];
      dat[0] = 0;
      dat[1] = ctrl_mode;
      dat[2] = ((state & 0xFU) << 4) | (counter);
      dat[3] = toyota_checksum(0x2FF, dat, 4);

      CAN_FIFOMailBox_TypeDef to_send;
      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDTR = 4;
      to_send.RIR = (0x2FF << 21) | 1U;
      can_send(&to_send, 0, false);

      counter += 1;
      counter &= COUNTER_CYCLE;
    }
    else {
      // old can packet hasn't sent!
      #ifdef DEBUG_CAN
        puts("CAN1 MISS1\n");
      #endif
    }
  }

  send = !send;

  // up timeouts
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
  if (timeout_f1 == MAX_TIMEOUT2){
    enable_acc = 0;
    ctrl_mode &= 0xFE; // clear ACC ctrl mode bit
    acc_cmd = 0;
    puts("F1 TIMEOUT");
  } else {
    timeout_f1 += 1U;
  }
  if (timeout_f2 == MAX_TIMEOUT2){
    enable_aeb_control = 0;
    ctrl_mode &= 0xFD; // clear AEB ctrl mode bit
    aeb_cmd = 0;
    pcs_force = 0;
    pcs_active = 0;
    pcs_counter = 0;
    puts("F2 TIMEOUT");
  } else {
    timeout_f2 += 1U;
  }

  TIM3->SR = 0;

#ifdef DEBUG_CTRL
puts("enable_acc: ");
puth2(enable_acc);
puts("\nenable_aeb_control: ");
puth2(enable_aeb_control);
puts("\nacc_cmd: ");
puth(acc_cmd);
puts("\naeb_cmd: ");
puth(aeb_cmd);
puts("\nstate: ");
puth2(state);
puts(" ctrl_mode: ");
puth2(ctrl_mode);
puts("\n");
#endif
}

// ***************************** main code *****************************


void gw(void) {
  // read/write
  // maybe we can implement the ADC and DAC here for pedal functionality?
  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  // REGISTER_INTERRUPT(CAN2_TX_IRQn, CAN2_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  // REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  // REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN3_TX_IRQn, CAN3_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_SCE_IRQn, CAN3_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init microsecond system timer
  // increments 1000000 times per second
  // generate an update to set the prescaler
  TIM2->PSC = 48-1;
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->EGR = TIM_EGR_UG;
  // use TIM2->CNT to read

  // init board
  current_board->init();
  // enable USB
  #ifdef TGW_USB
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  usb_init();
  #endif

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan1 speed");
  }
  // llcan_speed_set = llcan_set_speed(CAN2, 5000, false, false);
  // if (!llcan_speed_set) {
  //   puts("Failed to set llcan2 speed");
  // }
  llcan_speed_set = llcan_set_speed(CAN3, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan3 speed");
  }

  bool ret = llcan_init(CAN1);
  // ret = llcan_init(CAN2);
  ret = llcan_init(CAN3);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);

  // TODO: figure out a function for these GPIOs
  // set_gpio_mode(GPIOB, 12, MODE_OUTPUT);
  // set_gpio_output_type(GPIOB, 12, OUTPUT_TYPE_PUSH_PULL);
  // set_gpio_output(GPIOB, 12, 1);

  // set_gpio_mode(GPIOB, 13, MODE_OUTPUT);
  // set_gpio_output_type(GPIOB, 13, OUTPUT_TYPE_PUSH_PULL);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    gw();
  }

  return 0;
}
