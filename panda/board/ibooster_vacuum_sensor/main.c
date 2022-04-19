// ********************* Includes *********************
//#define IVS_USB
#include "../config.h"

#include "early_init.h"
#include "crc.h"

#define CAN CAN1
#define ADCCHAN_VACUUM_PUMP 10

#ifdef IVS_USB
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

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeefU
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early_initialization();
}

// ********************* serial debugging *********************

#ifdef IVS_USB

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len) {
  UNUSED(usbdata);
  UNUSED(len);
  return 0;
}
void usb_cb_ep2_out(void *usbdata, int len) {
  UNUSED(usbdata);
  UNUSED(len);
}
void usb_cb_ep3_out(void *usbdata, int len) {
  UNUSED(usbdata);
  UNUSED(len);
}
void usb_cb_ep3_out_complete(void) {}
void usb_cb_enumeration_complete(void) {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp) {
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xc1: get hardware type
    case 0xc1:
      resp[0] = hw_type;
      resp_len = 1;
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, USBPACKET_MAX_SIZE)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** ivs can checksum *****************************
uint8_t ivs_checksum(uint8_t *dat, int len, int addr) {
  int i;
  uint8_t s = 0;
  s += ((addr)&0xFF) + ((addr>>8)&0xFF);
  for (i = 0; i < len; i++) {
    s = (s + dat[i]) & 0xFF;
  }
  return s;
}

// ***************************** can port *****************************

// addresses to be used on CAN
#define CAN_IVS_INPUT  0x555U
#define CAN_IVS_OUTPUT 0x556U
#define CAN_IVS_SIZE 6
#define COUNTER_CYCLE 0xFU

#define TRIGGER_MSG_ID 0x318U //GTW_carState
#define TRIGGER_MSG_FREQ 10U //10 Hz

//avoid using floating points
#define INCRESE_IVS_PER_SECOND 65535U * 2U / 50U //0.2V per sec
#define DECREASE_IVS_PER_SECOND 65535U / 100U // 0.05V per sec
#define DECREASE_IVS_PER_SECOND_WHEN_BRAKING 65535U / 50U //0.1V per sec
#define MIN_IVS_VALUE  65535U * 17U / 50U  //1.7V
#define MAX_IVS_VALUE  65535U * 26U / 50U  //2.6V
#define MAX_IVS_VALUE_WHEN_BRAKING  65535U * 22U / 50U  //2.2V

#define COMPRESSOR_ON_THRESHOLD 50000U


//values used for logic and CAN messages
uint16_t ivs_sensor_value = 0;
uint8_t brake_pressed = 0;
uint8_t compressor_on = 0;
uint8_t ivs_ok = 1;
uint16_t vacuum_pump_state = 0;
unsigned int ivs_idx = 0;
int led_value = 0;

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

#define MAX_TIMEOUT 10U
uint32_t timeout = 0;
uint32_t current_index = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
uint8_t state = NO_FAULT;
const uint8_t crc_poly = 0xD5U;  // standard crc8

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    #ifdef DEBUG
      puts("CAN RX\n");
    #endif
    int address = CAN->sFIFOMailBox[0].RIR >> 21;
    if (address == CAN_IVS_INPUT) {
      // softloader entry
      if (GET_MAILBOX_BYTES_04(&CAN->sFIFOMailBox[0]) == 0xdeadface) {
        if (GET_MAILBOX_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        } else if (GET_MAILBOX_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x02b00b1e) {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        } else {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }

      // normal packet, for now do nothing
    }
    if (address == 0x20A) { //brake message, might use ibooster msg later
      if ((GET_MAILBOX_BYTE(&CAN->sFIFOMailBox[0],0) & 0x0C) >> 2 != 1) {
        brake_pressed = 1;
      } else {
        brake_pressed = 0;
      }
    }
    if (address == TRIGGER_MSG_ID) {//trigger message, let's do the logic
      //if compressor is on add the compressor increase
      if (compressor_on == 1) {
        ivs_sensor_value += (INCRESE_IVS_PER_SECOND / TRIGGER_MSG_FREQ);
      }
      //if brake is on decrease by the brake pressed
      if (brake_pressed == 1) {
        ivs_sensor_value -= (DECREASE_IVS_PER_SECOND_WHEN_BRAKING / TRIGGER_MSG_FREQ);
        if (ivs_sensor_value > MAX_IVS_VALUE_WHEN_BRAKING) {
          ivs_sensor_value = MAX_IVS_VALUE_WHEN_BRAKING;
        }
      } else {
        ivs_sensor_value -= (DECREASE_IVS_PER_SECOND / TRIGGER_MSG_FREQ);
        if (ivs_sensor_value > MAX_IVS_VALUE) {
          ivs_sensor_value = MAX_IVS_VALUE;
        }
      }
      if (ivs_sensor_value < MIN_IVS_VALUE) {
        ivs_sensor_value = MIN_IVS_VALUE;
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  llcan_clear_send(CAN);
  state = NO_FAULT;
}


void TIM3_IRQ_Handler(void) {
  #ifdef DEBUG
    puth(TIM3->CNT);
    puts(" ");
    puth(ivs_sensor_value);
    puts("\n");
  #endif

  // check timer for sending the user ivs and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    dat[0] = (ivs_sensor_value >> 0) & 0xFFU;
    dat[1] = (ivs_sensor_value >> 8) & 0xFFU;
    dat[2] = (vacuum_pump_state >> 0) & 0xFFU;
    dat[3] = (vacuum_pump_state >> 8) & 0xFFU;
    dat[4] = ((ivs_idx << 4) | (ivs_ok << 2) | (compressor_on << 1) | (brake_pressed)) & 0xFFU;
    dat[5] = ivs_checksum(dat, CAN_IVS_SIZE - 1, CAN_IVS_OUTPUT);
    CAN->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8);
    CAN->sTxMailBox[0].TDTR = 6;  // len of packet is 5
    CAN->sTxMailBox[0].TIR = (CAN_IVS_OUTPUT << 21) | 1U;
    ++ivs_idx;
    ivs_idx &= COUNTER_CYCLE;
  } else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      puts("CAN MISS\n");
    #endif
  }

  // blink the LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;

  TIM3->SR = 0;
}

// ***************************** main code *****************************

void ivs(void) {
  // read/write
  vacuum_pump_state = (adc_get(ADCCHAN_VACUUM_PUMP) >> 2);
  if (vacuum_pump_state >= COMPRESSOR_ON_THRESHOLD) {
    compressor_on = 1;
  } else {
    compressor_on = 0;
  }

  // write the ivs to the DAC
  dac_set(0, (ivs_sensor_value << 2));

  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  
  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_external_debug_serial();
  detect_board_type();

  // init board
  current_board->init();

#ifdef IVS_USB
  // enable USB
  usb_init();
#endif

  // ivs stuff
  dac_init();
  adc_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main ivs loop
  while (1) {
    ivs();
  }

  return 0;
}
