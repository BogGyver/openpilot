//#define EON 
//#define PANDA

// ********************* Includes *********************
#include "config.h"
#include "obj/gitversion.h"

#include "libc.h"
#include "provision.h"

#include "main_declarations.h"

#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/uart.h"
#include "drivers/usb.h"
#include "drivers/gmlan_alt.h"
#include "drivers/timer.h"
#include "drivers/clock.h"

#include "gpio.h"
#include "drivers/lin.h"

#ifndef EON
#include "drivers/spi.h"
#endif

#include "power_saving.h"
#include "safety.h"

#include "drivers/can.h"

// ********************* Serial debugging *********************

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv)) {
    (void)putc(ring, rcv);  // misra-c2012-17.7: cast to void is ok: debug function

    // jump to DFU flash
    if (rcv == 'z') {
      enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
      NVIC_SystemReset();
    }

    // normal reset
    if (rcv == 'x') {
      NVIC_SystemReset();
    }

    // enable CDP mode
    if (rcv == 'C') {
      puts("switching USB to CDP mode\n");
      current_board->set_usb_power_mode(USB_POWER_CDP);
    }
    if (rcv == 'c') {
      puts("switching USB to client mode\n");
      current_board->set_usb_power_mode(USB_POWER_CLIENT);
    }
    if (rcv == 'D') {
      puts("switching USB to DCP mode\n");
      current_board->set_usb_power_mode(USB_POWER_DCP);
    }
  }
}

// ***************************** started logic *****************************
void started_interrupt_handler(uint8_t interrupt_line) {
  volatile unsigned int pr = EXTI->PR & (1U << interrupt_line);
  if ((pr & (1U << interrupt_line)) != 0U) {
    #ifdef DEBUG
      puts("got started interrupt\n");
    #endif

    // jenky debounce
    delay(100000);

    // set power savings mode here
    int power_save_state = current_board->check_ignition() ? POWER_SAVE_STATUS_DISABLED : POWER_SAVE_STATUS_ENABLED;
    set_power_save_state(power_save_state);
  }
  EXTI->PR = (1U << interrupt_line);
}

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void EXTI0_IRQHandler(void) {
  started_interrupt_handler(0);
}

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void EXTI1_IRQHandler(void) {
  started_interrupt_handler(1);
}

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void EXTI3_IRQHandler(void) {
  started_interrupt_handler(3);
}

void started_interrupt_init(void) {
  SYSCFG->EXTICR[1] = SYSCFG_EXTICR1_EXTI1_PA;
  EXTI->IMR |= (1U << 1);
  EXTI->RTSR |= (1U << 1);
  EXTI->FTSR |= (1U << 1);
  NVIC_EnableIRQ(EXTI1_IRQn);
}

// ****************************** safety mode ******************************

// this is the only way to leave silent mode
void set_safety_mode(uint16_t mode, int16_t param) {
  int err = safety_set_mode(mode, param);
  if (err == -1) {
    puts("Error: safety set mode failed\n");
  } else {
    if (mode == SAFETY_NOOUTPUT) {
      can_silent = ALL_CAN_SILENT;
    } else {
      can_silent = ALL_CAN_LIVE;
    }

    switch (mode) {
        case SAFETY_NOOUTPUT:
          set_intercept_relay(false);
          if(hw_type == HW_TYPE_BLACK_PANDA){
            current_board->set_can_mode(CAN_MODE_NORMAL);
          }
          break;
        case SAFETY_ELM327:
          set_intercept_relay(false);
          if(hw_type == HW_TYPE_BLACK_PANDA){
            current_board->set_can_mode(CAN_MODE_OBD_CAN2);
          }
          break;
        default:
          set_intercept_relay(true);
          if(hw_type == HW_TYPE_BLACK_PANDA){
            current_board->set_can_mode(CAN_MODE_NORMAL);
          }
          break;
      }          
    if (safety_ignition_hook() != -1) {
      // if the ignition hook depends on something other than the started GPIO
      // we have to disable power savings (fix for GM and Tesla)
      set_power_save_state(POWER_SAVE_STATUS_DISABLED);
    } else {
      // power mode is already POWER_SAVE_STATUS_DISABLED and CAN TXs are active
    }
    can_init_all();
  }
}

// ***************************** USB port *****************************

int get_health_pkt(void *dat) {
  struct __attribute__((packed)) {
    uint32_t voltage_pkt;
    uint32_t current_pkt;
    uint32_t can_send_errs_pkt;
    uint32_t can_fwd_errs_pkt;
    uint32_t gmlan_send_errs_pkt;
    uint8_t started_pkt;
    uint8_t controls_allowed_pkt;
    uint8_t gas_interceptor_detected_pkt;
    uint8_t car_harness_status_pkt;
  } *health = dat;

  //Voltage will be measured in mv. 5000 = 5V
  uint32_t voltage = adc_get(ADCCHAN_VOLTAGE);

  // REVC has a 10, 1 (1/11) voltage divider
  // Here is the calculation for the scale (s)
  // ADCV = VIN_S * (1/11) * (4095/3.3)
  // RETVAL = ADCV * s = VIN_S*1000
  // s = 1000/((4095/3.3)*(1/11)) = 8.8623046875

  // Avoid needing floating point math
  health->voltage_pkt = (voltage * 8862U) / 1000U;

  // No current sense on panda black
  if(hw_type != HW_TYPE_BLACK_PANDA){
    health->current_pkt = adc_get(ADCCHAN_CURRENT);
  } else {
    health->current_pkt = 0;
  }

  int safety_ignition = safety_ignition_hook();
  if (safety_ignition < 0) {
    //Use the GPIO pin to determine ignition
    health->started_pkt = 1; // (uint8_t)(current_board->check_ignition());
  } else {
    //Current safety hooks want to determine ignition (ex: GM)
    health->started_pkt = safety_ignition;
  }

  health->controls_allowed_pkt = controls_allowed;
  health->gas_interceptor_detected_pkt = gas_interceptor_detected;
  health->can_send_errs_pkt = can_send_errs;
  health->can_fwd_errs_pkt = can_fwd_errs;
  health->gmlan_send_errs_pkt = gmlan_send_errs;
  health->car_harness_status_pkt = car_harness_status;
  
  return sizeof(*health);
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
    if ((usbdata8[0] < 2U) || safety_tx_lin_hook(usbdata8[0] - 2U, usbdata8 + 1, len - 1)) {
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
  UNUSED(hardwired);
  int dpkt = 0;
  uint32_t *d32 = (uint32_t *)usbdata;
  for (dpkt = 0; dpkt < (len / 4); dpkt += 4) {
    CAN_FIFOMailBox_TypeDef to_push;
    to_push.RDHR = d32[dpkt + 3];
    to_push.RDLR = d32[dpkt + 2];
    to_push.RDTR = d32[dpkt + 1];
    to_push.RIR = d32[dpkt];

    uint8_t bus_number = (to_push.RDTR >> 4) & CAN_BUS_NUM_MASK;
    can_send(&to_push, bus_number);
  }
}

void usb_cb_enumeration_complete() {
  puts("USB enumeration complete\n");
  is_enumerated = 1;
}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  int i;
  switch (setup->b.bRequest) {
    // **** 0xc0: get CAN debug info
    case 0xc0:
      puts("can tx: "); puth(can_tx_cnt);
      puts(" txd: "); puth(can_txd_cnt);
      puts(" rx: "); puth(can_rx_cnt);
      puts(" err: "); puth(can_err_cnt);
      puts("\n");
      break;
    // **** 0xc1: get hardware type
    case 0xc1:
      resp[0] = hw_type;
      resp_len = 1;
      break;
    // **** 0xd0: fetch serial number
    case 0xd0:
      // addresses are OTP
      if (setup->b.wValue.w == 1U) {
        (void)memcpy(resp, (uint8_t *)0x1fff79c0, 0x10);
        resp_len = 0x10;
      } else {
        get_provision_chunk(resp);
        resp_len = PROVISION_CHUNK_LEN;
      }
      break;
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      // so it's blocked over wifi
      switch (setup->b.wValue.w) {
        case 0:
          if (hardwired) {
            puts("-> entering bootloader\n");
            enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
            NVIC_SystemReset();
          }
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
    // **** 0xd2: get health packet
    case 0xd2:
      resp_len = get_health_pkt(resp);
      break;
    // **** 0xd6: get version
    case 0xd6:
      COMPILE_TIME_ASSERT(sizeof(gitversion) <= MAX_RESP_LEN);
      (void)memcpy(resp, gitversion, sizeof(gitversion));
      resp_len = sizeof(gitversion) - 1U;
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xd9: set ESP power
    case 0xd9:
      if (setup->b.wValue.w == 1U) {
        current_board->set_esp_gps_mode(ESP_GPS_ENABLED);
      } else if (setup->b.wValue.w == 2U) {
        current_board->set_esp_gps_mode(ESP_GPS_BOOTMODE);
      } else {
        current_board->set_esp_gps_mode(ESP_GPS_DISABLED);
      }
      break;
    // **** 0xda: reset ESP, with optional boot mode
    case 0xda:
      current_board->set_esp_gps_mode(ESP_GPS_DISABLED);
      delay(1000000);
      if (setup->b.wValue.w == 1U) {
        current_board->set_esp_gps_mode(ESP_GPS_BOOTMODE);
      } else {
        current_board->set_esp_gps_mode(ESP_GPS_ENABLED);
      }
      delay(1000000);
      current_board->set_esp_gps_mode(ESP_GPS_ENABLED);
      break;
    // **** 0xdb: set GMLAN (white/grey) or OBD CAN (black) multiplexing mode
    case 0xdb:
      if(hw_type == HW_TYPE_BLACK_PANDA){
        if (setup->b.wValue.w == 1U) {
          // Enable OBD CAN
          current_board->set_can_mode(CAN_MODE_OBD_CAN2);
        } else {
          // Disable OBD CAN
          current_board->set_can_mode(CAN_MODE_NORMAL);
        }        
      } else {
        if (setup->b.wValue.w == 1U) {
          // GMLAN ON
          if (setup->b.wIndex.w == 1U) {
            can_set_gmlan(1);
          } else if (setup->b.wIndex.w == 2U) {
            can_set_gmlan(2);
          } else {
            puts("Invalid bus num for GMLAN CAN set\n");
          }
        } else {
          can_set_gmlan(-1);
        }
      }
      break;
      
    // **** 0xdc: set safety mode
    case 0xdc:
      // Blocked over WiFi.
      // Allow NOOUTPUT and ELM security mode to be set over wifi.
      if (hardwired || (setup->b.wValue.w == SAFETY_NOOUTPUT) || (setup->b.wValue.w == SAFETY_ELM327)) {
        set_safety_mode(setup->b.wValue.w, (uint16_t) setup->b.wIndex.w);
      }
      break;
    // **** 0xdd: enable can forwarding
    case 0xdd:
      // wValue = Can Bus Num to forward from
      // wIndex = Can Bus Num to forward to
      if ((setup->b.wValue.w < BUS_MAX) && (setup->b.wIndex.w < BUS_MAX) &&
          (setup->b.wValue.w != setup->b.wIndex.w)) { // set forwarding
        can_set_forwarding(setup->b.wValue.w, setup->b.wIndex.w & CAN_BUS_NUM_MASK);
      } else if((setup->b.wValue.w < BUS_MAX) && (setup->b.wIndex.w == 0xFFU)){ //Clear Forwarding
        can_set_forwarding(setup->b.wValue.w, -1);
      } else {
        puts("Invalid CAN bus forwarding\n");
      }
      break;
    // **** 0xde: set can bitrate
    case 0xde:
      if (setup->b.wValue.w < BUS_MAX) {
        can_speed[setup->b.wValue.w] = setup->b.wIndex.w;
        can_init(CAN_NUM_FROM_BUS_NUM(setup->b.wValue.w));
      }
      break;
    // **** 0xdf: set long controls allowed
    case 0xdf:
      if (hardwired) {
        long_controls_allowed = setup->b.wValue.w & 1U;
      }
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      if (ur == &esp_ring) {
        uart_dma_drain();
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xe1: uart set baud rate
    case 0xe1:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      uart_set_baud(ur->uart, setup->b.wIndex.w);
      break;
    // **** 0xe2: uart set parity
    case 0xe2:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      switch (setup->b.wIndex.w) {
        case 0:
          // disable parity, 8-bit
          ur->uart->CR1 &= ~(USART_CR1_PCE | USART_CR1_M);
          break;
        case 1:
          // even parity, 9-bit
          ur->uart->CR1 &= ~USART_CR1_PS;
          ur->uart->CR1 |= USART_CR1_PCE | USART_CR1_M;
          break;
        case 2:
          // odd parity, 9-bit
          ur->uart->CR1 |= USART_CR1_PS;
          ur->uart->CR1 |= USART_CR1_PCE | USART_CR1_M;
          break;
        default:
          break;
      }
      break;
    // **** 0xe4: uart set baud rate extended
    case 0xe4:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      uart_set_baud(ur->uart, (int)setup->b.wIndex.w*300);
      break;
    // **** 0xe5: set CAN loopback (for testing)
    case 0xe5:
      can_loopback = (setup->b.wValue.w > 0U);
      can_init_all();
      break;
    // **** 0xe6: set USB power
    case 0xe6:
      if (setup->b.wValue.w == 1U) {
        puts("user setting CDP mode\n");
        current_board->set_usb_power_mode(USB_POWER_CDP);
      } else if (setup->b.wValue.w == 2U) {
        puts("user setting DCP mode\n");
        current_board->set_usb_power_mode(USB_POWER_DCP);
      } else {
        puts("user setting CLIENT mode\n");
        current_board->set_usb_power_mode(USB_POWER_CLIENT);
      }
      break;
    // **** 0xf0: do k-line wValue pulse on uart2 for Acura
    case 0xf0:
      if (setup->b.wValue.w == 1U) {
        GPIOC->ODR &= ~(1U << 10);
        GPIOC->MODER &= ~GPIO_MODER_MODER10_1;
        GPIOC->MODER |= GPIO_MODER_MODER10_0;
      } else {
        GPIOC->ODR &= ~(1U << 12);
        GPIOC->MODER &= ~GPIO_MODER_MODER12_1;
        GPIOC->MODER |= GPIO_MODER_MODER12_0;
      }

      for (i = 0; i < 80; i++) {
        delay(8000);
        if (setup->b.wValue.w == 1U) {
          GPIOC->ODR |= (1U << 10);
          GPIOC->ODR &= ~(1U << 10);
        } else {
          GPIOC->ODR |= (1U << 12);
          GPIOC->ODR &= ~(1U << 12);
        }
      }

      if (setup->b.wValue.w == 1U) {
        GPIOC->MODER &= ~GPIO_MODER_MODER10_0;
        GPIOC->MODER |= GPIO_MODER_MODER10_1;
      } else {
        GPIOC->MODER &= ~GPIO_MODER_MODER12_0;
        GPIOC->MODER |= GPIO_MODER_MODER12_1;
      }

      delay(140 * 9000);
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
    // **** 0xf3: Heartbeat. Resets heartbeat counter.
    case 0xf3:
      {
        heartbeat_counter = 0U;
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

#ifndef EON
int spi_cb_rx(uint8_t *data, int len, uint8_t *data_out) {
  // data[0]  = endpoint
  // data[2]  = length
  // data[4:] = data
  UNUSED(len);
  int resp_len = 0;
  switch (data[0]) {
    case 0:
      // control transfer
      resp_len = usb_cb_control_msg((USB_Setup_TypeDef *)(data+4), data_out, 0);
      break;
    case 1:
      // ep 1, read
      resp_len = usb_cb_ep1_in(data_out, 0x40, 0);
      break;
    case 2:
      // ep 2, send serial
      usb_cb_ep2_out(data+4, data[2], 0);
      break;
    case 3:
      // ep 3, send CAN
      usb_cb_ep3_out(data+4, data[2], 0);
      break;
    default:
      puts("SPI data invalid");
      break;
  }
  return resp_len;
}
#endif

// ***************************** main code *****************************

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}

uint64_t tcnt = 0;

// go into NOOUTPUT when the EON does not send a heartbeat for this amount of seconds.
#define EON_HEARTBEAT_THRESHOLD_IGNITION_ON 5U
#define EON_HEARTBEAT_THRESHOLD_IGNITION_OFF 2U

// called once per second
// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void TIM3_IRQHandler(void) {
  if (TIM3->SR != 0) {
    can_live = pending_can_live;

    current_board->usb_power_mode_tick(tcnt);

    //puth(usart1_dma); puts(" "); puth(DMA2_Stream5->M0AR); puts(" "); puth(DMA2_Stream5->NDTR); puts("\n");

    // reset this every 16th pass
    if ((tcnt & 0xFU) == 0U) {
      pending_can_live = 0;
    }
    #ifdef DEBUG
      //TODO: re-enable
      //puts("** blink ");
      //puth(can_rx_q.r_ptr); puts(" "); puth(can_rx_q.w_ptr); puts("  ");
      //puth(can_tx1_q.r_ptr); puts(" "); puth(can_tx1_q.w_ptr); puts("  ");
      //puth(can_tx2_q.r_ptr); puts(" "); puth(can_tx2_q.w_ptr); puts("\n");
    #endif

    // set green LED to be controls allowed
    current_board->set_led(LED_GREEN, controls_allowed);

    // turn off the blue LED, turned on by CAN
    // unless we are in power saving mode
    current_board->set_led(LED_BLUE, (tcnt & 1U) && (power_save_status == POWER_SAVE_STATUS_ENABLED));

    // increase heartbeat counter and cap it at the uint32 limit
    if (heartbeat_counter < __UINT32_MAX__) {
      heartbeat_counter += 1U;
    }

    // check heartbeat counter if we are running EON code. If the heartbeat has been gone for a while, go to NOOUTPUT safety mode.
    //BB we do not want to disable safety mode when on tesla
    /*
    #ifdef EON
    if (heartbeat_counter >= (current_board->check_ignition() ? EON_HEARTBEAT_THRESHOLD_IGNITION_ON : EON_HEARTBEAT_THRESHOLD_IGNITION_OFF)) {
      puts("EON hasn't sent a heartbeat for 0x"); puth(heartbeat_counter); puts(" seconds. Safety is set to NOOUTPUT mode.\n");
      set_safety_mode(SAFETY_NOOUTPUT, 0U);
    }
    #endif
    */
    // on to the next one
    tcnt += 1U;
  }
  TIM3->SR = 0;
}

int main(void) {
  // shouldn't have interrupts here, but just in case
  __disable_irq();

  // init early devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();
  adc_init();
  
  // print hello
  puts("\n\n\n************************ MAIN START ************************\n");

  // check for non-supported board types
  if(hw_type == HW_TYPE_UNKNOWN){
    puts("Unsupported board type\n");
    while (1) { /* hang */ }
  }

  puts("Config:\n");
  puts("  Board type: "); puts(current_board->board_type); puts("\n");
  puts(has_external_debug_serial ? "  Real serial\n" : "  USB serial\n");
  puts(is_entering_bootmode ? "  ESP wants bootmode\n" : "  No bootmode\n");

  // init board
  current_board->init();

  // panda has an FPU, let's use it!
  enable_fpu();

  // enable main uart if it's connected
  if (has_external_debug_serial) {
    // WEIRDNESS: without this gate around the UART, it would "crash", but only if the ESP is enabled
    // assuming it's because the lines were left floating and spurious noise was on them
    uart_init(USART2, 115200);
  }

  if (board_has_gps()) {
    uart_init(USART1, 9600);
  } else {
    // enable ESP uart
    uart_init(USART1, 115200);
  }

  // there is no LIN on panda black
  if(hw_type != HW_TYPE_BLACK_PANDA){
    // enable LIN
    uart_init(UART5, 10400);
    UART5->CR2 |= USART_CR2_LINEN;
    uart_init(USART3, 10400);
    USART3->CR2 |= USART_CR2_LINEN;
  }

  // init microsecond system timer
  // increments 1000000 times per second
  // generate an update to set the prescaler
  TIM2->PSC = 48-1;
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->EGR = TIM_EGR_UG;
  // use TIM2->CNT to read

  // default to silent mode to prevent issues with Ford
  // hardcode a specific safety mode if you want to force the panda to be in a specific mode
  int err = safety_set_mode(SAFETY_NOOUTPUT, 0);
  if (err == -1) {
    puts("Failed to set safety mode\n");
    while (true) {
      // if SAFETY_NOOUTPUT isn't succesfully set, we can't continue
    }
  }
  can_silent = ALL_CAN_SILENT;
  can_init_all();

#ifndef EON
  spi_init();
#endif

#ifdef EON
  // have to save power
  if (hw_type == HW_TYPE_WHITE_PANDA) {
    current_board->set_esp_gps_mode(ESP_GPS_DISABLED);
  }
  // only enter power save after the first cycle
  /*if (current_board->check_ignition()) {
    set_power_save_state(POWER_SAVE_STATUS_ENABLED);
  }*/

  if (hw_type != HW_TYPE_BLACK_PANDA) {
    // interrupt on started line
    started_interrupt_init();
  }
#endif

  // 48mhz / 65536 ~= 732 / 732 = 1
  timer_init(TIM3, 732);
  NVIC_EnableIRQ(TIM3_IRQn);

#ifdef DEBUG
  puts("DEBUG ENABLED\n");
#endif
  // enable USB (right before interrupts or enum can fail!)
  usb_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // LED should keep on blinking all the time
  uint64_t cnt = 0;

  for (cnt=0;;cnt++) {
    if (power_save_status == POWER_SAVE_STATUS_DISABLED) {
      int div_mode = ((usb_power_mode == USB_POWER_DCP) ? 4 : 1);

      // useful for debugging, fade breaks = panda is overloaded
      for (int div_mode_loop = 0; div_mode_loop < div_mode; div_mode_loop++) {
        for (int fade = 0; fade < 1024; fade += 8) {
          for (int i = 0; i < (128/div_mode); i++) {
            current_board->set_led(LED_RED, 1);
            if (fade < 512) { delay(fade); } else { delay(1024-fade); }
            current_board->set_led(LED_RED, 0);
            if (fade < 512) { delay(512-fade); } else { delay(fade-512); }
          }
        }
      }
    } else {
      __WFI();
    }
  }

  return 0;
}
