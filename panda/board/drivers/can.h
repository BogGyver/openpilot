// IRQs: CAN1_TX, CAN1_RX0, CAN1_SCE
//       CAN2_TX, CAN2_RX0, CAN2_SCE
//       CAN3_TX, CAN3_RX0, CAN3_SCE

typedef struct {
  uint32_t w_ptr;
  uint32_t r_ptr;
  uint32_t fifo_size;
  CAN_FIFOMailBox_TypeDef *elems;
} can_ring;

#define CAN_BUS_RET_FLAG 0x80
#define CAN_BUS_NUM_MASK 0x7F

#define BUS_MAX 4

extern int can_live, pending_can_live;

// must reinit after changing these
extern int can_loopback, can_silent;
extern uint32_t can_speed[];

void can_set_forwarding(int from, int to);

void can_init(uint8_t can_number);
void can_init_all();
void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number);
int can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem);

// end API

#define ALL_CAN_SILENT 0xFF
#define ALL_CAN_BUT_MAIN_SILENT 0xFE
#define ALL_CAN_LIVE 0

int can_live = 0, pending_can_live = 0, can_loopback = 0, can_silent = ALL_CAN_SILENT;

// ********************* instantiate queues *********************

#define can_buffer(x, size) \
  CAN_FIFOMailBox_TypeDef elems_##x[size]; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = size, .elems = (CAN_FIFOMailBox_TypeDef *)&elems_##x };

can_buffer(rx_q, 0x1000)
can_buffer(tx1_q, 0x100)
can_buffer(tx2_q, 0x100)
can_buffer(tx3_q, 0x100)
can_buffer(txgmlan_q, 0x100)
can_ring *can_queues[] = {&can_tx1_q, &can_tx2_q, &can_tx3_q, &can_txgmlan_q};

// ********************* interrupt safe queue *********************

int can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
  int ret = 0;

  enter_critical_section();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1) == q->fifo_size) q->r_ptr = 0;
    else q->r_ptr += 1;
    ret = 1;
  }
  exit_critical_section();

  return ret;
}

int can_push(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
  int ret = 0;
  uint32_t next_w_ptr;

  enter_critical_section();
  if ((q->w_ptr + 1) == q->fifo_size) next_w_ptr = 0;
  else next_w_ptr = q->w_ptr + 1;
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = 1;
  }
  exit_critical_section();
  if (ret == 0) puts("can_push failed!\n");
  return ret;
}

void can_clear(can_ring *q) {
  enter_critical_section();
  q->w_ptr = 0;
  q->r_ptr = 0;
  exit_critical_section();
}

// assign CAN numbering
// bus num: Can bus number on ODB connector. Sent to/from USB
//    Min: 0; Max: 127; Bit 7 marks message as receipt (bus 129 is receipt for but 1)
// cans: Look up MCU can interface from bus number
// can number: numeric lookup for MCU CAN interfaces (0 = CAN1, 1 = CAN2, etc);
// bus_lookup: Translates from 'can number' to 'bus number'.
// can_num_lookup: Translates from 'bus number' to 'can number'.
// can_forwarding: Given a bus num, lookup bus num to forward to. -1 means no forward.

int can_rx_cnt = 0;
int can_tx_cnt = 0;
int can_txd_cnt = 0;
int can_err_cnt = 0;

// Panda:       Bus 0=CAN1   Bus 1=CAN2   Bus 2=CAN3
CAN_TypeDef *cans[] = {CAN1, CAN2, CAN3};
uint8_t bus_lookup[] = {0,1,2};
uint8_t can_num_lookup[] = {0,1,2,-1};
int8_t can_forwarding[] = {-1,-1,-1,-1};
uint32_t can_speed[] = {5000, 5000, 5000, 333};
#define CAN_MAX 3

#define CANIF_FROM_CAN_NUM(num) (cans[num])
#define CAN_NUM_FROM_CANIF(CAN) (CAN==CAN1 ? 0 : (CAN==CAN2 ? 1 : 2))
#define CAN_NAME_FROM_CANIF(CAN) (CAN==CAN1 ? "CAN1" : (CAN==CAN2 ? "CAN2" : "CAN3"))
#define BUS_NUM_FROM_CAN_NUM(num) (bus_lookup[num])
#define CAN_NUM_FROM_BUS_NUM(num) (can_num_lookup[num])

void process_can(uint8_t can_number);

void can_set_speed(uint8_t can_number) {
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  while (true) {
    if (llcan_set_speed(CAN, can_speed[bus_number], can_loopback, can_silent & (1 << can_number))) {
      return;
    }

    puts("CAN init FAILED!!!!!\n");
    puth(can_number); puts(" ");
    puth(BUS_NUM_FROM_CAN_NUM(can_number)); puts("\n");
    return;
  }
}

void can_init(uint8_t can_number) {
  if (can_number == 0xff) return;

  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  set_can_enable(CAN, 1);
  can_set_speed(can_number);

  llcan_init(CAN);

  // in case there are queued up messages
  process_can(can_number);
}

void can_init_all() {
  for (int i=0; i < CAN_MAX; i++) {
    can_init(i);
  }
}

void can_set_gmlan(int bus) {
  if (bus == -1 || bus != can_num_lookup[3]) {
    // GMLAN OFF
    switch (can_num_lookup[3]) {
      case 1:
        puts("disable GMLAN on CAN2\n");
        set_can_mode(1, 0);
        bus_lookup[1] = 1;
        can_num_lookup[1] = 1;
        can_num_lookup[3] = -1;
        can_init(1);
        break;
      case 2:
        puts("disable GMLAN on CAN3\n");
        set_can_mode(2, 0);
        bus_lookup[2] = 2;
        can_num_lookup[2] = 2;
        can_num_lookup[3] = -1;
        can_init(2);
        break;
    }
  }

  if (bus == 1) {
    puts("GMLAN on CAN2\n");
    // GMLAN on CAN2
    set_can_mode(1, 1);
    bus_lookup[1] = 3;
    can_num_lookup[1] = -1;
    can_num_lookup[3] = 1;
    can_init(1);
  } else if (bus == 2) {
    puts("GMLAN on CAN3\n");
    // GMLAN on CAN3
    set_can_mode(2, 1);
    bus_lookup[2] = 3;
    can_num_lookup[2] = -1;
    can_num_lookup[3] = 2;
    can_init(2);
  }
}

// CAN error
void can_sce(CAN_TypeDef *CAN) {
  enter_critical_section();

  #ifdef DEBUG
    if (CAN==CAN1) puts("CAN1:  ");
    if (CAN==CAN2) puts("CAN2:  ");
    #ifdef CAN3
      if (CAN==CAN3) puts("CAN3:  ");
    #endif
    puts("MSR:");
    puth(CAN->MSR);
    puts(" TSR:");
    puth(CAN->TSR);
    puts(" RF0R:");
    puth(CAN->RF0R);
    puts(" RF1R:");
    puth(CAN->RF1R);
    puts(" ESR:");
    puth(CAN->ESR);
    puts("\n");
  #endif

  can_err_cnt += 1;
  llcan_clear_send(CAN);
  exit_critical_section();
}

// ***************************** CAN *****************************

void process_can(uint8_t can_number) {
  if (can_number == 0xff) return;

  enter_critical_section();

  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
  #ifdef DEBUG
    puts("process CAN TX\n");
  #endif

  // check for empty mailbox
  CAN_FIFOMailBox_TypeDef to_send;
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    // add successfully transmitted message to my fifo
    if ((CAN->TSR & CAN_TSR_RQCP0) == CAN_TSR_RQCP0) {
      can_txd_cnt += 1;

      if ((CAN->TSR & CAN_TSR_TXOK0) == CAN_TSR_TXOK0) {
        CAN_FIFOMailBox_TypeDef to_push;
        to_push.RIR = CAN->sTxMailBox[0].TIR;
        to_push.RDTR = (CAN->sTxMailBox[0].TDTR & 0xFFFF000F) | ((CAN_BUS_RET_FLAG | bus_number) << 4);
        to_push.RDLR = CAN->sTxMailBox[0].TDLR;
        to_push.RDHR = CAN->sTxMailBox[0].TDHR;
        can_push(&can_rx_q, &to_push);
      }

      if ((CAN->TSR & CAN_TSR_TERR0) == CAN_TSR_TERR0) {
        #ifdef DEBUG
          puts("CAN TX ERROR!\n");
        #endif
      }

      if ((CAN->TSR & CAN_TSR_ALST0) == CAN_TSR_ALST0) {
        #ifdef DEBUG
          puts("CAN TX ARBITRATION LOST!\n");
        #endif
      }

      // clear interrupt
      // careful, this can also be cleared by requesting a transmission
      CAN->TSR |= CAN_TSR_RQCP0;
    }

    if (can_pop(can_queues[bus_number], &to_send)) {
      if (CAN->MCR & CAN_MCR_SLEEP) {
        set_can_enable(CAN, 1);
        CAN->MCR &= ~(CAN_MCR_SLEEP);
        CAN->MCR |= CAN_MCR_INRQ;
        while((CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);
        CAN->MCR &= ~(CAN_MCR_INRQ);
      }
      can_tx_cnt += 1;
      // only send if we have received a packet
      CAN->sTxMailBox[0].TDLR = to_send.RDLR;
      CAN->sTxMailBox[0].TDHR = to_send.RDHR;
      CAN->sTxMailBox[0].TDTR = to_send.RDTR;
      CAN->sTxMailBox[0].TIR = to_send.RIR;
    }
  }

  exit_critical_section();
}

// CAN receive handlers
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
  while (CAN->RF0R & CAN_RF0R_FMP0) {
    can_rx_cnt += 1;

    // can is live
    pending_can_live = 1;

    // add to my fifo
    CAN_FIFOMailBox_TypeDef to_push;
    to_push.RIR = CAN->sFIFOMailBox[0].RIR;
    to_push.RDTR = CAN->sFIFOMailBox[0].RDTR;
    to_push.RDLR = CAN->sFIFOMailBox[0].RDLR;
    to_push.RDHR = CAN->sFIFOMailBox[0].RDHR;

    // modify RDTR for our API
    to_push.RDTR = (to_push.RDTR & 0xFFFF000F) | (bus_number << 4);

    // forwarding (panda only)
    int bus_fwd_num = can_forwarding[bus_number] != -1 ? can_forwarding[bus_number] : safety_fwd_hook(bus_number, &to_push);
    if (bus_fwd_num != -1) {
      CAN_FIFOMailBox_TypeDef to_send;
      to_send.RIR = to_push.RIR | 1; // TXRQ
      to_send.RDTR = to_push.RDTR;
      to_send.RDLR = to_push.RDLR;
      to_send.RDHR = to_push.RDHR;
      can_send(&to_send, bus_fwd_num);
    }

    safety_rx_hook(&to_push);

    set_led(LED_BLUE, 1);
    can_push(&can_rx_q, &to_push);

    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_TX_IRQHandler() { process_can(0); }
void CAN1_RX0_IRQHandler() { can_rx(0); }
void CAN1_SCE_IRQHandler() { can_sce(CAN1); }

void CAN2_TX_IRQHandler() { process_can(1); }
void CAN2_RX0_IRQHandler() { can_rx(1); }
void CAN2_SCE_IRQHandler() { can_sce(CAN2); }

void CAN3_TX_IRQHandler() { process_can(2); }
void CAN3_RX0_IRQHandler() { can_rx(2); }
void CAN3_SCE_IRQHandler() { can_sce(CAN3); }

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number) {
  if (safety_tx_hook(to_push)) {
    if (bus_number < BUS_MAX) {
      // add CAN packet to send queue
      // bus number isn't passed through
      to_push->RDTR &= 0xF;
      if (bus_number == 3 && can_num_lookup[3] == 0xFF) {
        // TODO: why uint8 bro? only int8?
        bitbang_gmlan(to_push);
      } else {
        can_push(can_queues[bus_number], to_push);
        process_can(CAN_NUM_FROM_BUS_NUM(bus_number));
      }
    }
  }
}

void can_set_forwarding(int from, int to) {
  can_forwarding[from] = to;
}

