typedef struct {
  // const params
  CanMsg msg;
  const int fwd_to_bus;              //bus to forward this message
  const uint32_t expected_timestep;  //expected time between message updates [us]
  const uint32_t counter_mask_H;     // high part of the counter mask
  const uint32_t counter_mask_L;     // low part of the message mask
  // dynamic flags
  uint32_t dataH;                    //high data part of the message
  uint32_t dataL;                    //low data part of the message
  uint32_t last_received_timestamp;  //timestamp when last modded data was received [us]
  uint32_t last_fwd_timestamp;       //timestamp when last fwd occured [us]
  bool is_valid;                     //is the message valid ?
} CanMsgFwd;

bool fwd_check_valid(CANPacket_t *to_push, 
                uint32_t mask_H, uint32_t mask_L) {
    //we check for now only if the counter set in Comma2 is > 0
    return (((uint32_t)((GET_BYTES_48(to_push) & mask_H) + (GET_BYTES_04(to_push) & mask_L)) > 0) || (mask_H + mask_L == 0));
}

int get_fwd_addr_check_index(CANPacket_t *to_fwd, 
    CanMsgFwd addr_list[], const int len, bool is_data) {
  int bus = GET_BUS(to_fwd);
  int addr = GET_ADDR(to_fwd);
  int length = GET_LEN(to_fwd);

  int index = -1;
  for (int i = 0; i < len; i++) {
    if ((addr == addr_list[i].msg.addr)  && (length == addr_list[i].msg.len) && 
     (((bus == addr_list[i].msg.bus) && (!is_data)) || ((bus == addr_list[i].fwd_to_bus) && (is_data)))) {
      index = i;
      break;
    }
  }
  return index;
}

int get_addr_index(int addr, int bus, CanMsgFwd addr_list[], const int len, bool is_data) {
  int index = -1;
  for (int i = 0; i < len; i++) {
    if ((addr == addr_list[i].msg.addr)  && 
     (((bus == addr_list[i].msg.bus) && (!is_data)) || ((bus == addr_list[i].fwd_to_bus) && (is_data)))) {
      index = i;
      break;
    }
  }
  return index;
}

void update_received_time(CanMsgFwd addr_list[], int index) {
    //this should never happen but let's check anyway
    if (index == -1) {
        return;
    }
    uint32_t ts = TIM2->CNT;
    addr_list[index].last_received_timestamp = ts;
}

bool check_fwd_time_validity(CanMsgFwd addr_list[], int index) {
    //this should never happen but let's check anyway
    if (index == -1) {
        return false;
    }
    uint32_t ts = TIM2->CNT;
    //check if we receied data since last forward, and the data was received in the correct timeframe
    bool time_is_valid = ((addr_list[index].last_received_timestamp - addr_list[index].last_fwd_timestamp> 0) &&
           (ts - addr_list[index].last_received_timestamp <= addr_list[index].expected_timestep));
    addr_list[index].last_fwd_timestamp = ts;
    return time_is_valid;
}

//if message is to be forwarded not sent, process and return true
//otherwise return false 
bool fwd_data_message(CANPacket_t *to_push,
                       CanMsgFwd *fwd_msg_def,
                       const int fwd_msg_def_len,
                       bool violation
                       ) {
    int index = get_fwd_addr_check_index(to_push, fwd_msg_def, fwd_msg_def_len,true);
    if (index == -1) {
        return false;
    }

    //update timestamp
    update_received_time(fwd_msg_def,index);
    if ((!violation) && fwd_check_valid(to_push,fwd_msg_def[index].counter_mask_H,fwd_msg_def[index].counter_mask_L)) {
        fwd_msg_def[index].dataH = GET_BYTES_48(to_push) & (fwd_msg_def[index].counter_mask_H ^ 0xFFFFFFFF);
        fwd_msg_def[index].dataL = GET_BYTES_04(to_push) & (fwd_msg_def[index].counter_mask_L ^ 0xFFFFFFFF);
        fwd_msg_def[index].is_valid = true; 
    } else {
        fwd_msg_def[index].dataH = 0;
        fwd_msg_def[index].dataL = 0;
        fwd_msg_def[index].is_valid = false;
    }

    //return true as the addr is in the forward list
    return true;
}

//if message not found return -2
//otherwise return the values set to where to forward
//only modify if timestamp and all other checks are valid
int fwd_modded_message(CANPacket_t *to_fwd,
                       CanMsgFwd *fwd_msg_def,
                       const int fwd_msg_def_len,
                       bool (*compute_fwd_should_mod)(CANPacket_t *to_fwd),
                       bool (*compute_fwd_checksum)(CANPacket_t *to_fwd)) {
    int index = get_fwd_addr_check_index(to_fwd, fwd_msg_def, fwd_msg_def_len,false);
    if (index == -1) {
        return -2;
    }

    //check if we received in the allocated time slot
    if (!check_fwd_time_validity(fwd_msg_def,index)) {
        return fwd_msg_def[index].fwd_to_bus;
    }

    //is the data valid to process?
    if (!fwd_msg_def[index].is_valid) {
        return fwd_msg_def[index].fwd_to_bus;
    }

    //check if we should mod at this time
    if (compute_fwd_should_mod != NULL) {
        if (!compute_fwd_should_mod(to_fwd)) {
            //we should not modify it
            return fwd_msg_def[index].fwd_to_bus;
        }
    }

    //start preparing the message mod
    //save old data
    uint32_t dataH = GET_BYTES_48(to_fwd);
    uint32_t dataL = GET_BYTES_04(to_fwd);

    //transfer counter
    WORD_TO_BYTE_ARRAY(&to_fwd->data[4],fwd_msg_def[index].dataH | (dataH & fwd_msg_def[index].counter_mask_H));
    WORD_TO_BYTE_ARRAY(&to_fwd->data[0],fwd_msg_def[index].dataL | (dataL & fwd_msg_def[index].counter_mask_L));
    
    //compute checksum
    if (compute_fwd_checksum != NULL) {
        if (!compute_fwd_checksum(to_fwd)) {
            //checksum function failed, revert
            WORD_TO_BYTE_ARRAY(&to_fwd->data[0], dataL);
            WORD_TO_BYTE_ARRAY(&to_fwd->data[4], dataH);
        }
    }

    //process
    return fwd_msg_def[index].fwd_to_bus;
}
