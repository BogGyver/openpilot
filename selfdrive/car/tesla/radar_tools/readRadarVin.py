#!/usr/bin/env python3
import time
import struct
from enum import IntEnum
from queue import Queue, Empty
import threading
from binascii import hexlify

DEBUG = False

class SERVICE_TYPE(IntEnum):
  DIAGNOSTIC_SESSION_CONTROL         = 0x10
  ECU_RESET                          = 0x11
  SECURITY_ACCESS                    = 0x27
  COMMUNICATION_CONTROL              = 0x28
  TESTER_PRESENT                     = 0x3E
  ACCESS_TIMING_PARAMETER            = 0x83
  SECURED_DATA_TRANSMISSION          = 0x84
  CONTROL_DTC_SETTING                = 0x85
  RESPONSE_ON_EVENT                  = 0x86
  LINK_CONTROL                       = 0x87
  READ_DATA_BY_IDENTIFIER            = 0x22
  READ_MEMORY_BY_ADDRESS             = 0x23
  READ_SCALING_DATA_BY_IDENTIFIER    = 0x24
  READ_DATA_BY_PERIODIC_IDENTIFIER   = 0x2A
  DYNAMICALLY_DEFINE_DATA_IDENTIFIER = 0x2C
  WRITE_DATA_BY_IDENTIFIER           = 0x2E
  WRITE_MEMORY_BY_ADDRESS            = 0x3D
  CLEAR_DIAGNOSTIC_INFORMATION       = 0x14
  READ_DTC_INFORMATION               = 0x19
  INPUT_OUTPUT_CONTROL_BY_IDENTIFIER = 0x2F
  ROUTINE_CONTROL                    = 0x31
  REQUEST_DOWNLOAD                   = 0x34
  REQUEST_UPLOAD                     = 0x35
  TRANSFER_DATA                      = 0x36
  REQUEST_TRANSFER_EXIT              = 0x37

_negative_response_codes = {
    0x00: 'positive response',
    0x10: 'general reject',
    0x11: 'service not supported',
    0x12: 'sub-function not supported',
    0x13: 'incorrect message length or invalid format',
    0x14: 'response too long',
    0x21: 'busy repeat request',
    0x22: 'conditions not correct',
    0x24: 'request sequence error',
    0x25: 'no response from subnet component',
    0x26: 'failure prevents execution of requested action',
    0x31: 'request out of range',
    0x33: 'security access denied',
    0x35: 'invalid key',
    0x36: 'exceed numebr of attempts',
    0x37: 'required time delay not expired',
    0x70: 'upload download not accepted',
    0x71: 'transfer data suspended',
    0x72: 'general programming failure',
    0x73: 'wrong block sequence counter',
    0x78: 'request correctly received - response pending',
    0x7e: 'sub-function not supported in active session',
    0x7f: 'service not supported in active session',
    0x81: 'rpm too high',
    0x82: 'rpm too low',
    0x83: 'engine is running',
    0x84: 'engine is not running',
    0x85: 'engine run time too low',
    0x86: 'temperature too high',
    0x87: 'temperature too low',
    0x88: 'vehicle speed too high',
    0x89: 'vehicle speed too low',
    0x8a: 'throttle/pedal too high',
    0x8b: 'throttle/pedal too low',
    0x8c: 'transmission not in neutral',
    0x8d: 'transmission not in gear',
    0x8f: 'brake switch(es) not closed',
    0x90: 'shifter lever not in park',
    0x91: 'torque converter clutch locked',
    0x92: 'voltage too high',
    0x93: 'voltage too low',
}

class MessageTimeoutError(Exception):
  pass

class NegativeResponseError(Exception):
  def __init__(self, message, service_id, error_code):
    super(Exception, self).__init__(message)
    self.service_id = service_id
    self.error_code = error_code

class InvalidServiceIdError(Exception):
  pass

class InvalidSubFunctioneError(Exception):
  pass

def _isotp_thread(panda, bus, tx_addr, tx_queue, rx_queue):
  try:
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    if tx_addr < 0xFFF8:
      filter_addr = tx_addr+0x10
    elif tx_addr > 0x10000000 and tx_addr < 0xFFFFFFFF:
      filter_addr = (tx_addr & 0xFFFF0000) + (tx_addr<<8 & 0xFF00) + (tx_addr>>8 & 0xFF)
    else:
      raise ValueError("invalid tx_addr: {}".format(tx_addr))
    rx_frame = {"size": 0, "data": "", "idx": 0, "done": True}
    tx_frame = {"size": 0, "data": "", "idx": 0, "done": True}

    # clear tx buffer
    panda.can_clear(bus)
    # clear rx buffer
    panda.can_clear(0xFFFF)
    time.sleep(1)
    while True:
      messages = panda.can_recv()
      for rx_addr, rx_ts, rx_data, rx_bus in messages:
        if rx_bus != bus or rx_addr != filter_addr or len(rx_data) == 0:
          continue
        rx_data = bytearray(rx_data)
        if (DEBUG): print("R: {} {}".format(hex(rx_addr), hexlify(rx_data)))
        if rx_data[0] >> 4 == 0x0:
          # single rx_frame
          rx_frame["size"] = rx_data[0] & 0xFF
          rx_frame["data"] = rx_data[1:1+rx_frame["size"]]
          rx_frame["idx"] = 0
          rx_frame["done"] = True
          rx_queue.put(rx_frame["data"])
        elif rx_data[0] >> 4 == 0x1:
          # first rx_frame
          rx_frame["size"] = ((rx_data[0] & 0x0F) << 8) + rx_data[1]
          rx_frame["data"] = rx_data[2:]
          rx_frame["idx"] = 0
          rx_frame["done"] = False
          # send flow control message (send all bytes)
          msg = "\x30\x00\x00".ljust(8, "\x00")
          if (DEBUG): print("S: {} {}".format(hex(tx_addr), hexlify(msg)))
          panda.can_send(tx_addr, msg.encode('utf-8'), bus)
        elif rx_data[0] >> 4 == 0x2:
          # consecutive rx frame
          assert rx_frame["done"] == False, "rx: no active frame"
          # validate frame index
          rx_frame["idx"] += 1
          assert rx_frame["idx"] & 0xF == rx_data[0] & 0xF, "rx: invalid consecutive frame index"
          rx_size = rx_frame["size"] - len(rx_frame["data"])
          rx_frame["data"] += rx_data[1:1+min(rx_size, 7)]
          if rx_frame["size"] == len(rx_frame["data"]):
            rx_frame["done"] = True
            rx_queue.put(rx_frame["data"])
        elif rx_data[0] >> 4 == 0x3:
          # flow control
          assert tx_frame["done"] == False, "tx: no active frame"
          # TODO: support non-zero block size and separate time
          assert rx_data[0] == 0x30 and rx_data[2] == 0x00, "tx: flow-control requires: continue, no delay"
          # first frame = 6 bytes, each consecutive frame = 7 bytes
          start = 6 + tx_frame["idx"] * 7
          count = rx_data[1]
          end = start + count * 7 if count > 0 else tx_frame["size"]
          for i in range(start, end, 7):
            tx_frame["idx"] += 1
            # consecutive tx frames
            msg = (chr(0x20 | (tx_frame["idx"] & 0xF)) + tx_frame["data"][i:i+7]).ljust(8, b'\0')
            if (DEBUG): print("S: {} {}".format(hex(tx_addr), hexlify(msg)))
            panda.can_send(tx_addr, msg.encode('utf-8'), bus)
          tx_frame["done"] = True

      if not tx_queue.empty():
        req = tx_queue.get(block=False)
        # reset rx and tx frames
        rx_frame = {"size": 0, "data": "", "idx": 0, "done": True}
        tx_frame = {"size": len(req), "data": req, "idx": 0, "done": False}
        if tx_frame["size"] < 8:
          # single frame
          tx_frame["done"] = True
          msg = (chr(tx_frame["size"]).encode("utf-8") + tx_frame["data"]).ljust(8, b'\0')
          if (DEBUG): print("S: {} {}".format(hex(tx_addr), hexlify(msg)))
          panda.can_send(tx_addr, msg, bus)
        else:
          # first rx_frame
          tx_frame["done"] = False
          msg = (struct.pack("!H", 0x1000 | tx_frame["size"]) + tx_frame["data"][:6]).ljust(8, b'\0')
          if (DEBUG): print("S: {} {}".format(hex(tx_addr), hexlify(msg)))
          panda.can_send(tx_addr, msg.encode('utf-8'), bus)
      else:
        time.sleep(0.01)
  finally:
    panda.close()

# generic uds request
def _uds_request(address, service_type, subfunction=None, data=None):
  req = chr(service_type).encode('utf-8')
  if subfunction is not None:
    req += chr(subfunction).encode('utf-8')
  if data is not None:
    req += data
  tx_queue.put(req)

  while True:
    try:
      resp = rx_queue.get(block=True, timeout=10)
    except Empty:
      print("** Timeout waiting for response. Make sure ignition line is on.")
      exit(1)
    resp_sid = resp[0] if len(resp) > 0 else None

    # negative response
    if resp_sid == 0x7F:
      service_id = resp[1] if len(resp) > 1 else -1
      try:
        service_desc = SERVICE_TYPE(service_id).name
      except Exception:
        service_desc = 'NON_STANDARD_SERVICE'
      error_code = resp[2] if len(resp) > 2 else -1
      try:
        error_desc = _negative_response_codes[error_code]
      except Exception:
        error_desc = 'unknown error'
      # wait for another message if response pending
      if error_code == 0x78:
        time.sleep(0.1)
        continue
      raise NegativeResponseError('{} - {}'.format(service_desc, error_desc), service_id, error_code)
    break

  # positive response
  if service_type+0x40 != resp_sid:
    resp_sid_hex = hex(resp_sid) if resp_sid is not None else None
    raise InvalidServiceIdError('invalid response service id: {}'.format(resp_sid_hex))

  if subfunction is not None:
    resp_sfn = resp[1] if len(resp) > 1 else None
    if subfunction != resp_sfn:
      resp_sfn_hex = hex(resp_sfn) if resp_sfn is not None else None
      raise InvalidSubFunctioneError('invalid response subfunction: {}'.format(hex(resp_sfn)))

  # return data (exclude service id and sub-function id)
  return resp[(1 if subfunction is None else 2):]

# services
class SESSION_TYPE(IntEnum):
  DEFAULT = 1
  PROGRAMMING = 2
  EXTENDED_DIAGNOSTIC = 3
  SAFETY_SYSTEM_DIAGNOSTIC = 4

def diagnostic_session_control(address, session_type):
  _uds_request(address, SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, subfunction=session_type)

class RESET_TYPE(IntEnum):
  HARD = 1
  KEY_OFF_ON = 2
  SOFT = 3
  ENABLE_RAPID_POWER_SHUTDOWN = 4
  DISABLE_RAPID_POWER_SHUTDOWN = 5

def ecu_reset(address, reset_type):
  resp = _uds_request(address, SERVICE_TYPE.ECU_RESET, subfunction=reset_type)
  power_down_time = None
  if reset_type == RESET_TYPE.ENABLE_RAPID_POWER_SHUTDOWN:
    power_down_time = resp[0]
    return power_down_time

class ACCESS_TYPE(IntEnum):
  REQUEST_SEED = 1
  SEND_KEY = 2

def security_access(address, access_type, security_key=None):
  request_seed = access_type % 2 != 0
  if request_seed and security_key is not None:
    raise ValueError('security_key not allowed')
  if not request_seed and security_key is None:
    raise ValueError('security_key is missing')
  resp = _uds_request(address, SERVICE_TYPE.SECURITY_ACCESS, subfunction=access_type, data=security_key)
  if request_seed:
    security_seed = resp
    return security_seed

class CONTROL_TYPE(IntEnum):
  ENABLE_RX_ENABLE_TX = 0
  ENABLE_RX_DISABLE_TX = 1
  DISABLE_RX_ENABLE_TX = 2
  DISABLE_RX_DISABLE_TX = 3

class MESSAGE_TYPE(IntEnum):
  NORMAL = 1
  NETWORK_MANAGEMENT = 2
  NORMAL_AND_NETWORK_MANAGEMENT = 3

def communication_control(address, control_type, message_type):
  data = chr(message_type)
  _uds_request(address, SERVICE_TYPE.COMMUNICATION_CONTROL, subfunction=control_type, data=data)

def tester_present(address):
  _uds_request(address, SERVICE_TYPE.TESTER_PRESENT, subfunction=0x00)

class TIMING_PARAMETER_TYPE(IntEnum):
  READ_EXTENDED_SET = 1
  SET_TO_DEFAULT_VALUES = 2
  READ_CURRENTLY_ACTIVE = 3
  SET_TO_GIVEN_VALUES = 4

def access_timing_parameter(address, timing_parameter_type, parameter_values):
  write_custom_values = timing_parameter_type == ACCESS_TIMING_PARAMETER_TYPE.SET_TO_GIVEN_VALUES
  read_values = (
    timing_parameter_type == ACCESS_TIMING_PARAMETER_TYPE.READ_CURRENTLY_ACTIVE or
    timing_parameter_type == ACCESS_TIMING_PARAMETER_TYPE.READ_EXTENDED_SET
  )
  if not write_custom_values and parameter_values is not None:
    raise ValueError('parameter_values not allowed')
  if write_custom_values and parameter_values is None:
    raise ValueError('parameter_values is missing')
  resp = _uds_request(address, SERVICE_TYPE.ACCESS_TIMING_PARAMETER, subfunction=timing_parameter_type, data=parameter_values)
  if read_values:
    # TODO: parse response into values?
    parameter_values = resp
    return parameter_values

def secured_data_transmission(address, data):
  # TODO: split data into multiple input parameters?
  resp = _uds_request(address, SERVICE_TYPE.SECURED_DATA_TRANSMISSION, subfunction=None, data=data)
  # TODO: parse response into multiple output values?
  return resp

class DTC_SETTING_TYPE(IntEnum):
  ON = 1
  OFF = 2

def control_dtc_setting(address, dtc_setting_type):
  _uds_request(address, SERVICE_TYPE.CONTROL_DTC_SETTING, subfunction=dtc_setting_type)

class RESPONSE_EVENT_TYPE(IntEnum):
  STOP_RESPONSE_ON_EVENT = 0
  ON_DTC_STATUS_CHANGE = 1
  ON_TIMER_INTERRUPT = 2
  ON_CHANGE_OF_DATA_IDENTIFIER = 3
  REPORT_ACTIVATED_EVENTS = 4
  START_RESPONSE_ON_EVENT = 5
  CLEAR_RESPONSE_ON_EVENT = 6
  ON_COMPARISON_OF_VALUES = 7

def response_on_event(address, response_event_type, store_event, window_time, event_type_record, service_response_record):
  if store_event:
    response_event_type |= 0x20
  # TODO: split record parameters into arrays
  data = char(window_time) + event_type_record + service_response_record
  resp = _uds_request(address, SERVICE_TYPE.RESPONSE_ON_EVENT, subfunction=response_event_type, data=data)

  if response_event_type == REPORT_ACTIVATED_EVENTS:
    return {
      "num_of_activated_events": resp[0],
      "data": resp[1:], # TODO: parse the reset of response
    }

  return {
    "num_of_identified_events": resp[0],
    "event_window_time": resp[1],
    "data": resp[2:], # TODO: parse the reset of response
  }

class LINK_CONTROL_TYPE(IntEnum):
  VERIFY_BAUDRATE_TRANSITION_WITH_FIXED_BAUDRATE = 1
  VERIFY_BAUDRATE_TRANSITION_WITH_SPECIFIC_BAUDRATE = 2
  TRANSITION_BAUDRATE = 3

class BAUD_RATE_TYPE(IntEnum):
  PC9600 = 1
  PC19200 = 2
  PC38400 = 3
  PC57600 = 4
  PC115200 = 5
  CAN125000 = 16
  CAN250000 = 17
  CAN500000 = 18
  CAN1000000 = 19

def link_control(address, link_control_type, baud_rate_type=None):
  if LINK_CONTROL_TYPE.VERIFY_BAUDRATE_TRANSITION_WITH_FIXED_BAUDRATE:
    # baud_rate_type = BAUD_RATE_TYPE
    data = chr(baud_rate_type)
  elif LINK_CONTROL_TYPE.VERIFY_BAUDRATE_TRANSITION_WITH_SPECIFIC_BAUDRATE:
    # baud_rate_type = custom value (3 bytes big-endian)
    data = struct.pack('!I', baud_rate_type)[1:]
  else:
    data = None
  _uds_request(address, SERVICE_TYPE.LINK_CONTROL, subfunction=link_control_type, data=data)

class DATA_IDENTIFIER_TYPE(IntEnum):
  BOOT_SOFTWARE_IDENTIFICATION = 0XF180
  APPLICATION_SOFTWARE_IDENTIFICATION = 0XF181
  APPLICATION_DATA_IDENTIFICATION = 0XF182
  BOOT_SOFTWARE_FINGERPRINT = 0XF183
  APPLICATION_SOFTWARE_FINGERPRINT = 0XF184
  APPLICATION_DATA_FINGERPRINT = 0XF185
  ACTIVE_DIAGNOSTIC_SESSION = 0XF186
  VEHICLE_MANUFACTURER_SPARE_PART_NUMBER = 0XF187
  VEHICLE_MANUFACTURER_ECU_SOFTWARE_NUMBER = 0XF188
  VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER = 0XF189
  SYSTEM_SUPPLIER_IDENTIFIER = 0XF18A
  ECU_MANUFACTURING_DATE = 0XF18B
  ECU_SERIAL_NUMBER = 0XF18C
  SUPPORTED_FUNCTIONAL_UNITS = 0XF18D
  VEHICLE_MANUFACTURER_KIT_ASSEMBLY_PART_NUMBER = 0XF18E
  VIN = 0XF190
  VEHICLE_MANUFACTURER_ECU_HARDWARE_NUMBER = 0XF191
  SYSTEM_SUPPLIER_ECU_HARDWARE_NUMBER = 0XF192
  SYSTEM_SUPPLIER_ECU_HARDWARE_VERSION_NUMBER = 0XF193
  SYSTEM_SUPPLIER_ECU_SOFTWARE_NUMBER = 0XF194
  SYSTEM_SUPPLIER_ECU_SOFTWARE_VERSION_NUMBER = 0XF195
  EXHAUST_REGULATION_OR_TYPE_APPROVAL_NUMBER = 0XF196
  SYSTEM_NAME_OR_ENGINE_TYPE = 0XF197
  REPAIR_SHOP_CODE_OR_TESTER_SERIAL_NUMBER = 0XF198
  PROGRAMMING_DATE = 0XF199
  CALIBRATION_REPAIR_SHOP_CODE_OR_CALIBRATION_EQUIPMENT_SERIAL_NUMBER = 0XF19A
  CALIBRATION_DATE = 0XF19B
  CALIBRATION_EQUIPMENT_SOFTWARE_NUMBER = 0XF19C
  ECU_INSTALLATION_DATE = 0XF19D
  ODX_FILE = 0XF19E
  ENTITY = 0XF19F

def read_data_by_identifier(address, data_identifier_type):
  # TODO: support list of identifiers
  data = struct.pack('!H', data_identifier_type)
  resp = _uds_request(address, SERVICE_TYPE.READ_DATA_BY_IDENTIFIER, subfunction=None, data=data)
  resp_id = struct.unpack('!H', resp[0:2])[0] if len(resp) >= 2 else None
  if resp_id != data_identifier_type:
    raise ValueError('invalid response data identifier: {}'.format(hex(resp_id)))
  return resp[2:]

def read_memory_by_address(address, memory_address, memory_size, memory_address_bytes=4, memory_size_bytes=1):
  if memory_address_bytes < 1 or memory_address_bytes > 4:
    raise ValueError('invalid memory_address_bytes: {}'.format(memory_address_bytes))
  if memory_size_bytes < 1 or memory_size_bytes > 4:
    raise ValueError('invalid memory_size_bytes: {}'.format(memory_size_bytes))
  data = chr(memory_size_bytes<<4 | memory_address_bytes)

  if memory_address >= 1<<(memory_address_bytes*8):
    raise ValueError('invalid memory_address: {}'.format(memory_address))
  data += struct.pack('!I', memory_address)[4-memory_address_bytes:]
  if memory_size >= 1<<(memory_size_bytes*8):
    raise ValueError('invalid memory_size: {}'.format(memory_size))
  data += struct.pack('!I', memory_size)[4-memory_size_bytes:]

  resp = _uds_request(address, SERVICE_TYPE.READ_MEMORY_BY_ADDRESS, subfunction=None, data=data)
  return resp

def read_scaling_data_by_identifier(address, data_identifier_type):
  data = struct.pack('!H', data_identifier_type)
  resp = _uds_request(address, SERVICE_TYPE.READ_SCALING_DATA_BY_IDENTIFIER, subfunction=None, data=data)
  resp_id = struct.unpack('!H', resp[0:2])[0] if len(resp) >= 2 else None
  if resp_id != data_identifier_type:
    raise ValueError('invalid response data identifier: {}'.format(hex(resp_id)))
  return resp[2:] # TODO: parse the response

class TRANSMISSION_MODE_TYPE(IntEnum):
  SEND_AT_SLOW_RATE = 1
  SEND_AT_MEDIUM_RATE = 2
  SEND_AT_FAST_RATE = 3
  STOP_SENDING = 4

def read_data_by_periodic_identifier(address, transmission_mode_type, periodic_data_identifier):
  # TODO: support list of identifiers
  data = chr(transmission_mode_type) + chr(periodic_data_identifier)
  _uds_request(address, SERVICE_TYPE.READ_DATA_BY_PERIODIC_IDENTIFIER, subfunction=None, data=data)

class DYNAMIC_DEFINITION_TYPE(IntEnum):
  DEFINE_BY_IDENTIFIER = 1
  DEFINE_BY_MEMORY_ADDRESS = 2
  CLEAR_DYNAMICALLY_DEFINED_DATA_IDENTIFIER = 3

def dynamically_define_data_identifier(address, dynamic_definition_type, dynamic_data_identifier, source_definitions, memory_address_bytes=4, memory_size_bytes=1):
  if memory_address_bytes < 1 or memory_address_bytes > 4:
    raise ValueError('invalid memory_address_bytes: {}'.format(memory_address_bytes))
  if memory_size_bytes < 1 or memory_size_bytes > 4:
    raise ValueError('invalid memory_size_bytes: {}'.format(memory_size_bytes))
  data = chr(memory_size_bytes<<4 | memory_address_bytes)

  data = struct.pack('!H', dynamic_data_identifier)
  if dynamic_definition_type == DYNAMIC_DEFINITION_TYPE.DEFINE_BY_IDENTIFIER:
    for s in source_definitions:
      data += struct.pack('!H', s["data_identifier"]) + chr(s["position"]) + chr(s["memory_size"])
  elif dynamic_definition_type == DYNAMIC_DEFINITION_TYPE.DEFINE_BY_MEMORY_ADDRESS:
    data += chr(memory_size_bytes<<4 | memory_address_bytes)
    for s in source_definitions:
      if s["memory_address"] >= 1<<(memory_address_bytes*8):
        raise ValueError('invalid memory_address: {}'.format(s["memory_address"]))
      data += struct.pack('!I', memory_address)[4-memory_address_bytes:]
      if s["memory_size"] >= 1<<(memory_size_bytes*8):
        raise ValueError('invalid memory_size: {}'.format(s["memory_size"]))
      data += struct.pack('!I', s["memory_size"])[4-memory_size_bytes:]
  elif dynamic_definition_type == DYNAMIC_DEFINITION_TYPE.CLEAR_DYNAMICALLY_DEFINED_DATA_IDENTIFIER:
    pass
  else:
    raise ValueError('invalid dynamic identifier type: {}'.format(hex(dynamic_definition_type)))
  _uds_request(address, SERVICE_TYPE.DYNAMICALLY_DEFINE_DATA_IDENTIFIER, subfunction=dynamic_definition_type, data=data)

def write_data_by_identifier(address, data_identifier_type, data_record):
  data = struct.pack('!H', data_identifier_type) + data_record
  resp = _uds_request(address, SERVICE_TYPE.WRITE_DATA_BY_IDENTIFIER, subfunction=None, data=data)
  resp_id = struct.unpack('!H', resp[0:2])[0] if len(resp) >= 2 else None
  if resp_id != data_identifier_type:
    raise ValueError('invalid response data identifier: {}'.format(hex(resp_id)))

def write_memory_by_address(address, memory_address, memory_size, data_record, memory_address_bytes=4, memory_size_bytes=1):
  if memory_address_bytes < 1 or memory_address_bytes > 4:
    raise ValueError('invalid memory_address_bytes: {}'.format(memory_address_bytes))
  if memory_size_bytes < 1 or memory_size_bytes > 4:
    raise ValueError('invalid memory_size_bytes: {}'.format(memory_size_bytes))
  data = chr(memory_size_bytes<<4 | memory_address_bytes)

  if memory_address >= 1<<(memory_address_bytes*8):
    raise ValueError('invalid memory_address: {}'.format(memory_address))
  data += struct.pack('!I', memory_address)[4-memory_address_bytes:]
  if memory_size >= 1<<(memory_size_bytes*8):
    raise ValueError('invalid memory_size: {}'.format(memory_size))
  data += struct.pack('!I', memory_size)[4-memory_size_bytes:]

  data += data_record
  _uds_request(address, SERVICE_TYPE.WRITE_MEMORY_BY_ADDRESS, subfunction=0x00, data=data)

class DTC_GROUP_TYPE(IntEnum):
  EMISSIONS = 0x000000
  ALL = 0xFFFFFF

def clear_diagnostic_information(address, dtc_group_type):
  data = struct.pack('!I', dtc_group_type)[1:] # 3 bytes
  _uds_request(address, SERVICE_TYPE.CLEAR_DIAGNOSTIC_INFORMATION, subfunction=None, data=data)

class DTC_REPORT_TYPE(IntEnum):
  NUMBER_OF_DTC_BY_STATUS_MASK = 0x01
  DTC_BY_STATUS_MASK = 0x02
  DTC_SNAPSHOT_IDENTIFICATION = 0x03
  DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER = 0x04
  DTC_SNAPSHOT_RECORD_BY_RECORD_NUMBER = 0x05
  DTC_EXTENDED_DATA_RECORD_BY_DTC_NUMBER = 0x06
  NUMBER_OF_DTC_BY_SEVERITY_MASK_RECORD = 0x07
  DTC_BY_SEVERITY_MASK_RECORD = 0x08
  SEVERITY_INFORMATION_OF_DTC = 0x09
  SUPPORTED_DTC = 0x0A
  FIRST_TEST_FAILED_DTC = 0x0B
  FIRST_CONFIRMED_DTC = 0x0C
  MOST_RECENT_TEST_FAILED_DTC = 0x0D
  MOST_RECENT_CONFIRMED_DTC = 0x0E
  MIRROR_MEMORY_DTC_BY_STATUS_MASK = 0x0F
  MIRROR_MEMORY_DTC_EXTENDED_DATA_RECORD_BY_DTC_NUMBER = 0x10
  NUMBER_OF_MIRROR_MEMORY_DTC_BY_STATUS_MASK = 0x11
  NUMBER_OF_EMISSIONS_RELATED_OBD_DTC_BY_STATUS_MASK = 0x12
  EMISSIONS_RELATED_OBD_DTC_BY_STATUS_MASK = 0x13
  DTC_FAULT_DETECTION_COUNTER = 0x14
  DTC_WITH_PERMANENT_STATUS = 0x15

class DTC_STATUS_MASK_TYPE(IntEnum):
  TEST_FAILED = 0x01
  TEST_FAILED_THIS_OPERATION_CYCLE = 0x02
  PENDING_DTC = 0x04
  CONFIRMED_DTC = 0x08
  TEST_NOT_COMPLETED_SINCE_LAST_CLEAR = 0x10
  TEST_FAILED_SINCE_LAST_CLEAR = 0x20
  TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE = 0x40
  WARNING_INDICATOR_uds_requestED = 0x80
  ALL = 0xFF

class DTC_SEVERITY_MASK_TYPE(IntEnum):
  MAINTENANCE_ONLY = 0x20
  CHECK_AT_NEXT_HALT = 0x40
  CHECK_IMMEDIATELY = 0x80
  ALL = 0xE0

def read_dtc_information(address, dtc_report_type, dtc_status_mask_type=DTC_STATUS_MASK_TYPE.ALL, dtc_severity_mask_type=DTC_SEVERITY_MASK_TYPE.ALL, dtc_mask_record=0xFFFFFF, dtc_snapshot_record_num=0xFF, dtc_extended_record_num=0xFF):
  data = ''
  # dtc_status_mask_type
  if dtc_report_type == DTC_REPORT_TYPE.NUMBER_OF_DTC_BY_STATUS_MASK or \
     dtc_report_type == DTC_REPORT_TYPE.DTC_BY_STATUS_MASK or \
     dtc_report_type == DTC_REPORT_TYPE.MIRROR_MEMORY_DTC_BY_STATUS_MASK or \
     dtc_report_type == DTC_REPORT_TYPE.NUMBER_OF_MIRROR_MEMORY_DTC_BY_STATUS_MASK or \
     dtc_report_type == DTC_REPORT_TYPE.NUMBER_OF_EMISSIONS_RELATED_OBD_DTC_BY_STATUS_MASK or \
     dtc_report_type == DTC_REPORT_TYPE.EMISSIONS_RELATED_OBD_DTC_BY_STATUS_MASK:
    data += chr(dtc_status_mask_type)
  # dtc_mask_record
  if dtc_report_type == DTC_REPORT_TYPE.DTC_SNAPSHOT_IDENTIFICATION or \
     dtc_report_type == DTC_REPORT_TYPE.DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER or \
     dtc_report_type == DTC_REPORT_TYPE.DTC_EXTENDED_DATA_RECORD_BY_DTC_NUMBER or \
     dtc_report_type == DTC_REPORT_TYPE.MIRROR_MEMORY_DTC_EXTENDED_DATA_RECORD_BY_DTC_NUMBER or \
     dtc_report_type == DTC_REPORT_TYPE.SEVERITY_INFORMATION_OF_DTC:
    data += struct.pack('!I', dtc_mask_record)[1:] # 3 bytes
  # dtc_snapshot_record_num
  if dtc_report_type == DTC_REPORT_TYPE.DTC_SNAPSHOT_IDENTIFICATION or \
     dtc_report_type == DTC_REPORT_TYPE.DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER or \
     dtc_report_type == DTC_REPORT_TYPE.DTC_SNAPSHOT_RECORD_BY_RECORD_NUMBER:
    data += ord(dtc_snapshot_record_num)
  # dtc_extended_record_num
  if dtc_report_type == DTC_REPORT_TYPE.DTC_EXTENDED_DATA_RECORD_BY_DTC_NUMBER or \
     dtc_report_type == DTC_REPORT_TYPE.MIRROR_MEMORY_DTC_EXTENDED_DATA_RECORD_BY_DTC_NUMBER:
    data += chr(dtc_extended_record_num)
  # dtc_severity_mask_type
  if dtc_report_type == DTC_REPORT_TYPE.NUMBER_OF_DTC_BY_SEVERITY_MASK_RECORD or \
     dtc_report_type == DTC_REPORT_TYPE.DTC_BY_SEVERITY_MASK_RECORD:
    data += chr(dtc_severity_mask_type) + chr(dtc_status_mask_type)
  
  resp = _uds_request(address, SERVICE_TYPE.READ_DTC_INFORMATION, subfunction=dtc_report_type, data=data)

  # TODO: parse response
  return resp

class CONTROL_OPTION_TYPE(IntEnum):
  RETURN_CONTROL_TO_ECU = 0
  RESET_TO_DEFAULT = 1
  FREEZE_CURRENT_STATE = 2
  SHORT_TERM_ADJUSTMENT = 3

def input_output_control_by_identifier(address, data_identifier_type, control_option_record, control_enable_mask_record=''):
  data = struct.pack('!H', data_identifier_type) + control_option_record + control_enable_mask_record
  resp = _uds_request(address, SERVICE_TYPE.INPUT_OUTPUT_CONTROL_BY_IDENTIFIER, subfunction=None, data=data)
  resp_id = struct.unpack('!H', resp[0:2])[0] if len(resp) >= 2 else None
  if resp_id != data_identifier_type:
    raise ValueError('invalid response data identifier: {}'.format(hex(resp_id)))
  return resp[2:]

class ROUTINE_CONTROL_TYPE(IntEnum):
  START = 1
  STOP = 2
  REQUEST_RESULTS = 3

class ROUTINE_IDENTIFIER_TYPE(IntEnum):
  ERASE_MEMORY = 0xFF00
  CHECK_PROGRAMMING_DEPENDENCIES = 0xFF01
  ERASE_MIRROR_MEMORY_DTCS = 0xFF02

def routine_control(address, routine_control_type, routine_identifier_type, routine_option_record=''):
  data = struct.pack('!H', routine_identifier_type) + routine_option_record
  resp = _uds_request(address, SERVICE_TYPE.ROUTINE_CONTROL, subfunction=routine_control_type, data=data)
  resp_id = struct.unpack('!H', resp[0:2])[0] if len(resp) >= 2 else None
  if resp_id != routine_identifier_type:
    raise ValueError('invalid response routine identifier: {}'.format(hex(resp_id)))
  return resp[2:]

def request_download(address, memory_address, memory_size, memory_address_bytes=4, memory_size_bytes=4, data_format=0x00):
  data = chr(data_format)

  if memory_address_bytes < 1 or memory_address_bytes > 4:
    raise ValueError('invalid memory_address_bytes: {}'.format(memory_address_bytes))
  if memory_size_bytes < 1 or memory_size_bytes > 4:
    raise ValueError('invalid memory_size_bytes: {}'.format(memory_size_bytes))
  data += chr(memory_size_bytes<<4 | memory_address_bytes)

  if memory_address >= 1<<(memory_address_bytes*8):
    raise ValueError('invalid memory_address: {}'.format(memory_address))
  data += struct.pack('!I', memory_address)[4-memory_address_bytes:]
  if memory_size >= 1<<(memory_size_bytes*8):
    raise ValueError('invalid memory_size: {}'.format(memory_size))
  data += struct.pack('!I', memory_size)[4-memory_size_bytes:]

  resp = _uds_request(address, SERVICE_TYPE.REQUEST_DOWNLOAD, subfunction=None, data=data)
  max_num_bytes_len = resp[0] >> 4 if len(resp) > 0 else None
  if max_num_bytes_len >= 1 and max_num_bytes_len <= 4:
    max_num_bytes = struct.unpack('!I', ('\x00'*(4-max_num_bytes_len))+resp[1:max_num_bytes_len+1])[0]
  else:
    raise ValueError('invalid max_num_bytes_len: {}'.format(max_num_bytes_len))

  return max_num_bytes # max number of bytes per transfer data request

def request_upload(address, memory_address, memory_size, memory_address_bytes=4, memory_size_bytes=4, data_format=0x00):
  data = chr(data_format)

  if memory_address_bytes < 1 or memory_address_bytes > 4:
    raise ValueError('invalid memory_address_bytes: {}'.format(memory_address_bytes))
  if memory_size_bytes < 1 or memory_size_bytes > 4:
    raise ValueError('invalid memory_size_bytes: {}'.format(memory_size_bytes))
  data += chr(memory_size_bytes<<4 | memory_address_bytes)

  if memory_address >= 1<<(memory_address_bytes*8):
    raise ValueError('invalid memory_address: {}'.format(memory_address))
  data += struct.pack('!I', memory_address)[4-memory_address_bytes:]
  if memory_size >= 1<<(memory_size_bytes*8):
    raise ValueError('invalid memory_size: {}'.format(memory_size))
  data += struct.pack('!I', memory_size)[4-memory_size_bytes:]

  resp = _uds_request(address, SERVICE_TYPE.REQUEST_UPLOAD, subfunction=None, data=data)
  max_num_bytes_len = resp[0] >> 4 if len(resp) > 0 else None
  if max_num_bytes_len >= 1 and max_num_bytes_len <= 4:
    max_num_bytes = struct.unpack('!I', ('\x00'*(4-max_num_bytes_len))+resp[1:max_num_bytes_len+1])[0]
  else:
    raise ValueError('invalid max_num_bytes_len: {}'.format(max_num_bytes_len))

  return max_num_bytes # max number of bytes per transfer data request

def transfer_data(address, block_sequence_count, data=''):
  data = chr(block_sequence_count)+data
  resp = _uds_request(address, SERVICE_TYPE.TRANSFER_DATA, subfunction=None, data=data)
  resp_id = resp[0] if len(resp) > 0 else None
  if resp_id != block_sequence_count:
    raise ValueError('invalid block_sequence_count: {}'.format(resp_id))
  return resp[1:]

def request_transfer_exit(address):
  _uds_request(address, SERVICE_TYPE.REQUEST_TRANSFER_EXIT, subfunction=None)

if __name__ == "__main__":
  from panda.python import Panda
  panda = Panda()
  bus = 1 
  tx_addr = 0x641 # tesla bosch radar RCM addr 0x18da30f1 # EPS
  tx_queue = Queue()
  rx_queue = Queue()
  can_reader_t = threading.Thread(target=_isotp_thread, args=(panda, bus, tx_addr, tx_queue, rx_queue))
  can_reader_t.daemon = True
  can_reader_t.start()

  # examples
  print("tester present ...")
  tester_present(tx_addr)
  print("extended diagnostic session ...")
  diagnostic_session_control(tx_addr, SESSION_TYPE.EXTENDED_DIAGNOSTIC)
  print("reading VIN from radar...")
  vin = read_data_by_identifier(tx_addr, DATA_IDENTIFIER_TYPE.VIN)
  print("new VIN: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0xA022)
  print("plant mode: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0xF014)
  print("board part #: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0xF015)
  print("board ser #: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  
  vin = read_data_by_identifier(tx_addr, 0xFC01)
  print("Active alignment horizontal angle: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0x508)
  print("Active Alignment Horizontal Screw: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0x505)
  print("Active Alignment State: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0xFC02)
  print("Active Alignment Vertical Angle: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0x507)
  print("Active Alignment Vertical Screw: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0x506)
  print("Active Alignment Operation: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0x50A)
  print("Service Drive Alignment State: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  vin = read_data_by_identifier(tx_addr, 0x509)
  print("Service Drive Alignment Status: {} [{}]".format(vin.decode("utf-8"), hexlify(vin)))
  
  print("reading variables from radar...")
  for i in range(0xF100, 0xF2FF):
    try:
      dat = read_data_by_identifier(tx_addr, i)
      desc = ""
      try:
        desc = " [" + DATA_IDENTIFIER_TYPE(i).name + "]"
      except ValueError:
        pass
      print("{}:{} {} {}".format(hex(i), desc, hexlify(dat), "")) #, dat.decode(errors="ignore")))
    except NegativeResponseError as e:
      if e.error_code != 0x31:
        print("{}: {}".format(hex(i), e))
