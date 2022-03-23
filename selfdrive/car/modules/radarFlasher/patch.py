#!/usr/bin/env python3
import argparse
import os
import sys
import struct
import binascii
from tqdm import tqdm
from enum import IntEnum
import time

from selfdrive.car.modules.CFG_module import load_bool_param
from panda import Panda
from panda.python.uds import UdsClient, MessageTimeoutError, NegativeResponseError, _negative_response_codes
from panda.python.uds import SESSION_TYPE, ACCESS_TYPE, ROUTINE_CONTROL_TYPE, ROUTINE_IDENTIFIER_TYPE, RESET_TYPE,DATA_IDENTIFIER_TYPE

# md5sum of supported (unmodified) firmware
FW_MD5SUM = "9e51ddd80606fbdaaf604c73c8dde0d1"
FW_START_ADDR = 0x7000
FW_END_ADDR = 0x45FFF
FW_SIZE = FW_END_ADDR - FW_START_ADDR + 1
BOOTLOADER_ADDR = 0x3ff7000

#tesla access codes
class ACCESS_TYPE_LEVEL_1(IntEnum):
  REQUEST_SEED = 0x11
  SEND_KEY = REQUEST_SEED + 1

def tesla_radar_security_access_algorithm(seeda, DEBUG=False):
    # k4 = 4 bits
    seed = int.from_bytes(seeda, byteorder="big")
    k4 = ((seed >> 5) & 8) | ((seed >> 0xB) & 4) | ((seed >> 0x18) & 1) | ((seed >> 1) & 2)
    if DEBUG: print("k4=",hex(k4))
    if DEBUG: print("seed&0x20000=",hex(seed&0x20000))

    # k32 = 32 bits
    if seed & 0x20000 == 0:
        k32 = (seed & ~(0xff << k4 & 0xFFFFFFFF)) << 0x20 - k4 & 0xFFFFFFFF | seed >> k4 & 0xFFFFFFFF
    else:
        k32 = (~(0xff << k4 & 0xFFFFFFFF) << 0x20 - k4 & seed & 0xFFFFFFFF) >> 0x20 - k4 & 0xFFFFFFFF | seed << k4 & 0xFFFFFFFF
    if DEBUG: print("k32=",hex(k32))

    # k2 = 2 bits
    k2 = seed >> 4 & 2 | seed >> 0x1F
    if DEBUG: print("k2=",hex(k2))
    if k2 == 0:
        return k32 | seed
    if k2 == 1:
        return k32 & seed
    if k2 == 2:
        return k32 ^ seed
    return k32

def tesla_epas_security_access_key(seed):

  key = 0xc541a9

  mask = struct.unpack('<I', seed.ljust(4, b'\x00'))[0] | 0x20000000
  for _i in range(32):
    msb = key & 1 ^ mask & 1
    mask = mask >> 1
    key = key >> 1
    if (msb != 0):
      key = (key | msb << 0x17) ^ 0x109028

  mask = 0x55f222f9
  for _i in range(32):
    msb = key & 1 ^ mask & 1
    mask = mask >> 1
    key = key >> 1
    if (msb != 0):
      key = (key | msb << 0x17) ^ 0x109028

  key = bytes([
    (key & 0xff0) >> 4,
    (key & 0xf000) >> 8 | (key & 0xf00000) >> 20,
    (key & 0xf0000) >> 16 | (key & 0xf) << 4,
  ])

  return key

def wait(uds_client):
  print("  wait .", end="")
  prev_timeout = uds_client.timeout
  uds_client.timeout = 0.1
  for _ in range(10):
    try:
      uds_client.tester_present()
      uds_client.timeout = prev_timeout
      print("")
      return
    except MessageTimeoutError:
      print(".", end="")
  raise Exception("reboot failed!")

def extract_firmware(uds_client, start_addr, end_addr):
  print("start extended diagnostic session ...")
  uds_client.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)

  print("request security access seed ...")
  seed = uds_client.security_access(ACCESS_TYPE.REQUEST_SEED)
  print(f"  seed: 0x{seed.hex()}")

  print("send security access key ...")
  key = tesla_radar_security_access_algorithm(seed)
  print(f"  key: 0x{key.hex()}")
  uds_client.security_access(ACCESS_TYPE.SEND_KEY, key)

  print("extract firmware ...")
  print(f"  start addr: {hex(start_addr)}")
  print(f"  end addr:   {hex(end_addr)}")
  fw = b""
  chunk_size = 128
  for addr in tqdm(range(start_addr, end_addr + 1, chunk_size)):
    dat = uds_client.read_memory_by_address(addr, chunk_size)
    assert len(dat) == chunk_size, f"expected {chunk_size} bytes but received {len(dat)} bytes starting at address {addr}"
    fw += dat
  return fw

def update_checksums(fw, offset, restore=False):
  for addr in [ 0x79c0, 0x79d0 ]:
    idx = addr - offset
    assert idx >= 0
    start = struct.unpack('<I', fw[idx:idx+4])[0]
    assert start-offset >= 0
    end = struct.unpack('<I', fw[idx+4:idx+8])[0]
    assert start < end, f"start addr {start} not less than end addr {end}"
    assert end-offset < len(fw), f"end addr {end} not inside firmware range"
    crc32 = binascii.crc32(fw[start-offset:end-offset+1])
    cksum = struct.pack("<I", crc32)
    if not restore:
      print(f"  {hex(start)}-{hex(end)} : {hex(crc32)} {'(no change)' if cksum == fw[idx+8:idx+12] else '(change)'}")
    fw = fw[:idx+8] + cksum + fw[idx+12:]
  return fw

def patch_firmware(fw, offset, restore=False):
  mods = [
    # load 1 instead of extracting EPB_epasEACAllow (message must still be present on bus)
    [0x031750, b"\x80\xff\x74\x2b", b"\x20\x56\x01\x00"],
    # load 1 instead of extracting GTW_epasControlType (message must still be present on bus)
    [0x031892, b"\x80\xff\x32\x2a", b"\x20\x56\x01\x00"],
    # load 1 instead of extracting GTW_epasLDWEnable (message must still be present on bus)
    [0x031974, b"\x80\xff\x50\x29", b"\x20\x56\x01\x00"],
  ]
  for addr, old_val, new_val in mods:
    idx = addr - offset
    assert idx >= 0
    if restore:
      # undo firmware modifications (if firmware was already patched)
      if fw[idx:idx+len(old_val)] != new_val:
        continue
      tmp_val = old_val
      old_val = new_val
      new_val = tmp_val
    else:
      # patch firmware
      print(f"  {hex(addr)} : 0x{fw[idx:idx+len(old_val)].hex()} -> 0x{new_val.hex()}")
      assert len(old_val) == len(new_val), f"{len(old_val)} != {len(new_val)}"
      assert fw[idx:idx+len(old_val)] == old_val, f"0x{fw[idx:idx+len(old_val)].hex()} != 0x{old_val.hex()}"
    fw = fw[:idx] + new_val + fw[idx+len(new_val):]
  return fw

def flash_bootloader(uds_client, bootloader_filename, start_addr):
  print("read bootloader ...")
  with open(bootloader_filename, "rb") as f:
    fw = f.read()
  fw_len = len(fw)
  end_addr = start_addr + fw_len - 1

  print("start programming session ...")
  uds_client.diagnostic_session_control(SESSION_TYPE.PROGRAMMING)
  wait(uds_client)

  print("request security access seed ...")
  seed = uds_client.security_access(ACCESS_TYPE.REQUEST_SEED)
  print(f"  seed: 0x{seed.hex()}")

  print("send security access key ...")
  key = tesla_radar_security_access_algorithm(seed)
  print(f"  key: 0x{key.hex()}")
  uds_client.security_access(ACCESS_TYPE.SEND_KEY, key)

  print("request download ...")
  print(f"  start addr: {hex(start_addr)}")
  print(f"  end addr: {hex(end_addr)}")
  print(f"  data length: {hex(fw_len)}")
  block_size = uds_client.request_download(start_addr, fw_len)

  print("transfer data ...")
  print(f"  block size: {block_size}")
  chunk_size = block_size - 2
  cnt = 0
  for i in tqdm(range(0, fw_len, chunk_size)):
    cnt += 1
    uds_client.transfer_data(cnt & 0xFF, fw[i:i+chunk_size])

  print("request transfer exit ...")
  uds_client.request_transfer_exit()

  print("enter bootloader ...")
  uds_client.routine_control(ROUTINE_CONTROL_TYPE.START, 0x0301, struct.pack(">I", start_addr))

def flash_firmware(uds_client, fw_slice, start_addr, end_addr):
  slice_len = end_addr - start_addr + 1
  assert slice_len == len(fw_slice)
  start_and_length = struct.pack('>II', start_addr, slice_len)

  print("erase memory ...")
  print(f"  start addr: {hex(start_addr)}")
  print(f"  end addr: {hex(end_addr)}")
  print(f"  data length: {hex(slice_len)}")
  uds_client.routine_control(ROUTINE_CONTROL_TYPE.START, ROUTINE_IDENTIFIER_TYPE.ERASE_MEMORY, start_and_length)

  print("request download ...")
  print(f"  start addr: {hex(start_addr)}")
  print(f"  end addr: {hex(end_addr)}")
  print(f"  data length: {hex(slice_len)}")
  block_size = uds_client.request_download(start_addr, slice_len)

  print("transfer data ...")
  print(f"  block size: {block_size}")
  chunk_size = block_size - 2
  cnt = 0
  for i in tqdm(range(0, slice_len, chunk_size)):
    cnt += 1
    uds_client.transfer_data(cnt & 0xFF, fw_slice[i:i+chunk_size])

  print("request transfer exit ...")
  uds_client.request_transfer_exit()

  print("reset ...")
  wait(uds_client)
  try:
    uds_client.ecu_reset(RESET_TYPE.HARD | 0x80)
  except MessageTimeoutError:
    # supress response bit set, so timeout expected
    # (timeout is used to wait for reboot to complete)
    pass
  wait(uds_client)

  print("start extended diagnostic session ...")
  uds_client.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)

  print("request security access seed ...")
  seed = uds_client.security_access(ACCESS_TYPE.REQUEST_SEED)
  print(f"  seed: 0x{seed.hex()}")

  print("send security access key ...")
  key = tesla_radar_security_access_algorithm(seed)
  print(f"  key: 0x{key.hex()}")
  uds_client.security_access(ACCESS_TYPE.SEND_KEY, key)

  print("check dependencies ...")
  uds_client.routine_control(ROUTINE_CONTROL_TYPE.START, 0xDC03)

  print("complete!")

def vin_learn(udcli):
  print("\n[START DIAGNOSTIC SESSION]")
  udcli.tester_present()
  udcli.diagnostic_session_control(SESSION_TYPE.DEFAULT)
  udcli.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
  wait(udcli)
  print("request security access seed ...")
  seed = udcli.security_access(ACCESS_TYPE_LEVEL_1.REQUEST_SEED)
  print(f"  seed: 0x{seed.hex()}")

  print("send security access key ...")
  key = struct.pack("!I",tesla_radar_security_access_algorithm(seed))
  udcli.security_access(ACCESS_TYPE_LEVEL_1.SEND_KEY, key)

  print("Starting VIN learn...")
  output = udcli.routine_control(ROUTINE_CONTROL_TYPE.START, 2563)
  ns = 0
  nsmax = 2
  while ns < nsmax:
    for i in range(3):
      time.sleep(2)
      try:
        output = udcli.routine_control(ROUTINE_CONTROL_TYPE.STOP, 2563)
      except NegativeResponseError as e:
        print(('Failed to stop vin learning on attempt #{0}. ({1})').format(i + 1,_negative_response_codes[e.error_code]))
        if i == 2:
          raise
      else:
          ns += 1
          if ns >= nsmax:
            output = udcli.routine_control(ROUTINE_CONTROL_TYPE.REQUEST_RESULTS, 2563)
          break
  print("VIN learn complete! [",output,"]")

def read_values_from_radar(udcli):
  print("\n[START DIAGNOSTIC SESSION]")
  #udcli.tester_present()
  udcli.diagnostic_session_control(SESSION_TYPE.DEFAULT)
  udcli.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)

  print("reading VIN from radar...")
  vin = udcli.read_data_by_identifier(DATA_IDENTIFIER_TYPE.VIN)
  print("new VIN: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0xA022)
  print("plant mode: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0xF014)
  print("board part #: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0xF015)
  print("board ser #: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  
  vin = udcli.read_data_by_identifier(0xFC01)
  print("Active alignment horizontal angle: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0x508)
  print("Active Alignment Horizontal Screw: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0x505)
  print("Active Alignment State: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0xFC02)
  print("Active Alignment Vertical Angle: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0x507)
  print("Active Alignment Vertical Screw: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0x506)
  print("Active Alignment Operation: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0x50A)
  print("Service Drive Alignment State: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  vin = udcli.read_data_by_identifier(0x509)
  print("Service Drive Alignment Status: {} [{}]".format(vin.decode("utf-8"), binascii.hexlify(vin)))
  
  print("reading variables from radar...")
  for i in range(0xF100, 0xF2FF):
    try:
      dat = udcli.read_data_by_identifier(i)
      desc = ""
      try:
        desc = " [" + DATA_IDENTIFIER_TYPE(i).name + "]"
      except ValueError:
        pass
      print("{}:{} {} {}".format(hex(i), desc, binascii.hexlify(dat), "")) #, dat.decode(errors="ignore")))
    except NegativeResponseError as e:
      if e.error_code != 0x31:
        print("{}: {}".format(hex(i), e))

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--extract-only', action='store_true', help='extract the firmware (do not flash)')
  parser.add_argument('--extract-params', action='store_true', help='extract the programmed params from radar')
  parser.add_argument('--vin-learn', action='store_true', help='switch radar in programming mode to learn VIN')
  parser.add_argument('--restore', action='store_true', help='flash firmware without modification')
  parser.add_argument('--debug', action='store_true', help='print additional debug messages')
  parser.add_argument('--bootloader', type=str, default="./radarfw.bin", help='path to firmware file')
  parser.add_argument('--can-addr', type=int, default=0x641, help='TX CAN address for UDS')  #receives on 0x671, answers on 0x681  - UDS_radcRequest 1649 -  RADC_udsResponse 1665
  parser.add_argument('--can-bus', type=int, default=1, help='CAN bus number (zero based)')
  args = parser.parse_args()

  safetyParam = 0
  if not load_bool_param("TinklaForceTeslaPreAP", False):
    print("This software should only be used on preAP Tesla Model S!!!")
    sys.exit(1)
  if load_bool_param("TinklaUseTeslaRadar",False):
      safetyParam = safetyParam | Panda.FLAG_TESLA_NEED_RADAR_EMULATION
      if load_bool_param("TinklaTeslaRadarBehindNosecone",False):
        safetyParam = safetyParam | Panda.FLAG_TESLA_RADAR_BEHIND_NOSECONE
  else:
    print("Use of Tesla Radar not enabled in toggles.")
    sys.exit(1)

  if load_bool_param("TinklaHasIBooster",False):
      safetyParam = safetyParam | Panda.FLAG_TESLA_HAS_IBOOSTER

  panda = Panda()
  panda.reset()
  #negative safetyParam used when doing VIN learn in our SAFETY_TESL for now
  panda.set_safety_mode(Panda.SAFETY_TESLA, param=-safetyParam)
  uds_client = UdsClient(panda, args.can_addr, bus=args.can_bus, rx_addr=args.can_addr + 0x10, timeout=3, debug=args.debug)

  os.chdir(os.path.dirname(os.path.realpath(__file__)))

  #radc:7	radc/1/radc.bhx	radc.bhx	radc	fbfe4746	dasHw=1,forwardRadarHW=1
  
  if args.extract_params:
    read_values_from_radar(uds_client)
    sys.exit(0)

  if args.vin_learn:
    vin_learn(uds_client)
    sys.exit(0)

  # fw_slice = None
  # fw_fn = f"./epas-firmware-{hex(FW_START_ADDR)}-{hex(FW_END_ADDR)}.bin"
  # if args.extract_only or not os.path.exists(fw_fn):
  #   fw_slice = extract_firmware(uds_client, FW_START_ADDR, FW_END_ADDR)
  #   # undo firmware changes in case firmware was already patched
  #   fw_slice = patch_firmware(fw_slice, FW_START_ADDR, restore=True)
  #   fw_slice = update_checksums(fw_slice, FW_START_ADDR, restore=True)
  #   print(f"  file name: {fw_fn}")
  #   with open(fw_fn, "wb") as f:
  #     f.write(fw_slice)

  # if args.extract_only:
  #   exit(0)

  # if fw_slice is None:
  #   print("load firmware ...")
  #   print(f"  file name: {fw_fn}")
  #   with open(fw_fn, "rb") as f:
  #     fw_slice = f.read()
  # md5 = hashlib.md5(fw_slice).hexdigest()

  # assert len(fw_slice) == FW_SIZE, f"expected {FW_SIZE} bytes of firmware but got {len(fw_slice)} bytes"
  # assert md5 == FW_MD5SUM, f"expected md5sum of firmware to be in {FW_MD5SUM} but found {md5}"
  # fw_mod_fn = f"{md5}.modified.bin"

  # if not args.restore:
  #   print("modify firmware ...")
  #   fw_slice = patch_firmware(fw_slice, FW_START_ADDR)
  #   print("update checksums ...")
  #   fw_slice = update_checksums(fw_slice, FW_START_ADDR)
  #   print(f"  file name: {fw_mod_fn}")
  #   with open(fw_mod_fn, "wb") as f:
  #     f.write(fw_slice)

  # if args.extract_only:
  #   sys.exit(0)

  # flash_bootloader(uds_client, args.bootloader, BOOTLOADER_ADDR)
  # flash_firmware(uds_client, fw_slice, FW_START_ADDR, FW_END_ADDR)
