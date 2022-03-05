#!/usr/bin/env python3
import argparse
import os
import sys
import struct
import hashlib
import binascii
from tqdm import tqdm

from panda import Panda
from panda.python.uds import UdsClient, MessageTimeoutError
from panda.python.uds import SESSION_TYPE, ACCESS_TYPE, ROUTINE_CONTROL_TYPE, ROUTINE_IDENTIFIER_TYPE, RESET_TYPE

# md5sum of supported (unmodified) firmware
FW_MD5SUM = "9e51ddd80606fbdaaf604c73c8dde0d1"
FW_START_ADDR = 0x7000
FW_END_ADDR = 0x45FFF
FW_SIZE = FW_END_ADDR - FW_START_ADDR + 1
BOOTLOADER_ADDR = 0x3ff7000

def get_security_access_key(seed):
  key = 0xc541a9

  mask = struct.unpack('<I', seed.ljust(4, b'\x00'))[0] | 0x20000000
  for i in range(32):
    msb = key & 1 ^ mask & 1
    mask = mask >> 1
    key = key >> 1
    if (msb != 0):
      key = (key | msb << 0x17) ^ 0x109028

  mask = 0x55f222f9
  for i in range(32):
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
  key = get_security_access_key(seed)
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
  print(f"read bootloader ...")
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
  key = get_security_access_key(seed)
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
  key = get_security_access_key(seed)
  print(f"  key: 0x{key.hex()}")
  uds_client.security_access(ACCESS_TYPE.SEND_KEY, key)

  print("check dependencies ...")
  uds_client.routine_control(ROUTINE_CONTROL_TYPE.START, 0xDC03)

  print("complete!")

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--extract-only', action='store_true', help='extract the firmware (do not flash)')
  parser.add_argument('--restore', action='store_true', help='flash firmware without modification')
  parser.add_argument('--debug', action='store_true', help='print additional debug messages')
  parser.add_argument('--bootloader', type=str, default="./epas-bootloader-0x3ff7000-0x3ffacbd.bin", help='path to bootloader file')
  parser.add_argument('--can-addr', type=int, default=0x730, help='TX CAN address for UDS')
  parser.add_argument('--can-bus', type=int, default=0, help='CAN bus number (zero based)')
  args = parser.parse_args()

  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ELM327)
  uds_client = UdsClient(panda, args.can_addr, bus=args.can_bus, timeout=1, debug=args.debug)

  os.chdir(os.path.dirname(os.path.realpath(__file__)))

  fw_slice = None
  fw_fn = f"./epas-firmware-{hex(FW_START_ADDR)}-{hex(FW_END_ADDR)}.bin"
  if args.extract_only or not os.path.exists(fw_fn):
    fw_slice = extract_firmware(uds_client, FW_START_ADDR, FW_END_ADDR)
    # undo firmware changes in case firmware was already patched
    fw_slice = patch_firmware(fw_slice, FW_START_ADDR, restore=True)
    fw_slice = update_checksums(fw_slice, FW_START_ADDR, restore=True)
    print(f"  file name: {fw_fn}")
    with open(fw_fn, "wb") as f:
      f.write(fw_slice)

  if args.extract_only:
    exit(0)

  if fw_slice is None:
    print("load firmware ...")
    print(f"  file name: {fw_fn}")
    with open(fw_fn, "rb") as f:
      fw_slice = f.read()
  md5 = hashlib.md5(fw_slice).hexdigest()

  assert len(fw_slice) == FW_SIZE, f"expected {FW_SIZE} bytes of firmware but got {len(fw_slice)} bytes"
  assert md5 == FW_MD5SUM, f"expected md5sum of firmware to be in {FW_MD5SUM} but found {md5}"
  fw_mod_fn = f"{md5}.modified.bin"

  if not args.restore:
    print("modify firmware ...")
    fw_slice = patch_firmware(fw_slice, FW_START_ADDR)
    print("update checksums ...")
    fw_slice = update_checksums(fw_slice, FW_START_ADDR)
    print(f"  file name: {fw_mod_fn}")
    with open(fw_mod_fn, "wb") as f:
      f.write(fw_slice)

  if args.extract_only:
    sys.exit(0)

  flash_bootloader(uds_client, args.bootloader, BOOTLOADER_ADDR)
  flash_firmware(uds_client, fw_slice, FW_START_ADDR, FW_END_ADDR)
