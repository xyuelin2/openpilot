#!/usr/bin/env python3
import struct
import traceback
import re

import cereal.messaging as messaging
import panda.python.uds as uds
from panda.python.uds import FUNCTIONAL_ADDRS
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from system.swaglog import cloudlog

OBD_VIN_REQUEST = b'\x09\x02'
OBD_VIN_RESPONSE = b'\x49\x02\x01'

UDS_VIN_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + struct.pack("!H", uds.DATA_IDENTIFIER_TYPE.VIN)
UDS_VIN_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + struct.pack("!H", uds.DATA_IDENTIFIER_TYPE.VIN)

VIN_UNKNOWN = "0" * 17


def check_vin(vin: str) -> bool:
  # 17 characters, 0-9, A-Z except 'I', 'H', 'O' and 'Q'
  return vin is not None and re.fullmatch(r"[0-9A-HJ-NPR-Z]{17}", vin, flags=re.I | re.A) is not None


def get_vin(logcan, sendcan, bus, timeout=0.1, retry=5, debug=False):
  for request, response in ((UDS_VIN_REQUEST, UDS_VIN_RESPONSE), (OBD_VIN_REQUEST, OBD_VIN_RESPONSE)):
    for i in range(retry):
      try:
        query = IsoTpParallelQuery(sendcan, logcan, bus, FUNCTIONAL_ADDRS, [request, ], [response, ], functional_addr=True, debug=debug)
        for addr, vin in query.get_data(timeout).items():

          # Honda Bosch response starts with a length, trim to correct length
          if vin.startswith(b'\x11'):
            vin = vin[1:18]

          return addr[0], vin.decode()
        print(f"vin query retry ({i+1}) ...")
      except Exception:
        cloudlog.warning(f"VIN query exception: {traceback.format_exc()}")

  return 0, VIN_UNKNOWN


class GMVinCapturer:
  # TODO: Generalize in case other brands need similar method
  PART_A_MSG_ID: int = 1300
  PART_B_MSG_ID: int = 1249
  
  def __init__(self):
    self.__need_a: bool = True
    self.__need_b: bool = True
    self.success: bool = False
    self.__a: str = None
    self.__b: str = None
    self.vin: str = None
    self.complete: bool = False
    
  def read(self, msg):
    """Check if a CAN message is a VIN part and collect values
    Args:
        msg (Any): Can message
    """
    # Attempted to makes this as fast as possible, and
    # possibly tolerant of re-entrance
    
    if self.complete:
      return
    
    if len(msg.dat) != 8:
      self.complete = True
      return
    
    if self.__need_a and msg.address == GMVinCapturer.PART_A_MSG_ID:
      self.__a = msg.dat.decode()
      self.__need_a = False
      print(f"GMVIN Saw message part A ({msg.address}): {self.__a}")
    elif self.__need_b and msg.address == GMVinCapturer.PART_B_MSG_ID:
      self.__b = msg.dat.decode()
      self.__need_b = False
      print(f"GMVIN Saw message part B ({msg.address}): {self.__b}")


    if (not self.__need_a) and (not self.__need_b):
      vin_t  = "1" + self.__a + self.__b
      print(f"GMVIN Got both parts: {vin_t}")  
      if check_vin(vin_t):
        self.vin = vin_t
        self.success = True
      else:
        print(f"GMVIN VIN was bad: {vin_t}")  
      self.complete = True


if __name__ == "__main__":
  import time
  sendcan = messaging.pub_sock('sendcan')
  logcan = messaging.sub_sock('can')
  time.sleep(1)
  addr, vin = get_vin(logcan, sendcan, 1, debug=False)
  print(hex(addr), vin)
