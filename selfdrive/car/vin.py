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
  __INVALID_VIN_REGEX = re.compile(r"[OoIiQq]")
  PART_A_MSG_ID = 1300
  PART_B_MSG_ID = 1249
  
  def __init__(self):
    self.__need_a: bool = True
    self.__need_b: bool = True
    self.__success: bool = False
    self.__a: str = None
    self.__b: str = None
    self.vin: str = None
    self.complete: bool = False
  
  def __gm_validate_vin(self, vin: str) -> bool:
    # TODO: inline
    # TODO: there is another global restriction on the year character
    # Basic sanity check. VIN rules vary globally, and while a 1G
    # prefix applies to GM in the US and canada, there are EU vehicles
    # identical to US counterparts
    if vin is None or not isinstance(vin, str) or len(vin) != 17 or GMVinCapturer.__INVALID_VIN_REGEX.match(vin):
      return False
    #17-character VIN, which does not include the letters O (o), I (i), and Q (q) (to avoid confusion with numerals 0, 1, and 9).
    return True
  
  def read(self, msg):
    """Check if a CAN message is a VIN part and collect values
    Args:
        msg (Any): Can message

    Returns:
        str: None until complete VIN is captured, then the captured VIN
    """
    if self.complete:
      if self.__success:
        return vin
      else:
        return None
    
    if len(msg.dat) != 8:
      self.complete = True
      return None
    
    if msg.address == GMVinCapturer.PART_A_MSG_ID and self.__need_a:
      self.__need_a = False
      self.__a = msg.msg.decode()
    elif msg.address == GMVinCapturer.PART_B_MSG_ID and self.__need_b:
      self.__need_b = False
      self.__b = msg.msg.decode()

    if not self.__need_a and not self.__need_b:
      vin_t  = "1" + self.__a + self.__b      
      if self.__gm_validate_vin(vin_t):
        self.vin = vin_t
        self.__success = True
        return vin
    
    return None

if __name__ == "__main__":
  import time
  sendcan = messaging.pub_sock('sendcan')
  logcan = messaging.sub_sock('can')
  time.sleep(1)
  addr, vin = get_vin(logcan, sendcan, 1, debug=False)
  print(hex(addr), vin)
