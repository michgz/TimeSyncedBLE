#!/usr/bin/python3

import asyncio
import bleak
import bleak.utils
import logging
import random
import struct
import datetime

from bleak import _logger as logger

SERVICE_UUID = "21171523-4740-4AA5-B66B-5D2C6851CC5C"
READ_CHAR_UUID = "21171524-4740-4aa5-b66b-5d2c6851cc5c"
WRITE_CHAR_UUID = "21170002-4740-4aa5-b66b-5d2c6851cc5c"


CONFIG_SERVICE_UUID = "2117000B-4740-4AA5-B66B-5D2C6851CC5C"
CONFIG_1_CHAR_UUID =  "2117000C-4740-4AA5-B66B-5D2C6851CC5C"
CONFIG_2_CHAR_UUID =  "2117000D-4740-4AA5-B66B-5D2C6851CC5C"
CONFIG_3_CHAR_UUID =  "2117000E-4740-4AA5-B66B-5D2C6851CC5C"

async def run_all_devices(debug=False):
  
    if debug:
        import sys

        # loop.set_debug(True)
        l = logging.getLogger("asyncio")
        l.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        l.addHandler(h)
        logger.addHandler(h)

    x = await bleak.BleakScanner.discover()
    l.info("*********** Start of discovered list *************")
    i = 1
    for d in x:
        #l.info(d)
        #l.info(bleak.utils.mac_str_2_int(d.address))
        print("[{0}]   ".format(i) + d.address + "  " + d.name)
        i += 1
        if i >= 10:
            break
        #if d.address == TARGET_ADDRESS:
        #    await found_unit(d)
    l.info("*********** End of discovered list ***************")

    print("Enter number to connect (or anything else to cancel):")

    s = input()

    try:
        j_s = int(s)
    except ValueError:
        j_s = -1
        
    if j_s > 0 and j_s <= len(x):
        # a good value!
        async with bleak.BleakClient(x[j_s-1].address) as client:
            y = await client.is_connected()
            l.info("Connected: {0}".format(y))
            w = await client.get_services()
            w_serv = w.get_service(CONFIG_SERVICE_UUID.lower())
            l.info(w_serv)
            w_char = [w.get_characteristic(CONFIG_1_CHAR_UUID.lower()),w.get_characteristic(CONFIG_2_CHAR_UUID.lower()),w.get_characteristic(CONFIG_3_CHAR_UUID.lower())]
            l.info(w_char[0].service_uuid)   
            l.info(w_char[0])
            l.info(w_char[1].service_uuid)   
            l.info(w_char[1])
            l.info(w_char[2].service_uuid)   
            l.info(w_char[2])
            
            t_0 = await client.read_gatt_char(w_char[0])
            t_1 = await client.read_gatt_char(w_char[1])
            t_2 = await client.read_gatt_char(w_char[2])
            
            
            print(t_0)
            print(t_1)
            print(t_2)
            
            
            is_central = False
            
            if len(t_0) >= 1:
                is_central = ((t_0[0] & 0x01) != 0)
                if is_central:
                  print("This is a central device. Choose:")
                  print("[1] change to peripheral")
                  print("[2] no change (remain as central)")
                  
                  s = input()
                  
                  try:
                      k_s = int(s)
                  except ValueError:
                      k_s = 2
                      
                  if k_s == 1:
                      await client.write_gatt_char(w_char[0], bytearray(b'\x00'))
                      is_central = False
                      print("Changed")
                else:
                  print("This is a peripheral device. Choose:")
                  print("[1] no change (remain as peripheral)")
                  print("[2] change to central")
                  
                  s = input()
                  
                  try:
                      k_s = int(s)
                  except ValueError:
                      k_s = 1
                      
                  if k_s == 2:
                      await client.write_gatt_char(w_char[0], bytearray(b'\x01'))
                      print("Changed")
                      is_central = True
                      
                  
            if len(t_1) >= 4 and is_central:
                val_2 = struct.unpack('<I', t_1)[0]
                print("This is a central device with trigger threshold {0}.".format(0x3FFF & val_2))
                print("Enter a new value, or <Enter> to keep old one:")
                
                s = input()
                
                try:
                    k_s = int(s)
                except ValueError:
                    k_s = -1
                      
                if k_s >= 0:
                    new_val = (val_2 & 0xFFFFC000 ) | (k_s & 0x3FFF)
                    await client.write_gatt_char(w_char[1], bytearray(struct.pack('<I', new_val)))
                    print("Changed")
                else:
                    print("No change")

            z = await client.disconnect()
            logger.info("Disconnected: {0}".format(z))
            await asyncio.sleep(1.0)

    await asyncio.sleep(1.0)
    
if __name__ == "__main__":
    import os

    print("Starting:  " + datetime.datetime.now().isoformat())

    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    asyncio.run(run_all_devices(True))

    print("Ending:    " + datetime.datetime.now().isoformat())
