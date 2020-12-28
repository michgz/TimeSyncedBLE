#!/usr/bin/python3

import asyncio
import bleak
import bleak.utils
import logging
import random
import struct
import datetime

from bleak import _logger as logger

TARGET_ADDRESS = "F1:C8:1A:8D:37:8B"
TARGET_LEAF_ADDRESS =   "F1:C8:1A:8D:37:8B"  #"E2:45:E5:25:22:F0"


SERVICE_UUID = "21171523-4740-4AA5-B66B-5D2C6851CC5C"
READ_CHAR_UUID = "21171524-4740-4aa5-b66b-5d2c6851cc5c"
WRITE_CHAR_UUID = "21170002-4740-4aa5-b66b-5d2c6851cc5c"


CONFIG_SERVICE_UUID = "2117000B-4740-4AA5-B66B-5D2C6851CC5C"
CONFIG_1_CHAR_UUID =  "2117000C-4740-4AA5-B66B-5D2C6851CC5C"
CONFIG_2_CHAR_UUID =  "2117000D-4740-4AA5-B66B-5D2C6851CC5C"
CONFIG_3_CHAR_UUID =  "2117000E-4740-4AA5-B66B-5D2C6851CC5C"

# This function exists in random 3.9, but not 3.8
def randbytes(n):
  if n == 1:
    return struct.pack('B', random.randint(0,255))
  else:
    return struct.pack('4B', random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255))

INSTRUCTION_CODE__QUERY_IS_SYNCED = 0x11
INSTRUCTION_CODE__QUERY_CURRENT_TIME = 0x16
INSTRUCTION_CODE__LOCK = 0xA
INSTRUCTION_CODE__IS_LOCKED = 0xB
INSTRUCTION_CODE__READ_OUT = 0xE
INSTRUCTION_CODE__IDENTIFY_SELF = 0x23


rxed_data = b''
dev_list = []
do_dev_list = False
is_leaf_locked = False

def notification_handler_leaf(sender, data):
  
    global rxed_data
    global dev_list
    global do_dev_list
    global is_leaf_locked
    
    if len(data) >= 8:
      if struct.unpack('<I', data[0:4])[0] == 0x0000001A:
        count_dev = struct.unpack('<I', data[4:8])[0]
        if 8+8*count_dev == len(data):
          print("Got a list")
          devs = []
          i = 8
          while i+8 <= len(data):
            a = ""
            for j in range(6):
              a += "{0:02X}".format(data[i+6-j])
              if j < 5:
                a += ":"
            devs.append(a)
            i += 8
          dev_list = devs
          #print(dev_list)
          do_dev_list = True
      if struct.unpack('<I', data[0:4])[0] == 0x8000000B:
        res_dev = struct.unpack('<I', data[4:8])[0]
        is_leaf_locked = (res_dev != 0)
          
    """Simple notification handler which prints the data received."""
    rxed_data += data
    print("{0}: {1}".format(sender, data))

def notification_handler_leaf_2(sender, data):
  
    """Simple notification handler which prints the data received."""
    print("        {0}: {1}".format(sender, data))


async def run_specific_device_leaf(addr, debug=False):
  
    global rxed_data
    global dev_list
    global do_dev_list
    global is_leaf_locked
    
    if debug:
        import sys

        # loop.set_debug(True)
        l = logging.getLogger("asyncio")
        l.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        l.addHandler(h)
        logger.addHandler(h)

    # Do an initial connection to wake it up
    #async with bleak.BleakClient(addr) as client:
    #    x = await client.is_connected()
    #    l.info("Connected: {0}".format(x))

    #await asyncio.sleep(40.0)

    # Now do the connection where we lock and read data
    async with bleak.BleakClient(addr) as client:
        x = await client.is_connected()
        l.info("Connected: {0}".format(x))
        y = await client.get_services()
        y_serv = y.get_service(SERVICE_UUID.lower())
        l.info(y_serv)
        y_char = y.get_characteristic(READ_CHAR_UUID.lower())
        l.info(y_char)
        l.info(y_char.service_uuid)   
        
        await client.start_notify(READ_CHAR_UUID, notification_handler_leaf)
        
        if False:
          #await client.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__IDENTIFY_SELF, 0)))  # no data, just send 0
          #await asyncio.sleep(2.0)

          await client.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__QUERY_IS_SYNCED, 0)))  # no data, just send 0
          await asyncio.sleep(0.2)


        else:
          
          #  #await client.write_gatt_char(WRITE_CHAR_UUID, bytearray([0xe,0,0,0,0xff,0xff,0xff,0xff]))
          await client.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__QUERY_IS_SYNCED, 0)))  # no data, just send 0
          await asyncio.sleep(0.2)
          await client.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__QUERY_CURRENT_TIME, 0)))  # no data, just send 0
          await asyncio.sleep(0.2)
          await client.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__IS_LOCKED, 0)))  # no data, just send 0
          await asyncio.sleep(0.2)
          
          #await client.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__LOCK, 0xFFFFFFFF)))  # 0xFFFFFFFF means current time
          #await asyncio.sleep(0.4)
          await client.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__IS_LOCKED, 0)))  # no data, just send 0
          await asyncio.sleep(5.)  # Need to give sufficient time for the buffer to fill up.
          # If not sufficient, nothing will be returned
          
          
          rxed_data = b''
          
          #await client.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__READ_OUT, 0)))  # no data, just send 0
          #await asyncio.sleep(0.2)

        a = 0
        while a < 1000:
          await asyncio.sleep(1.0)
          if do_dev_list:
            do_dev_list = False
            for dd in dev_list:
              async with bleak.BleakClient(dd) as client_2:
                x_2 = await client.is_connected()
                l.info("Connected: {0}".format(x_2))
                await client_2.start_notify(READ_CHAR_UUID, notification_handler_leaf_2)
                await asyncio.sleep(1.2)
                await client_2.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__QUERY_IS_SYNCED, 0)))  # no data, just send 0
                await asyncio.sleep(0.2)
                await client_2.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__QUERY_CURRENT_TIME, 0)))  # no data, just send 0
                await asyncio.sleep(0.2)
                is_leaf_locked = False
                await client_2.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__IS_LOCKED, 0)))  # no data, just send 0
                await asyncio.sleep(0.4)
                if is_leaf_locked:
                  rxed_data = b''
                  await client_2.write_gatt_char(WRITE_CHAR_UUID, bytearray(struct.pack('<2I', INSTRUCTION_CODE__READ_OUT, 0)))  # no data, just send 0
                  await asyncio.sleep(5.2)
                  if len(rxed_data) > 0:
                    with open(datetime.datetime.now().strftime("%Y%m%d_%H%M%S_")+dd+".bin", "wb") as f1:
                      f1.write(rxed_data)
                
                await client_2.stop_notify(READ_CHAR_UUID)

                z_2 = await client_2.disconnect()
                l.info("Disconnected: {0}".format(z_2))
          a += 1
        
        # Now save any data that has been received
        if len(rxed_data) > 0:
          with open(datetime.datetime.now().strftime("%Y%m%d_%H%M%S.bin"), "wb") as f1:
            f1.write(rxed_data)
          with open(datetime.datetime.now().strftime("%Y%m%d_%H%M%S.txt"), "w") as f1:
            i = 0
            j = 0
            while i+8 <= len(rxed_data):
              tt = struct.unpack('<4h', rxed_data[i:i+8])
              
              f1.write("{0},{1},{2},{3},{4}\n".format(j, tt[0],tt[1],tt[2], tt[3]))
              i += 8
              j += 1

        
        rxed_data = b''
        await client.stop_notify(READ_CHAR_UUID)

        z = await client.disconnect()
        logger.info("Disconnected: {0}".format(z))
        await asyncio.sleep(1.0)





def notification_handler(sender, data):
    """Simple notification handler which prints the data received."""
    if len(data)==12 and data[:4]==b'\x22\x00\x00\x80':
      # This is a trigger event
      tt = struct.unpack('<3I', data)
      print(tt)
      
      print(datetime.datetime.now().isoformat() + "   Size : {0}     Time : {1}".format(float(tt[1]), tt[2]))
    else:
      print("{0}: {1}".format(sender, data))

async def run_specific_device(addr, debug=False):
    if debug:
        import sys

        # loop.set_debug(True)
        l = logging.getLogger("asyncio")
        l.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        l.addHandler(h)
        logger.addHandler(h)

    async with bleak.BleakClient(addr) as client:
        x = await client.is_connected()
        l.info("Connected: {0}".format(x))
        y = await client.get_services()
        y_serv = y.get_service(SERVICE_UUID.lower())
        l.info(y_serv)
        y_char = y.get_characteristic(READ_CHAR_UUID.lower())
        l.info(y_char)
        l.info(y_char.service_uuid)   
        
        #c_serv = y.get_service(CONFIG_SERVICE_UUID.lower())
        #l.info(c_serv)
        #c_char = y.get_characteristic(CONFIG_3_CHAR_UUID.lower())
        #l.info(c_char)
        #l.info(c_char.service_uuid)
        
        char_1_val = await client.read_gatt_char(CONFIG_1_CHAR_UUID.lower())
        print("Char 1 Value: 0x{0:04X}".format(char_1_val[0]))
        print("Char 1 length: {0}".format(len(char_1_val)))
        char_2_val = await client.read_gatt_char(CONFIG_2_CHAR_UUID.lower())
        print("Char 2 Value: 0x{0:02X} {1:02X} {2:02X} {3:02X}".format(char_2_val[0], char_2_val[1], char_2_val[2], char_2_val[3]))
        print("Char 2 length: {0}".format(len(char_2_val)))
        char_3_val = await client.read_gatt_char(CONFIG_3_CHAR_UUID.lower())
        print("Char 3 Value: 0x{0:04X}".format(char_3_val[0]))
        print("Char 3 length: {0}".format(len(char_3_val)))
        
        
        if False:
          x = bytearray(randbytes(1))
          print("Writing Char1 to 0x{0:02X}".format(x[0]))
          await client.write_gatt_char(CONFIG_1_CHAR_UUID, x)
          x = bytearray(randbytes(4))
          print("Writing Char2 to 0x{0:02X} {1:02X} {2:02X} {3:02X}".format(x[0],x[1],x[2],x[3]))
          await client.write_gatt_char(CONFIG_2_CHAR_UUID, x)
          await asyncio.sleep(0.6)
          print("Done")
        
        
        
        await client.start_notify(READ_CHAR_UUID, notification_handler)
        #  #await client.write_gatt_char(WRITE_CHAR_UUID, bytearray([0xe,0,0,0,0xff,0xff,0xff,0xff]))
        #await client.write_gatt_char(WRITE_CHAR_UUID, bytearray([0x21,0,0,0,0,0,0,0]))
        #await asyncio.sleep(0.2)
        #await client.write_gatt_char(WRITE_CHAR_UUID, bytearray([0x0E,0,0,0,0,0,0,0]))
        await asyncio.sleep(25.0)
        await client.stop_notify(READ_CHAR_UUID)


        z = await client.disconnect()
        logger.info("Disconnected: {0}".format(z))
        await asyncio.sleep(1.0)



async def run_any_device(debug=False):
    async def found_unit(dev):
        await asyncio.sleep(1.0)
        async with bleak.BleakClient(dev.address) as client:
            svcs = await client.get_services()
            print("Services:", svcs.services)

    if debug:
        import sys
        import logging

        # loop.set_debug(True)
        l = logging.getLogger("asyncio")
        l.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        l.addHandler(h)
        logger.addHandler(h)

    x = await bleak.BleakScanner.discover()
    l.info("*********** Start of discovered list *************")
    for d in x:
        l.info(d)
        l.info(bleak.utils.mac_str_2_int(d.address))
        if d.address == TARGET_ADDRESS:
            await found_unit(d)
    l.info("*********** End of discovered list ***************")
    await asyncio.sleep(1.0)

if __name__ == "__main__":
    import os

    print("Starting:  " + datetime.datetime.now().isoformat())

    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    #asyncio.run(run_any_device(True))
    #asyncio.run(run_specific_device(TARGET_ADDRESS, True))
    asyncio.run(run_specific_device_leaf(TARGET_LEAF_ADDRESS, True))

    print("Ending:    " + datetime.datetime.now().isoformat())
