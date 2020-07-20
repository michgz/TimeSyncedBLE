#!/usr/bin/python3

import asyncio
import bleak
import bleak.utils
import logging

from bleak import _logger as logger

TARGET_ADDRESS = "DC:9E:80:E8:B2:83"

SERVICE_UUID = "21171523-4740-4AA5-B66B-5D2C6851CC5C"
READ_CHAR_UUID = "21171524-4740-4aa5-b66b-5d2c6851cc5c"
WRITE_CHAR_UUID = "21170002-4740-4aa5-b66b-5d2c6851cc5c"



def notification_handler(sender, data):
    """Simple notification handler which prints the data received."""
    print("{0}: {1}".format(sender, data))

async def run_specific_device(loop, addr, debug=False):
    if debug:
        import sys

        # loop.set_debug(True)
        l = logging.getLogger("asyncio")
        l.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        l.addHandler(h)
        logger.addHandler(h)

    async with bleak.BleakClient(addr, loop=loop) as client:
        x = await client.is_connected()
        logger.info("Connected: {0}".format(x))
        y = await client.get_services()
        y_serv = y.get_service(SERVICE_UUID.lower())
        logger.info(y_serv)
        y_char = y.get_characteristic(READ_CHAR_UUID.lower())
        logger.info(y_char)
        logger.info(y_char.service_uuid)   
        
        await client.start_notify(READ_CHAR_UUID, notification_handler)
        #await client.write_gatt_char(WRITE_CHAR_UUID, bytearray([0xe,0,0,0,0xff,0xff,0xff,0xff]))
        await client.write_gatt_char(WRITE_CHAR_UUID, bytearray([0x21,0,0,0,0,0,0,0]))
        await asyncio.sleep(0.2, loop=loop)
        await client.write_gatt_char(WRITE_CHAR_UUID, bytearray([0x0E,0,0,0,0,0,0,0]))
        await asyncio.sleep(15.0, loop=loop)
        await client.stop_notify(READ_CHAR_UUID)


        z = await client.disconnect()
        logger.info("Disconnected: {0}".format(z))
        await asyncio.sleep(1.0, loop=loop)



async def run(loop, debug=False):
    async def found_unit(dev):
        await asyncio.sleep(1.0, loop=loop)
        async with bleak.BleakClient(dev.address, loop=loop) as client:
            svcs = await client.get_services()
            print("Services:", svcs.services)

    if debug:
        import sys

        # loop.set_debug(True)
        l = logging.getLogger("asyncio")
        l.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        l.addHandler(h)
        logger.addHandler(h)

    async with bleak.BleakScanner(loop=loop) as scanner:
        x = await scanner.discover()
        logger.info("*********** Start of discovered list *************")
        for d in x:
            logger.info(d)
            logger.info(bleak.utils.mac_str_2_int(d.address))
            if d.address == TARGET_ADDRESS:
                await found_unit(d)
        logger.info("*********** End of discovered list ***************")
        await asyncio.sleep(1.0, loop=loop)

if __name__ == "__main__":
    import os

    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    loop = asyncio.get_event_loop()
    #loop.run_until_complete(run(loop, True))
    loop.run_until_complete(run_specific_device(loop, TARGET_ADDRESS, True))
