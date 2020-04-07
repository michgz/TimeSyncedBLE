# TimeSyncedBLE

A system for taking syncronised accelerometer measurements with a network of BLE (Bluetooth Low-Energy) nodes in a way that all measurements are time-stamped to better than 1 millisecond accuracy. It builds on the time-syncing system pioneered by Audun Korneliussen (https://github.com/nordic-auko).

The nodes are implemented on nRF52 SOC (for example, nRF52832) and uses the ST accelerometers LIS2DH. A controller nRF52 provides the time sync to all leaf nodes, and also acts as a relay for getting the data back to a host PC.


 +------+        +------+
 |      |        |      |
 | PC   |  --z   |nRF52 | --z
 |      |    z-- |      |   z-- ( ..   multiple nodes ... )
 +------+        +------+       


Several sources of code have been borrowed and used, and are hereby acknowledged:

* Time-sync framework from https://github.com/nordic-auko/nRF5-ble-timesync-demo. More details are given in the blogs here https://devzone.nordicsemi.com/nordic/short-range-guides/b/bluetooth-low-energy/posts/wireless-timer-synchronization-among-nrf5-devices
* Relay code for the "controller" to relay between the leaf nodes and the PC taken from an example in the Nordic SDK examples\ble_peripheral_and_central\experimental\ble_app_relay_hrs_rscs.
* PC host code derived from Raymond Chen's samples at https://github.com/microsoft/Windows-universal-samples/tree/win10-1607/Samples/BluetoothLEClient/

## Development Frameworks

Embedded code for the nRF52 units uses the 16.0.0 version of the nRF52 (download from https://www.nordicsemi.com/Software-and-tools/Software/nRF5-SDK/Download). It should be compilable by SES. Note that the SDK is not included in this repository -- it must be downloaded and copied in. There is an empty directory there for it.

PC host code was developed in Visual Studio 2019 and uses the Universal Windows framework. This unfortunately means it will only run on Windows 10 (Linux and earlier versions of Windows are not suitable). There are only a few functions that really require Universal Windows, such as scanning and connecting to a chosen unit which can just be done within the OS. For that reason, I'd like to migrate it to a different framework.


