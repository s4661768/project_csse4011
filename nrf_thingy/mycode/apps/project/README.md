# CSSE4011 Project

## Connor Bilzon 45887837

### AHU Functionality:
#### blecon -s <BLE ADDR> fully functional
#### blecon -p fully functional

### Thingy:52 Functionality:
#### iBeacon broadcasting fully functional
#### iBeacon receiving bluetooth packets fully functional
#### iBeacon filtering bluetooth packets fully functional

### Folder Structure: AHU
#### main.c: nrf_thingy/mycode/apps/project/src/main.c
#### included files for AHU, Bluetooth, HCI and GCU commands: project/mycode/mylib/..

### Folder Structure: Thingy:52
#### main.c: project/mycode/apps/Thingy:52/src/main.c
#### ibeacon.c: project/mycode/mylib/Thingy:52/sensors/bluetooth/ibeacon.c

### Folder Structure: GUI
#### gui.py: project/mycode/apps/gui/gui.py

### References:
#### blinky.c
#### echo-bot.c
#### observer.c
#### ibeacon.c

### User Instructions: nrf52840dk
#### Building: west build -b nrf52840dk_nrf52840 ../<location to src folder> --pristine
#### Building with Style Checker: scan-build west build -b nrf52840dk_nrf52840 ../<location to src folder> --pristine
##### Note: must have clang-tools to use scan-build.
#### Flashing: west flash 

### User Instructions: Thingy:52 Board
#### Building: west build -b thingy52_nrf52832 ../<location to bluetooth folder> --pristine
#### Building with Style Checker: scan-build west build -b thingy52_nrf52832 ../<location to bluetooth folder> --pristine
##### Note: must have clang-tools to use scan-build.
#### Flashing: west flash
