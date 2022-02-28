# Tesla pre-AP EPAS steering firmware patch
Normally on Tesla vehicles without autopilot the gateway sends a message indicating that steering over CAN is disabled.

This tool patches the firmware to enable steering over CAN.

## important - read this first!
* flashing firmware can fail and brick your EPAS, while unlikely,  
  **do not flash something that you are not willing to pay to replace**
* [comma.ai panda](https://comma.ai/shop/products/panda-obd-ii-dongle)
  is used to communicate with EPAS over CAN
* requires connection the chassis CAN bus such that you are connected in parallel with the EPAopenpilotS
  (flashing does not happen through the gateway)
* before flashing a backup of the firmware is take from the EPAS to ensure that
  the firmware on your EPAS is the expected firmware and compatible

## setup
```sh
# install python packages
pip3 install -r requirements.txt
# clone openpilot into home directory to get panda repo
git clone https://github.com/commaai/openpilot.git ~/openpilot
```

## patch the firmware
connect [comma.ai panda](https://comma.ai/shop/products/panda-obd-ii-dongle) to EPAS and run:

```sh
# replace PYTHONPATH with location you cloned openpilot to
PYTHONPATH=~/openpilot ./patch.py
```

## restore original firmware
connect [comma.ai panda](https://comma.ai/shop/products/panda-obd-ii-dongle) to EPAS and run:

```sh
# replace PYTHONPATH with location you cloned openpilot to
PYTHONPATH=~/openpilot ./patch.py --restore
```

## how it works
The gateway in vehicles without autopilot constantly sends a `GTW_epasControl` message
(CAN address 0x101) with two signals we care about:
* `GTW_epasControlType` (3 bits)
  * 0 = INHIBIT
  * 1 = ANGLE
  * 2 = TORQUE
  * 3 = BOTH
* `GTW_epasLDWEnable` (1 bit)
  * 0 = DISABLE
  * 1 = ENABLE
(the gateway in vehicles without autopilot sets these signals to have a value of 0)

The electronic parking brake in vehicles without autopilot constantly sends a `EPB_epasControl`
message (CAN address 0x214) with one signal we care about:
* EPB_epasEACAllow (3 bits)
  * 0 = DISABLE
  * 1 = ENABLE
(the electronic parking brake in vehicles without autopilot sets this signal to have a value of 0)

The firmware patch replaces parsing these signals with loading the value 1
(after the checksum is validated).

Note that if you have no electronic parking brake you still need to send `EPB_epasControl` with
a good checksum and counter to prevent the EPAS from faulting.
