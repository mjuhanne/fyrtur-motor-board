# IKEA Fyrtyr roller blind custom firmware

**Table of Contents**

[TOC]

## Introduction

This is a custom firmware for the TQL25-KT972 motor module (powered by the STM32F030K6T6 ARM microcontroller) used by IKEA Fyrtyr and Kadrilj roller blinds.

It mimics the functionality of the original firmware with following additions:
 * **Allow setting custom motor speed**: The full speed with original FW is a bit too noisy for my ears, especially when used to control morning sunlight in the bedroom. Now, it's possible to set the speed to 3 RPM and enjoy the completely silent operation, waking up to the sunlight instead of whirring noise :)
 * **Allow the use of 5-6 volt DC source:** Original firmware was intended to be used with chargeable battery which was protected from under-voltage by ceasing operation when voltage drops below 6 VDC. Our custom firmware instead is intended to be used with [custom Fyrtyr Wifi Module](https://github.com/mjuhanne/fyrtur-esp) (plugged to DC adapter) so the low voltage limit for motor operation is ignored, mitigating the need to shop for the harder-to-get 6-7.5 volt adapters.
 * **Allow finer curtain position handling:** Original firmware has 1% granularity for the curtain position (0% - 100%). This translates to 1.5 cm resolution when using blinds with 1.5m tall window. It doesn't sound much, but when using sunlight-blocking curtain a lot of sunlight can seep between lower curtain and window board from a 0.5-1.5 cm gap. The custom firmware allows setting target position with sub-percent resolution so curtain can be lowered more accurately.

Although the new firmware can be used with original Ikea Fyrtur wireless (Zigbee) module, most of its additional features can only be used with [ESP8266/ESP32 based module using WiFi and MQTT connectivity](https://github.com/mjuhanne/fyrtur-esp)

## Wiring and interface

The motor is connected with 4 wires (TX, RX, GND, VCC) where TX and RX are UART lines operating at 2400 kbps  (8N1 = 1 start bit, 8 data bits,no parity, 1 stop bit) using 3.3V logic high. VCC is nominal 5.0-7.4 V.

## Motor board reverse engineering

*pcb-reverse-engineering/* folder contains very rough schematic of the motor board in PDF and KiCad project format. 

## Command structure

All the commands consist of 6 bytes and must follow this pattern:

`(0x00 0xff 0x9a) DATA1 DATA2 CHECKSUM`

Where first 3 bytes are the header, DATA1 and DATA2 are the command/argument bytes and CHECKSUM is a bitwise XOR of the DATA1 and DATA2 bytes.

#### Normal commands

#####CMD_GO_TO
`00 ff 9a dd XX CHECKSUM`
- Moves into position XX (between 0 and 100). 
- Checksum has to be calculated as described above.

#####CMD_UP
`00 ff 9a 0a dd d7`
- Starts winding up the blinds continously
- This continues until resistance is met (See Position chapter below) Custom firmware also stops when position 0 is reached.

#####CMD_DOWN
`00 ff 9a 0a ee e4`
- Starts winding down the blinds continously (until maximum length is reached. See Position chapter below)

#####CMD_UP_17
`00 ff 9a 0a 0d 07`
- Rotates up the curtain rod 17 degrees (around 0.60cm). 
 - Note that in the original firmware this will for some reason convert to continous up movement (CMD_UP) if used when motor position is 0!
 - In custom firmware it will not move beyond the upper limit however.

#####CMD_DOWN_17
`00 ff 9a 0a 0e 04`
- Rotates down the curtain rod 17 degrees (around 0.60cm). It will not move beyond lower limit however.

#####CMD_STOP
`00 ff 9a 0a cc c6`
- Stops the continous movement

#####CMD_STATUS
`00 ff 9a cc cc 00`
- Polls the motor module for status update.

The motor module response consists of 8 bytes and follows this pattern:

`0x00 0xff 0xd8 BATTERY_LEVEL VOLTAGE SPEED POSITION CHECKSUM`
- The first 3 bytes is the header
- BATTERY_LEVEL, VOLTAGE and POSITION are described below
- SPEED is motor speed in revolutions per minute (rpm)
- CHECKSUM is a bitwise XOR of the (BATTERY_LEVEL,VOLTAGE,SPEED,POSITION) bytes. 


**Example**: `00 ff d8 16 d8 19 32 e5`
- Header (0x00 0xff 0xd8)
- Battery level 0x16 (30%(?))
- Voltage around 7.0 Volts
- Speed 0x19=25 RPM (full speed)
- Position 0x32=50
- Checksum 0xe5


#### Overriding move commands

These can move blinds past the soft or hard limits (see Position chapter below)

#####CMD_UP_90
`00 ff 9a fa d1 2b`
- Rotates the curtain rod up by 90 degrees (around 3cm)

#####CMD_DOWN_90
`00 ff 9a fa d2 28`
- Rotates the curtain rod down by 90 degrees (around 3cm)

#####CMD_UP_6
`00 ff 9a fa d3 29`
- Rotates the curtain rod up by 6 degrees (around 0.22cm)

#####CMD_DOWN_6
`00 ff 9a fa d4 2e`
- Rotates the curtain rod down by 6 degrees (around 0.22cm)


#### Commands involving lower limits 
See Position chapter below for more information

#####CMD_SET_SOFT_LIMIT
`00 ff 9a fa ee 14`
- Sets the lower curtain limit (pos=100) to the current curtain position.

#####CMD_SET_HARD_MAXIMUM
`00 ff 9a fa cc 36`
- Sets the hard maximum lower limit (pos=100) to current curtain position.

#####CMD_RESET_SOFT_LIMIT
`00 ff 9a fa 00 fa`
- Reset the soft lower limit to the previously hard maximum limit. Motor also loses its current position and must be reset by winding up the curtaing and engaging upper hard stop.


#### Custom firmware commands

#####CMD_EXT_GO_TO
`00 ff 9a 1X YZ CHECKSUM`
- "Go to position" command with finer resolution (position is given between 0.0 and 100.0). 
- Target position is given with XY.Z (lower 4 bits of the 1st byte + 2nd byte  = 12 bits total) where XY is the decimal part and Z is the fractional part.
  - POS = (X*256 + YZ) / 16
- Checksum has to be calculated as with CMD_GO_TO.

#####CMD_SET_SPEED
`00 ff 9a 20 XX CHECKSUM`
- Set the motor speed to XX (in RPM). Normal values are between 1 and 25 RPM.

#####CMD_GET_EXT_STATUS
`00 ff 9a cc de CHECKSUM`
- Get extended status bytes.

The motor module response consists of 8 bytes and follows this pattern:

`0x00 0xff 0xda MODULE_STATUS MOTOR_CURRENT POSITION_DEC POSITION_FRAC CHECKSUM`
 - The First 3 bytes is the header
 - MODULE_STATUS (0=Stopped, 1=Moving, 2=Error)
 - MOTOR_CURRENT (in mA).
 - POSITION_DEC and POSITION_FRAC report the higher resolution position info
  - POS = POSITION_DEC + POSITION_FRAC/256).
 - CHECKSUM is a bitwise XOR of the (MODULE_STATUS,MOTOR_CURRENT,POSITION_DEC,POSITION_FRAC) bytes.

#####CMD_GET_EXT_LIMITS
`00 ff 9a cc df CHECKSUM`
- Get extended limits/resetting data

The motor module response consists of 8 bytes and follows this pattern:

`0x00 0xff 0xbb RESETTING SLL_1 SLL_2 HLL_1 HLL_2 CHECKSUM`
- The first 3 bytes is the header
- RESETTING (0=Normal operation, 1=In resetting mode. Position is always reported as 50)
- SLL = Soft Lower Limit = SLL_1 * 256 + SLL_2
- HLL = Hard Lower Limit = HLL_1 * 256 + HLL_2
- CHECKSUM is a bitwise XOR of the (RESETTING,SLL_1,SLL_2,HLL_1,HLL_2) bytes. 

## Position

Normally the motor unit reports its position as percentage of maximum operating distance (0 for upper limit and 100 for lower limit).

When first powered, the motor doesn't know it's position and it begins winding the blinds upwards (in original FW. Customizable in our FW).
During this time position byte stands at 100. Rewinding continues until sufficient resistance occurs (motor is stalling) and now the motor detects the 
curtain has been lifted to upmost position. Position is now reset to 0.

When winding down the motor automatically stops when default maximum length (13 turns + 265 degrees) is been reached (HARD LOWER LIMIT).

Motor can be commanded to move above (below) the upper (lower) limit with overriding move commands. In this case the motor will keep track of its position with internal counters. 
It will however NOT describe the position in STATUS message, but position is truncated between 0 and 100.

Maximum curtain length can be shortened by using CMD_SET_SOFT_LIMIT / CMD_SET_HARD_LIMIT commands. These will set the current motor position to 100 and will restrict normal movements below this limit. Reported motor position data is now scaled accordingly.

Soft lower limit can be reset via CMD_RESET_SOFT_LIMIT command. This will cause motor to lose its position and allow uninhibited movement
to all directions. Position will be reported as 50 regardless of motor movement until blinds are winded up to hard stop (mechanical resistance), 
after which motor considers to be in upmost position. Position now is now set to be 0 and normal operation can continue as described above.

Difference between hard and soft limits is that soft limits are limited between 0 and hard limit (by default 13 turns+265 degrees), whereas hard limits
are unrestricted in this way. Also when soft limit is reset, it's value is set to be the previously set hard limit. In original firmware, hard limits seem to withstand loss of power (most likely written to EEPROM/FLASH). This is currently unsupported in our custom firmware.
Since hard limits cannot be restored to any default value, to extend the hard limit further one can only use overriding movement commands to lower the blinds and then invoke CMD_SET_HARD_LIMIT again.

## Battery and Voltage bytes

Battery level: (0x00=empty, >0x45 = full)
Voltage follows roughly the formula: V = (voltage byte) / 31.

Below are the results of battery and voltage bytes from original firmware, experimented with external DC voltage source using various voltage levels (did not want to go above 7.5..)
```
volt 3.7: 0x00 0x7c 
volt 4.5: 0x00 0x89
...
volt 4.7: 0x00 0x90 
volt 5.0: 0x00 0x99
volt 5.5: 0x00 0xa9
volt 5.6: 0x00 0xac 
..
volt 6.0: 0x00 0xb8 
volt 6.4: 0x00 0xc5 
volt 6.5: 0x01 0xc8 
volt 6.8: 0x08 0xd2 
volt 7.0: 0x12 0xd8
volt 7.1: 0x20 0xdb
volt 7.2: 0x2e 0xde
volt 7.3: 0x38 0xe1
volt 7.4: 0x40 0xe4
volt 7.5: 0x45 0xe7 
```

With original firmware, the motor operates with voltages such as 6.0-6.4 volts even though battery is reported to be empty.
Below 6V the motor itself would not engage, but the motor module still responds to status queries until around 3.7V is reached.

Our custom firmware reports the voltage correctly, but battery byte for now is fixed 0x12 (matching 7.0V)

## Calibration in practice

- If motor is powered up and doesn't know its position, it begins windind up until hard stop is met (see Position chapter above). After this the current motor position is reset to 0. Also the previously set lower limit is now enforced. This "upper limit reset" is done every time motor is winded up and it stalls.
- To set lower limit, wind down to suitable position and call CMD_SET_SOFT_LIMIT (position is now reset to 100)
- To adjust soft lower limit, move to suitable position and call CMD_SET_SOFT_LIMIT again. If this desirable position is outside current limits, we have to use the "Overriding" move commands to navigate the rest of the way.
- To reset the soft limit, use CMD_RESET_SOFT_LIMIT command and wind up the motor to upper hard stop. Now position is reset to 0 and previously set soft limit is replaced with hard maximum limit.
- To adjust the hard limit, navigate to desired position with "Overriding" move commands and call CMD_SET_HARD_MAXIMUM. This will change also the soft limit to the same value.

---

## Installation

**Note that disassembling the motor unit will most definitely void your warranty!**

**Please first dump the original FW as below before proceeding to install custom firmware!**

**This software is provided "as is". Use with caution and at your own risk! (See LICENSE file for DISCLAIMER)**

### Software installation
For compiling the sources, install [STM32CubeIde](https://www.st.com/en/development-tools/stm32cubeide.html) (I used version 1.4.2) Alternatively, you can use binaries found in bin/ folder.

### Disassembly
- Unscrew the 3 small screws on the plastic enclosure
- Carefully lift the small aluminum tabs holding the alumimum tube in its place
- (No need to disassemble the other side with the motor)
- Motor driver module can be now detached from the tube/motor side.

### Firmware dumping with ST-Link V2:

- connect SWD wires to IO1 header: GND, RST, SWCLK, SWDIO, 3V3 (3V3 is the square pin hole)
- External power (>= 4.0V) is required via 7.4V pin
- Launch STM32 ST-LINK Utility
- Edit Target->Settings: 
    Connection Settings: SWD, Frequency 480 Khz. Mode: Connect Under Reset, Debug in Low Power enable )
    Reset Mode: Hardware reset
- Connect to the target
- (if connecting fails, re-check wiring. If connection to ST-Link seems finicky, try upgrading the ST-Link firmware)
- Memory display: Address 0x08000000  with Size 0x8000 and Data width 32 bits (so total 32 KB flash size)
- You should see the firmware now displayed. Save it to a binary file and backup it.


### (Custom) firmware installation with ST-Link V2:
 - proceed with steps described above
 - Click Target->Program & Verify
 - Select custom/original firmware file (.bin)
 - Make sure start address is 0x08000000
 - Skip Flash Erase and Skip Flash Protection verification can be set
 - Hit Start!






