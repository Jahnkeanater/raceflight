# Board - CC3D

The OpenPilot Revolution aka Revo is a board more tuned to Acrobatic flying or GPS based
auto-piloting.  It has three sensors, the MPU6000 SPI based Accelerometer/Gyro, the MS5611 barometric pressure sensor and the and the Honeywell HMC5883L three axis digital compass module. 
It also features a 2Mbit SPI based EEPROM chip.  It has 8 ports labeled as inputs (one pin each)
and 6 ports labeled as motor/servo outputs (3 pins each).

It uses an STM32F4 MCU with 1 MB of flash storage , 192 Kb of RAM and runs at 168 MHz.

If issues are found with this board please report via the [github issue tracker](https://github.com/cleanflight/cleanflight/issues).

The board has a USB port directly connected to the processor.  Other boards like the Naze and Flip32
have an on-board USB to uart adapter which connect to the processor's serial port instead.

All Cleanflight support is untested, please report success or failure if you try it. 



////////////// All below is untocuhed from CC3D documentation.
# Pinouts

The 10 pin Flexi_IO connector has the following pinouts when used in RX_PPM/RX_PARALLEL_PWM mode

| Pin | Function  | Notes                            |
| --- | --------- | -------------------------------- |
| 1   | Ground    |                                  |
| 2   | +5V       |                                  |
| 3   | | | 
| 4   | | |
| 5   | PWM input 1 / PPM input 1-8 | Enable `feature RX_PPM` | 
| 6   | PWM input 2 | |
| 7   | PWM input 3 / USART6 Tx | |
| 8   | PWM input 4 / USART6 RX | |
| 9   | PWM input 5 | |
| 10  | PWM input 6 | |


The 6 pin RC_Output connector has the following pinouts when used in RX_PPM/RX_PARALLEL_PWM mode

| Pin | Function  | Notes |
| --- | ----------| ------|
| 1   | MOTOR 1   |       |
| 2   | MOTOR 2   |       |
| 3   | MOTOR 3   |       |
| 4   | MOTOR 4   |       |
| 5   | LED Strip |       |
| 6   | Unused    |       |

# Serial Ports

| Value | Identifier   | Board Markings | Notes                                     |
| ----- | ------------ | -------------- | ------------------------------------------|
| 1     | VCP          | USB PORT       |                                           |
| 2     | USART1       | MAIN PORT      | Connected to an MCU controllable inverter |
| 3     | USART3       | FLEX PORT      |                                           |
| 4     | USART6       | Flexi-IO       | Pins 7 and 8 (Tx and Rx respectively)     |

USART6 is not available when RX_PARALLEL_PWM is used.

To connect the GUI to the flight controller you just need a USB cable to use the Virtual Com Port (VCP) or you can use USART3 (FLEX Port 9600 baud).

# Main Port

The main port is connected to an inverter which is automatically enabled as required.  For example, if the main port is used for SBus Serial RX then an external inverter is not required.

## Main port pinout

| Pin | Signal             | Notes                   |
| --- | ------------------ | ----------------------- |
| 1   | GND                |                         |
| 2   | VCC unregulated    |                         |
| 3   | USART1 TX          | 3.3v level              |
| 4   | USART1 RX          | 3.3v level (5v tolerant)|

# Flex Port

The Revo uses the same FlexiPort as the CC3D. The port can be used as either a UART or for I2C bus connectivity. It can be connected to serial devices like the OP GPS or any I2C device like the the EagleTree Airspeed expander module, ADCs, I2C ESCs and a lot more. It can also be used to connect Spektrum DSM2/DSMX Satellite to be used as receiver, or any other custom component interfacing with I2C or a serial connection including custom extension boards. Of course, it’s also possible to run a serial Telemetry link to the GCS over the FlexiPort.

The Flex port has MSP support enabled on it by default 9600 Baud.

The flex port will be enabled in I2C mode unless USART3 is used.  You can connect external I2C sensors and displays to this port.

You cannot use USART3 and I2C at the same time.

## Flex port pinout

| Pin | Signal             | Notes                   |
| --- | ------------------ | ----------------------- |
| 1   | GND                |                         |
| 2   | VCC unregulated    |                         |
| 3   | I2C SCL / UART3 TX | 3.3v level              |
| 4   | I2C SDA / UART3 RX | 3.3v level (5v tolerant)|

# Current/Sonar Port

This port can be configured to accommodate an Autopilot current sensor and a low cost Sonar sensor such as the HC-SR04. It can also be used as a general purpose input/output port or as a one or two channel analog input port.

## Current/Sonar port pinout

| Pin | Signal             | Notes                   |
| --- | ------------------ | ----------------------- |
| 1   | GND                |                         |
| 2   | VCC unregulated    |                         |
| 3   | Current | 3.3v level              |
| 4   | Voltage | 3.3v level (5v tolerant)|

# SWD Port

Serial wire debug port. This allows the use of cheap boards like the STM F4 Discovery as an in-circuit debugger to ease the firmware development.

## SWD port pinout

| Pin | Pin Description    |
| --- | ------------------ |
| 1   | GND                |
| 2   | NRS                |
| 3   | IO                 |
| 4   | CLK                |

# Flashing

There are two primary ways to get Cleanflight onto a CC3D board.

* Single binary image mode - best mode if you don't want to use OpenPilot.
* OpenPilot Bootloader compatible image mode - best mode if you want to switch between OpenPilot and Cleanflight.

## Single binary image mode.

The entire flash ram on the target processor is flashed with a single image.

The image can be flashed by using a USB to UART adapter connected to the main port when the CC3D is put into the STM32 bootloader mode, achieved by powering on the CC3D with the SBL/3.3v pads bridged.  

## OpenPilot Bootloader compatible image mode.

The initial section of flash ram on the target process is flashed with a bootloader which can then run the code in the
remaining area of flash ram.

The OpenPilot bootloader code also allows the remaining section of flash to be reconfigured and re-flashed by the
OpenPilot Ground Station (GCS) via USB without requiring a USB to uart adapter.

The following features are not available:
 * Display
 * Sonar

# Restoring OpenPilot bootloader

If you have a JLink debugger, you can use JLinkExe to flash the open pilot bootloader.

1. Run JLinkExe `/Applications/SEGGER/JLink/JLinkExe`
2. `device STM32F103CB`
3. `r`
4. `h`
5. `loadbin opbl.bin, 0x08000000`
6. `q`
7. Re-plug CC3D.

Here's an example session:

```
$ /Applications/SEGGER/JLink/JLinkExe 
SEGGER J-Link Commander V4.90c ('?' for help)
Compiled Aug 29 2014 09:52:38
DLL version V4.90c, compiled Aug 29 2014 09:52:33
Firmware: J-Link ARM-OB STM32 compiled Aug 22 2012 19:52:04
Hardware: V7.00
S/N: -1 
Feature(s): RDI,FlashDL,FlashBP,JFlash,GDBFull 
VTarget = 3.300V
Info: Could not measure total IR len. TDO is constant high.
Info: Could not measure total IR len. TDO is constant high.
No devices found on JTAG chain. Trying to find device on SWD.
Info: Found SWD-DP with ID 0x1BA01477
Info: Found Cortex-M3 r1p1, Little endian.
Info: FPUnit: 6 code (BP) slots and 2 literal slots
Info: TPIU fitted.
Cortex-M3 identified.
Target interface speed: 100 kHz
J-Link>device STM32F103CB
Info: Device "STM32F103CB" selected (128 KB flash, 20 KB RAM).
Reconnecting to target...
Info: Found SWD-DP with ID 0x1BA01477
Info: Found SWD-DP with ID 0x1BA01477
Info: Found Cortex-M3 r1p1, Little endian.
Info: FPUnit: 6 code (BP) slots and 2 literal slots
Info: TPIU fitted.
J-Link>r
Reset delay: 0 ms
Reset type NORMAL: Resets core & peripherals via SYSRESETREQ & VECTRESET bit.
J-Link>h
PC = 0800010C, CycleCnt = 00000000
R0 = 0000000C, R1 = 0000003F, R2 = 00000000, R3 = 00000008
R4 = 00003000, R5 = 023ACEFC, R6 = 200000F0, R7 = 20000304
R8 = 023B92BC, R9 = 00000000, R10= ED691105, R11= F626177C
R12= 000F0000
SP(R13)= 20000934, MSP= 20000934, PSP= 20000934, R14(LR) = FFFFFFFF
XPSR = 01000000: APSR = nzcvq, EPSR = 01000000, IPSR = 000 (NoException)
CFBP = 00000000, CONTROL = 00, FAULTMASK = 00, BASEPRI = 00, PRIMASK = 00
J-Link>loadbin opbl.bin, 0x08000000
Downloading file [opbl.bin]...
WARNING: CPU is running at low speed (8004 kHz).
Info: J-Link: Flash download: Flash download into internal flash skipped. Flash contents already match
Info: J-Link: Flash download: Total time needed: 0.898s (Prepare: 0.709s, Compare: 0.128s, Erase: 0.000s, Program: 0.000s, Verify: 0.000s, Restore: 0.059s)
O.K.
J-Link>q
$ 
```
