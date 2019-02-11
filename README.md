# CC2640 Home Automation

This project is an application for building Bluetooth low energy (BLE) sensors
and actuators based on the TI [CC2640](http://www.ti.com/product/CC2640) and
[CC2650](http://www.ti.com/product/CC2650) SoC.

Currently, the supported modules include:
- Sensors:
  - Button / switch
  - Reed switch
  - [OPT3001](http://www.ti.com/product/OPT3001) ambient light sensor
  - [BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280)
    enviromental sensor (pressure, temperature and humidity)
  - Thermistor temperature sensor
  - PIR motion sensor
- Actuators:
  - Relay and latching relay
  - LED
  - WS2812B addressable RGB LEDs

## Services and Characteristics

Depending on the modules enabled in the [board file](#configuration), the
following GATT services and characteristics may be exposed:

- `0000180f-0000-1000-8000-00805f9b34fb` **Battery Service**
  - `00002a19-0000-1000-8000-00805f9b34fb` **Battery Level** (read, notify)
    - A uint8 in percent describing the battery level
- `0000181a-0000-1000-8000-00805f9b34fb` **Environmental Sensing**
  - `00002a6d-0000-1000-8000-00805f9b34fb` **Pressure** (read, notify)
    - A uint32 in pascals with a resolution of 0.1 Pa
  - `00002a6e-0000-1000-8000-00805f9b34fb` **Temperature** (read, notify)
    - A sint16 in degrees Celsius with a resolution of 0.01 degrees Celsius
  - `00002a6f-0000-1000-8000-00805f9b34fb` **Humidity** (read, notify)
    - A uint16 in percent with a resolution of 0.01 percent
  - `1e352f41-1217-5887-a357-c9acbd789b14` **Luminance** (read, notify)
    - A uint32 in Lux with a resolution of 0.01 Lux
- `1e352f00-1217-5887-a357-c9acbd789b14` **Relay Service**
  - `1e352f01-1217-5887-a357-c9acbd789b14` **Relay State** (read, write, notify)
    - A boolean value for getting and setting the relay state (on / off)
- `1e352f10-1217-5887-a357-c9acbd789b14` **Switch Service**
  - `1e352f11-1217-5887-a357-c9acbd789b14` **Switch state** (read, notify)
    - A boolean value describing the switch state (on / off)
- `1e352f20-1217-5887-a357-c9acbd789b14` **Contact Sensor Service**
  - `1e352f21-1217-5887-a357-c9acbd789b14` **Contact Sensor State** (read, notify)
    - A boolean value describing the contact sensor state (open / closed)
- `1e352f30-1217-5887-a357-c9acbd789b14` **Motion Sensor Service**
  - `1e352f31-1217-5887-a357-c9acbd789b14` **Motion Sensor State** (read, notify)
    - A boolean value describing the motion sensor state (open / closed)
- `1e352f50-1217-5887-a357-c9acbd789b14` **RGB LED Service**
  - `1e352f51-1217-5887-a357-c9acbd789b14` **RGB LED Color** (read, write, notify)
    - An array of 3 uint8 representing the R, G and B values of each LED,
      repeated per LED

The services and characteristic configuration for use with
[esp32-ble2mqtt](https://github.com/shmuelzon/esp32-ble2mqtt) is available at
[ble2mqtt\_configuration.json](ble2mqtt_configuration.json)

## Configuration

There are two steps for configuration this project. The first is defining the
SoC variant and the second is configuring the peripherals.

The SoC variant is defined the project's predefined symbols using TI's notation,
one of: CC2650\_LAUNCHXL, CC2650DK\_7ID, CC2650RC, CC2650STK, BOOSTXL\_CC2650MA,
CC2650DK\_5XD or CC2650DK\_4XS

Configuring the peripherals is done the
[app/Startup/board.h](app/Startup/board.h) file.  There are macros for each
peripheral to define the relevant GPIOs or I2C address. GPIO values use TI's
`IOID_X` macros where `X` is the GPIO number.  Use `PIN_UNASSIGNED` for GPIOs
or `0` for I2C addresses when the
peripheral is not used.

Below is a table summarizing the different peripheral options:

| Peripheral | Macros | Type |
| ---        | ---    | ---   |
| Battery | `Board_BATT_MIN`, `Board_BATT_MAX` | Number (mV), number (mV) |
| Switch | `Board_SWITCH` | GPIO |
| Button | `Board_BUTTON` | GPIO |
| Contact sensor | `Board_CONTACT` | GPIO |
| OPT3001 | `Board_OPT3001_ADDR` | I2C address\* |
| BME280 | `Board_BME280_ADDR` | I2C address\* |
| Thermistor | `Board_THERMISTOR_POWER`, `Board_THERMISTOR` | GPIO (optional), ADC |
| Motion | `Board_MOTION` | GPIO |
| Relay | `Board_RELAY_SET`, `Board_RELAY_RESET` | GPIO, GPIO (optional) |
| LED | `Board_LED1`, `Board_LED_ON`, `Board_LED_ON` | GPIO, logic level, logic level ||
| WS2812B | `Board_WS2812_NUM_LEDS`, `Board_SPI0_MOSI` | Number, GPIO (data line) |

\* When using I2C devices, you should also define `Board_I2C0_SDA0` and
`Board_I2C0_SCL0` accordingly.

#### Increasing TX Power

When the device is running of mains power (`EXCLUDE_BATT` is defined) or when
the `MAX_TX_POWER` preprocessor symbol is defined, the peripheral will transmit
using the maximum power setting (either +2 or +5 dBm).

#### Security
By default, authentication is enabled requiring out-of-band pairing using a
passkey before accessing one of the sensitive characteristics. If needed, this
can be disabled by defining the `DISABLE_AUTHENTICATION` preprocessor symbol.

## Compiling

The project is compiled and tested with [TI's Code Composer
Studio](http://www.ti.com/tool/CCSTUDIO) version 8.0.0. In addition, you need
to install, in the default directory (C:\\ti), TI's [BLE
stack](http://www.ti.com/tool/BLE-STACK) version 2.2.2.

First, build the stack project and then the app project. Once both images are
built, execute the [create\_image.py](create_image.py) script. This script will
create a unified image and also inject a random passkey. Make note of this
passkey as you will need it to perform out-of-band pairing with the peripheral.

**Note**: If you can't or do not wish to make use of this script, you will need
to set the passkey manually in
[app/Application/home\_automation.c](app/Application/home_automation.c), look
for `0xdeadbeef` and replace that value with your chosen passkey. Then flash
both the stack and app images on to your device.

## Flashing

Flashing the CC2640 / CC2650 SoC can be done with the various XDS emulators,
e.g. XDS100v3. Once connected, use TI's [SmartRF Flash Programmer
2](http://www.ti.com/tool/FLASH-PROGRAMMER) to load the image.

**Note**: When flashing the stack and app images separately, make sure to erase
only the sections in the image file and not the entire flash memory.
