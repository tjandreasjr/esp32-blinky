# ESP32 Blinky

Program receives input over USB serial port to control the blue user LED. LED may be turned on, turned off, or set to blink at a fixed frequency. State of the LED will persist after reset or power cycle.

## HW/SW Environment

### Hardware Used

ESP32-WROOM-32D module
micro-USB cable

### Software

Espressif IDE for Windows v2.4.2
ESP-IDE v4.4.1
python v3.7.4
pySerial v3.5

## Quick setup

Open project in Espressif IDE and select Build.
Connect ESP32 to USB port. Select Run to download program to the ESP32.

The ESP32 is now ready to receive commands over the USB serial port. Supported commands are:

| Command | Description      |
| ------- | ---------------- |
| 'h'     | set LED pin High |
| 'l'     | set LED pin Low  |
| 'b'     | Blink the LED    |

The included python module may be used, or the command bytes may be sent using any terminal program such as PuTTY.

### Using python module on host PC

Check that python v3.7 or later is installed. If necessary, install pySerial package by running the following at an administrator command prompt:

```
python -m pip install pyserial
```

With the ESP32 connected to your system, open python/esp32_blinky.py and change the UART_COM_PORT parameter if necessary to match the ESP32 serial port's name on your system.

To use the python module, run from the command prompt after navigating to the python/ directory:

```
>python
>>> from esp32_blinky import * # Open ESP32 serial port
>>> ledOn() # turns LED on
>>> ledOff() # turns LED off
>>> ledBlink() # blinks the LED
>>> quit() # quit Python, close serial port
```
