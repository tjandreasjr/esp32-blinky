import serial

UART_COM_PORT = 'COM3'
UART_BAUD_RATE = 115200

ser = serial.Serial(UART_COM_PORT, baudrate = UART_BAUD_RATE, timeout = 1)

def ledOn():
    ''' Turn on blue LED on ESP32 board. State persists after power cycle. '''
    ser.write(bytes('h', 'ascii'))

def ledOff():
    ''' Turn off blue LED on ESP32 board. State persists after power cycle. '''
    ser.write(bytes('l', 'ascii'))

def ledBlink():
    ''' Blink the blue LED on ESP32 board. State persists after power cycle. '''
    ser.write(bytes('b', 'ascii'))