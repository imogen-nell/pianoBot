#!/usr/bin/python3

import serial
import time

SERIAL_PORT = '/dev/ttyACM0'  # Update this to your serial port
BAUD_RATE = 115200

def main():
    try:
        hall = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return
    
    while True:
        line = hall.readline().decode(errors= 'ignore').strip()
        if not line:
            print("no line")
            continue
        print(f"Received: {line}")

if __name__ == "__main__":

	main()
