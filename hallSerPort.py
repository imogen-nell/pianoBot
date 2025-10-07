#!/usr/bin/python3
import serial # type: ignore
import time
from enum import Enum
from queue import Queue
from threading import Thread
from pathlib import Path

class readState(Enum):
    READING = 1
    WAITING = 2
    ERROR = 0
        
class HallSensor:
    def __init__(self, port= '/dev/ttyACM0', baud_rate=115200):
        self.serialPort = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize
        self.state = readState.READING
        self.data  = Queue()
        
        self.read_thread = Thread(target=self.read_data, args=(self.serialPort,))
        #get unique filename
        filename = self.get_file_name()
        self.save_thread = Thread(target=self.save_data, args=(filename,))
        
        self.save_thread.start()
        self.read_thread.start()
        
    def read_data(self, serialPort):
        while True:
            try:
                line = serialPort.readline().decode(errors='ignore').strip()
                if line:
                    self.state = readState.READING
                    self.data.put(line)#, time.time()))
                else:
                    self.state = readState.WAITING
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.state = readState.ERROR
            
    def save_data(self, filename):
        while True:
            with open(filename, 'a') as f:
                while not self.data.empty():
                    entry = self.data.get()
                    # file auto closed and saved here
                    f.write(f"{entry[1]}, {entry[0]}\n")
                    f.flush()
                time.sleep(0.1)


    def close(self):
        self.serialPort.close()
        
    def get_file_name(self):
        # Generate a unique filename based on the current timestamp
        filename = "hall_data.txt"
        file_path = Path(filename)
        stem = file_path.stem      # filename without extension
        suffix = file_path.suffix  # extension (e.g. ".txt")

        counter = 1
        new_path = file_path

        # keep incrementing until we find a name that doesn't exist
        while new_path.exists():
            new_name = f"{stem}_{counter}{suffix}"
            new_path = file_path.with_name(new_name)
            counter += 1

        return str(new_path)

def main():
    hall = HallSensor()
    
if __name__ == "__main__":
    main()
