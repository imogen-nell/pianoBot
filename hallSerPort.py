import serial
import time
from enum import Enum

class readState(Enum):
    READING = 1
    WAITING = 2
    ERROR = 0
        
class HallSensor:
    def __init__(self, port= '/dev/ttyACM0', baud_rate=115200):
        self.serialPort = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize
        self.state = readState.READING
        self.data  =[]

    def read_data(self):
        try:
            line = self.serialPort.readline().decode(errors='ignore').strip()
            if line:
                self.state = readState.READING
                self.save_data(line, time.time())
                return line
            else:
                self.state = readState.WAITING
                return None
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            self.state = readState.ERROR
            return None
    
    def save_data(self, data, time_stamp):
        #TODO: implement  data processing
        self.data.append((data, time_stamp))
    
    def save_dalalog(self, filename):
        with open(filename, 'w') as f:
            for entry in self.data:
                f.write(f"{entry[1]}, {entry[0]}\n")
        self.data = []

    def close(self):
        self.serialPort.close()
