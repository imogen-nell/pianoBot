#!/usr/bin/python3
import sys
import serial # type: ignore
import time
# from enum import Enum
from queue import Queue
from threading import Thread
from pathlib import Path
import keyboard 

# class readState(Enum): #currently unused
#     READING = 1
#     WAITING = 2
#     ERROR = 0
        
# Read serial data from the hall effect sensor and save to a file

class hallSensor:
    
    def __init__(self, port= 'COM7', baud_rate=115200, mode = "continuous"):
        """Initialize the hallSensor.

        Args:
            port (str, optional): The serial port to use. Defaults to 'COM7'.
            baud_rate (int, optional): The baud rate for the serial connection. Defaults to 115200.
            mode (str, optional): The mode of operation. Defaults to "continuous".
                continuous: continuously read and save data
                keyboard: read and save a single data point
        """
        self.serialPort = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud.")
        time.sleep(2)  # Wait for the serial connection to initialize
        
        
        self.halldata  = Queue() #unused if inm keyboard mode 
        self.pwmdata  = Queue()
        self.running = True # Control flag for threads, needed despite daemon if i want to start/stop threads cleanly while program is running
        self.mode = mode
        filename = self.get_file_name("hall.txt") #unique filename
        pid_filename = self.get_file_name("pwm.txt")
        print(f"Saving hall data to {filename}")
        print(f"Saving PWM data to {pid_filename}")
        #threads
        self.read_thread = Thread(target=self.read_data, args=(self.serialPort,), daemon=True) #daemon so it stops w program
        self.save_thread = Thread(target=self.save_data, args=(filename, self.halldata), daemon=True)
        self.save_pid_thread = Thread(target=self.save_data, args=(pid_filename, self.pwmdata), daemon=True)
        self.read_thread.start()
        self.save_thread.start()
        self.save_pid_thread.start()
        self.latest_hall_value = None
        
    def read_data(self, serialPort):
        """ Continuously read data from serial port and put in queue """
        # v_ref = 5.0 ##5.0
        # v_conv = v_ref / 4095.0  # convert voltage
        ##voltage converted in controller 
        while self.running:
            if self.mode == "continuous":
                try:
                    #add data to queue to be saved
                    line = serialPort.readline().decode(errors='ignore').strip()
                    # print(line)
                    if line[:4] == "Hall": #useful later when more sensors 
                        _, t, v = line.split(",")
                        # v = float(v) * v_conv
                        line = f"{t},{v}"
                        self.halldata.put(line)
                    elif line[:3] == "PID":
                        _, t, pid_out,pwm = line.split(",")
                        # v = float(v) * v_conv
                        line = f"{t},{pid_out},{pwm}"
                        self.pwmdata.put(line)
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    self.running = False
            elif self.mode == "keyboard":
                try:
                    #save avg of 10 values:
                    total = 0.0
                    for _ in range(10):
                        line = serialPort.readline().decode(errors='ignore').strip()
                        if line[:4] == "Hall":
                            _, t, v = line.split(",")
                            total +=float(v)  
                    self.latest_hall_value = total / 10
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    self.running = False


    def save_data(self, filename, data_queue = None):
        while self.running:#makesure thread stops when main program stops w keyboard interrupt
            if self.mode == "keyboard":
                try:
                    #get keyboard input
                    event = keyboard.read_event()
                    if event.event_type == keyboard.KEY_DOWN and int(event.name)<=9 :  
                        with open(filename, 'a') as f:
                            #no check that latest hall value exists - fine for now
                            f.write(f"{event.name}, {self.latest_hall_value}\n")
                except Exception as e:
                    print(f"Error saving data: {e}")
                    
            else: #continuous mode
                while self.running:
                    with open(filename, 'a') as f:
                        while not data_queue.empty():
                            entry = data_queue.get()
                            f.write(f"{entry}\n")
                        time.sleep(0.1)

    def close(self):
        self.running = False
        time.sleep(0.2)  # Let threads finish current loop
        self.serialPort.close()
        
    def get_file_name(self, filename ):
        # Generate a unique filename based on the current timestamp
        path = r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata"

        # file_path = Path(filename)
        file_path = Path(path) / filename
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
    mode = "continuous"
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        if mode not in ("keyboard", "continuous"):
            print("Defaulting to continuous.")
        else:
            print(f"Running in {mode} mode.")
    hall = hallSensor(mode=mode)
    #keep program alive to allow threads to run
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt: #ctrl+c to stop
        print("\nKeyboardInterrupt detected. Stopping...")
        hall.close()
        sys.exit(0)
    
if __name__ == "__main__":
    main()
