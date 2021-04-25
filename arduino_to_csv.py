import serial
import sys
import time

# port = "/dev/cu.usbmodem14401"
port = "/dev/ttyACM0"
baudrate = 115200

if len(sys.argv) == 1 or sys.argv[1][-4:] != ".csv":
    print("Usage: python arduino_to_csv.py filename.csv")
    exit(1)

filename = sys.argv[1]

s = serial.Serial(port, baudrate)

# Tell the Arduino to reset
s.reset_input_buffer()
s.write(b'r')


while True:
    line = s.readline().decode("utf-8")
    print(line, end="")

    if line[:5] not in ['Error', 'Reset']:
        with open(filename, 'a+') as f:
            f.write(line)
