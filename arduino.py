import serial
# from serial import serial
import time
import io
import numpy as np
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)
def write_data(angles):
    string=f"{angles[0]},{angles[1]}\n"
    arduino.write(bytes(string,'utf-8'))
    arduino.flush()
    
while True:
    num1 = input("Enter a number: ") # Taking input from user
    num2 = input("Enter a number: ") # Taking input from user
    
    value = write_data(np.array([num1,num2]))
    print("PRINTED:",value) # printing the value