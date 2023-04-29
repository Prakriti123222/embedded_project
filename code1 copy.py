import serial
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial('COM4', 115200, timeout=1)
a = 0
arr = []
while (a < 600):
    print("Sample number ", end="")
    print(a, end=" ")
    print(" : ", end="")
    # Read analog value from STM32
    data = ser.readline().decode()
    a = a+1
    if (data == b''):
        value = 0
    else:
        # Convert data to analog value and print
        try:
            value = float(data)
        except:
            value = 0
    # analog_value = value * 3.3 / 4095
    # print(value)
    arr.append(value/10.0)
    print(value/10.0)
print(arr)