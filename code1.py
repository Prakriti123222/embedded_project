import serial
import matplotlib.pyplot as plt
import numpy as np
import pickle

def get_data():
    arr = []
    print("Enter Name :", end=" ")
    name = input()
    print("Enter Age :", end=" ")
    age = input()

    arr.append(name)
    arr.append(age)
    return arr

def get_values():
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
        arr.append(value)
        print(value)
    return arr

arr2 = get_data()
with open('data1.pkl', 'wb') as q:
    pickle.dump(arr2, q)

arr = get_values()
with open('data.pkl', 'wb') as f:
    pickle.dump(arr, f)
