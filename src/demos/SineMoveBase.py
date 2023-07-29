#!/usr/bin/python3
import serial
import math
import time

def generate_sin_wave(frequency, duration):
    global port
    amplitude = 70.0  # You can adjust this to control the amplitude of the sine wave.
    sampling_rate = 10  # Adjust this value based on your requirements (samples per second).
    period = 1 / frequency
    num_samples = int(period * sampling_rate)

    for i in range(num_samples):
        t = i / sampling_rate
        value = amplitude * math.sin(2 * math.pi * frequency * t)
        port.write((str(int(value))+"\t").encode())
        time.sleep(1 / sampling_rate)

def sin_waves(frequency_range, duration):
    for frequency in frequency_range:
        print(f"Frequency: {frequency} Hz")
        generate_sin_wave(frequency, duration)

if __name__ == "__main__":
    global port
    frequency_range = [0.1, 0.2, 0.5, 1, 2, 4]  # Frequencies from 0.5 Hz to 10 Hz
    duration = 5  # Duration of each sinusoidal wave in seconds
    port = serial.Serial("/dev/ttyACM0", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
    sin_waves(frequency_range, duration)
    port.write(("0"+"\t").encode())
