import serial
import time
ser = serial.Serial('COM15', baudrate=9600)
ser.write(bytes([255, 255]))
time.sleep(5)
ser.close()