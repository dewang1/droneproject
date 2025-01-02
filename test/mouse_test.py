import pyautogui
import keyboard
import serial

def get_mouse_position():
    x, y = pyautogui.position()
    return x, y

max_x = 255
neutral_x = 125
min_x = 0

max_y = 255
neutral_y = 125
min_y = 0

x_pin = 5
y_pin = 4

ser = serial.Serial('COM15', baudrate=9600)

def map_value(value, in_min, in_max, out_min, out_max):
    # Map value from input range to output range
    return round((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:

    x, y = get_mouse_position()
    serial_x = max(min_x, min(map_value(x, 0, 1919, min_x, max_x), max_x))
    serial_y = max(min_y, min(map_value(y, 0, 1079, max_y, min_y), max_y))
    print(str(serial_x) + ", " + str(serial_y))
    ser.write(bytes([serial_x, serial_y]))
    if keyboard.is_pressed('q'):
        print("You pressed 'q'")
        break
ser.write(bytes([neutral_x, neutral_y]))
ser.close()