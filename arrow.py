import keyboard
import serial



max_x = 190
neutral_x = 95
min_x = 0

max_y = 190
neutral_y = 95
min_y = 0


ser = serial.Serial('COM15', baudrate=9600)

serial_x = neutral_x
serial_y = neutral_y
while True:
    if keyboard.is_pressed('w'):
        serial_y = max_y
    elif keyboard.is_pressed('s'):
        serial_y = min_y
    else:
        serial_y = neutral_y
    if keyboard.is_pressed('d'):
        serial_x = max_x
    elif keyboard.is_pressed('a'):
        serial_x = min_x
    else:
        serial_x = neutral_x
    if keyboard.is_pressed('q'):
        print("q")
        break
    ser.write(bytes([serial_x, serial_y]))
    print(str(serial_x) + ", " + str(serial_y))
ser.write(bytes([neutral_x, neutral_y]))
ser.close()