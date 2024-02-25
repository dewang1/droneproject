import cv2
import numpy as np
import serial
import math
import time

# PID constants
kp = 0.7  # Proportional gain
ki = 0.0026  # Integral gain
kd = 2.5  # Derivative gain

# Initialize PID errors and integral term
prev_x, prev_y = -1, -1
integral_x, integral_y = 0, 0

# Define drone's target
home = (320, 240)

max_x = 190
neutral_x = 95
min_x = 0

max_y = 190
neutral_y = 95
min_y = 0

map = 9

# Define lower and upper bounds for color_a in HSV color space
lower_color_a = np.array([160, 60, 85])
upper_color_a = np.array([173, 200, 255])
#166 101  96, 167  58 202, 173  90 113, 174  77 198, 174 110 132, 173 116 170, 173 116 170, 172 148 107, 170 97 210

# Define lower and upper bounds for color_b in HSV color space
lower_color_b = np.array([19, 100, 85])
upper_color_b = np.array([40, 255, 255])
#23 181 199, 23 200 181, 21 184 111, 24 180 232, 30 133 188, 28 175 157, 29 156 182, 28 236 148, 28 193  87, 29 112 198

# Capture video from the webcam
cap = cv2.VideoCapture(0)


# Initialize variable if drone is detected
drone_detected = False
last_detection = None

ser = serial.Serial('COM15', baudrate=9600)

def map_value(value, in_min, in_max, out_min, out_max):
    # Map value from input range to output range
    return round((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def on_mouse_click(event, x, y, flags, param):
    global home

    if event == cv2.EVENT_LBUTTONDOWN:
        home = (x, y)
        # Reset integral terms
        integral_x = 0
        integral_y = 0

        # Reset previous errors
        prev_x = -1
        prev_y = -1
        print("New home location:", home)

cv2.namedWindow('Frame')
cv2.setMouseCallback('Frame', on_mouse_click)

ser.write(bytes([neutral_x, neutral_y]))

while True:
    ret, frame = cap.read()

    # Convert the frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for both color_a and color_b
    mask_color_a = cv2.inRange(hsv, lower_color_a, upper_color_a)
    mask_color_b = cv2.inRange(hsv, lower_color_b, upper_color_b)

    # Find contours in the color_a mask
    contours_color_a, _ = cv2.findContours(mask_color_a, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center_color_a = None

    if contours_color_a:
        largest_contour_color_a = max(contours_color_a, key=cv2.contourArea)
        # Ensure the contour has a non-zero area before calculating the center
        if cv2.contourArea(largest_contour_color_a) > 0:
            # Get the center of the largest color_a blob
            M = cv2.moments(largest_contour_color_a)
            center_color_a = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Draw a dot at the center of the largest color_a blob on the original frame
            cv2.circle(frame, center_color_a, 5, (121, 28, 227), -1)

    # Find contours in the color_b mask
    contours_color_b, _ = cv2.findContours(mask_color_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center_color_b = None

    if contours_color_b:
        # Get the largest contour based on area
        largest_contour_color_b = max(contours_color_b, key=cv2.contourArea)

        # Ensure the contour has a non-zero area before calculating the center
        if cv2.contourArea(largest_contour_color_b) > 0:
            # Get the center of the largest color_b blob
            M = cv2.moments(largest_contour_color_b)
            center_color_b = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Draw a dot at the center of the largest color_b blob on the original frame
            cv2.circle(frame, center_color_b, 5, (150, 0, 0), -1)

    if center_color_a and center_color_b:
        center_x = round((center_color_a[0] + center_color_b[0]) / 2)
        center_y = round((center_color_a[1] + center_color_b[1]) / 2)
        center = (center_x, center_y)

        # Draw a dot at the center on the original frame
        cv2.circle(frame, center, 5, (255, 255, 0), -1)

        # Calculate distance between home and center
        distance_x = center[0] - home[0]
        distance_y = home[1] - center[1]

        # Calculate the distance between the markers
        size = np.sqrt((center_color_a[0] - center_color_b[0]) ** 2 + (center_color_a[1] - center_color_b[1]) ** 2)
        angle = math.atan2(center_color_a[1] - center_color_b[1], center_color_a[0] - center_color_b[0])
        corrected_x = (distance_x * math.cos(angle) - distance_y * math.sin(angle))/size
        corrected_y = (distance_x * math.sin(angle) + distance_y * math.cos(angle))/size
        print("Corrected x: " + str(corrected_x))
        print("Corrected y: " + str(corrected_y))

        if prev_x == -1:
            prev_x = corrected_x
        if prev_y == -1:
            prev_y = corrected_y

        pid_output_x = kp * corrected_x + ki * integral_x + kd * (corrected_x - prev_x)
        pid_output_y = kp * corrected_y + ki * integral_y + kd * (corrected_y - prev_y)

        #print("PID x: " + str(pid_output_x))
        #print("PID y: " + str(pid_output_y))
        #print("P x: " + str(kp * corrected_x))
        #print("P y: " + str(kp * corrected_y))
        #print("I x: " + str(ki * integral_x))
        #print("I y: " + str(ki * integral_y))
        #print("D x: " + str(kd * (corrected_x - prev_x)))
        #print("D y: " + str(kd * (corrected_y - prev_y)))

        # Update integral terms
        integral_x += corrected_x
        integral_y += corrected_y

        # Store current errors for the next iteration
        prev_x = corrected_x
        prev_y = corrected_y

        # Print out the distance
        #print("Distance (x, y) from home:", corrected_x, "units,", corrected_y, "units")
        #print("Distance (x, y) from home:", distance_x, "pixels,", distance_y, "pixels")

        serial_x = max(min_x, min(map_value(pid_output_x, map, -map, min_x, max_x), max_x))
        serial_y = max(min_y, min(map_value(pid_output_y, map, -map, min_y, max_y), max_y))
        #print("Serial x: " + str(serial_x))
        #print("Serial y: " + str(serial_y))
        # Write the mapped value to the serial port
        ser.write(bytes([serial_x, serial_y]))


        # Drone is detected
        drone_detected = True
        last_detection = time.time()
    elif last_detection is not None and drone_detected and time.time() - last_detection >= 3.0:
        ser.write(bytes([neutral_x, neutral_y]))
        # Reset integral terms
        integral_x = 0
        integral_y = 0

        # Reset previous errors
        prev_x = -1
        prev_y = -1

        print("Drone lost")
        drone_detected = False

    # Draw a circle at home
    cv2.circle(frame, home, 5, (0, 0, 255), -1)

    # Show videos
    cv2.imshow('Frame', frame)
    cv2.imshow('color_a Mask', mask_color_a)
    cv2.imshow('color_b Mask', mask_color_b)

    if cv2.waitKey(1) == ord('q'):
        break
ser.write(bytes([neutral_x, neutral_y]))
ser.close()
cap.release()
cv2.destroyAllWindows()