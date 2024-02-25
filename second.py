import cv2
import numpy as np
import serial
import math

# Define drone's target
home = (320, 240)

max_x = 190
neutral_x = 90
min_x = 0

max_y = 190
neutral_y = 90
min_y = 0

strength = 15
real_x_max = strength
real_x_min = -strength
real_y_max = strength
real_y_min = -strength

# Define lower and upper bounds for pink in HSV color space
lower_pink = np.array([153, 80, 80])
upper_pink = np.array([173, 180, 230])

# Define lower and upper bounds for yellow in HSV color space
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])
'''if corrected_x > 0:
            serial_x = max(min_x, min(map_value(corrected_x, 0, real_x_max, neutral_x, min_x), neutral_x))
        else:
            serial_x = max(neutral_x, min(map_value(corrected_x, 0, real_x_min, neutral_x, max_x), max_x))
        if corrected_y > 0:
            serial_y = max(min_y, min(map_value(corrected_y, 0, real_y_max, neutral_y, min_y), neutral_y))
        else:
            serial_y = max(neutral_y, min(map_value(corrected_y, 0, real_y_min, neutral_y, max_y), max_y))'''

# Capture video from the webcam
cap = cv2.VideoCapture(0)

# Initialize last known information
last_distance_x, last_distance_y = 0, 0
last_size = 0
last_corrected_x, last_corrected_y = 0, 0

# Initialize variable if drone is detected
drone_detected = False

ser = serial.Serial('COM15', baudrate=9600)

def map_value(value, in_min, in_max, out_min, out_max):
    # Map value from input range to output range
    return round((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
def nonlinear_mapping(value, max_value, min_value, neutral_value):
    # Adjust the nonlinear mapping based on the distance from home
    distance_factor = 1 - (abs(value) / max_value) ** 2  # Quadratic function

    mapped_value = neutral_value + (value - neutral_value) * distance_factor
    return int(mapped_value)

ser.write(bytes([neutral_x, neutral_y]))

while True:
    ret, frame = cap.read()

    # Convert the frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for both pink and yellow
    mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Find contours in the pink mask
    contours_pink, _ = cv2.findContours(mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center_pink = None

    if contours_pink:
        largest_contour_pink = max(contours_pink, key=cv2.contourArea)
        # Ensure the contour has a non-zero area before calculating the center
        if cv2.contourArea(largest_contour_pink) > 0:
            # Get the center of the largest pink blob
            M = cv2.moments(largest_contour_pink)
            center_pink = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Draw a dot at the center of the largest pink blob on the original frame
            cv2.circle(frame, center_pink, 5, (0, 150, 0), -1)

    # Find contours in the yellow mask
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center_yellow = None

    if contours_yellow:
        # Get the largest contour based on area
        largest_contour_yellow = max(contours_yellow, key=cv2.contourArea)

        # Ensure the contour has a non-zero area before calculating the center
        if cv2.contourArea(largest_contour_yellow) > 0:
            # Get the center of the largest yellow blob
            M = cv2.moments(largest_contour_yellow)
            center_yellow = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Draw a dot at the center of the largest yellow blob on the original frame
            cv2.circle(frame, center_yellow, 5, (150, 0, 0), -1)

    if center_pink and center_yellow:
        center_x = round((center_pink[0] + center_yellow[0]) / 2)
        center_y = round((center_pink[1] + center_yellow[1]) / 2)
        center = (center_x, center_y)

        # Draw a dot at the center on the original frame
        cv2.circle(frame, center, 5, (255, 255, 0), -1)

        # Calculate distance between home and center
        distance_x = center[0] - home[0]
        distance_y = home[1] - center[1]

        # Calculate the distance between the markers
        size = np.sqrt((center_pink[0] - center_yellow[0]) ** 2 + (center_pink[1] - center_yellow[1]) ** 2)
        angle = math.atan2(center_pink[1] - center_yellow[1], center_pink[0] - center_yellow[0])
        corrected_x = (distance_x * math.cos(angle) - distance_y * math.sin(angle))/size
        corrected_y = (distance_x * math.sin(angle) + distance_y * math.cos(angle))/size
        print("Angle x: " + str(corrected_x))
        print("Angle y: " + str(corrected_y))
        #print("x: " + str(distance_x/size))
        #print("y: " + str(distance_y/size))

        # Print out the distance
        #print("Distance (x, y) from home:", corrected_x, "units,", corrected_y, "units")
        #print("Distance (x, y) from home:", distance_x, "pixels,", distance_y, "pixels")
        serial_x = nonlinear_mapping(corrected_x, real_x_max, real_x_min, neutral_x)
        serial_y = nonlinear_mapping(corrected_y, real_y_max, real_y_min, neutral_y)

        print("Serial x: " + str(serial_x))
        print("Serial y: " + str(serial_y))
        # Write the mapped value to the serial port
        ser.write(bytes([serial_x, serial_y]))

        # Update last known distances
        last_distance_x, last_distance_y = distance_x, distance_y
        last_size = size
        last_corrected_x, last_corrected_y = corrected_x, corrected_y

        # Drone is detected
        drone_detected = True
    elif drone_detected:
        ser.write(bytes([neutral_x, neutral_y]))
        # Drone lost, print last known distances
        #print("Drone lost")
        #print("Last known distance (x, y) from home:", last_corrected_x, "units,", last_corrected_y, "units")
        # print("Last known distance between markers:", last_size, "pixels")
        drone_detected = False

    # Draw a circle at home
    cv2.circle(frame, home, 5, (0, 0, 255), -1)

    # Show videos
    cv2.imshow('Frame', frame)
    #cv2.imshow('Pink Mask', mask_pink)
    #cv2.imshow('Yellow Mask', mask_yellow)

    if cv2.waitKey(1) == ord('q'):
        break
ser.write(bytes([neutral_x, neutral_y]))
ser.close()
cap.release()
cv2.destroyAllWindows()