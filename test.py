import cv2
import numpy as np
y = np.array([[[253,160,220,]]], dtype=np.uint8)
x = cv2.cvtColor(y, cv2.COLOR_RGB2HSV)
print(x[0][0])