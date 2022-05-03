import cv2
from cv2 import COLOR_BGR2GRAY
import numpy as np
import imutils

def convertToOpenCVHSV(H,S,V):
    return np.array([H//2, S*2.55, V*2.55])

def callback(value):
    pass

def create_HSV_trackbars():
    for i in ["MIN","MAX"]:
        v=0 if i== "MIN" else 255
        for j in "HSV":
            cv2.createTrackbar("%s_%s" % (j,i), "trackbars", v, 255, callback)


imagelink="bag_images/rgb/frame000000.png"
cv_image=cv2.imread(imagelink)
hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

cv2.imshow("HSV Image")
cv2.imshow("Original Image",cv_image)

cv2.waitKey(0)
