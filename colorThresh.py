import numpy as np
import cv2  # OpenCV module

imagelink="bag_images/rgb/frame000000.png"
cv_image=cv2.imread(imagelink)
cv2.imshow("Original Image",cv_image)
cv2.waitKey(0)