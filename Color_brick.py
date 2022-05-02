import numpy as np
from scipy import signal
import cv2

image_file_path="kyle_depth_1.png"
# image_file_path="frame000000.png"
img=cv2.imread(image_file_path)
img2=cv2.imread('kyle_color_1.png')
img2=img2[100:,257:,:]

# Define kernel for convolution                                         
kernel = np.array([[1,1,1],
                   [1,1,1],
                   [1,1,1]]) 

def get_highest_brick():
    data=img[100:,257:,0]
    kernel=np.ones((20,20))
    out = signal.convolve2d(data, kernel, boundary='wrap', mode='same')/kernel.sum()
    result=np.where(out==np.amin(out))
    location=list(zip(result[0],result[1]))
    return location[0]

def get_average_Hue(img,location):
    kernel=np.ones((3,3))
    img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    out = signal.convolve2d(img[:,:,0], kernel, boundary='wrap', mode='same')/kernel.sum()
    return out[location[1],location[0]]



# Perform 2D convolution with input data and kernel 
# img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
location=get_highest_brick()
print(location)
cv2.circle(img2,location,2,(0,0,255),3)
cv2.imshow("image",img2)
cv2.waitKey(0)

# print(get_average_Hue(img2,location))
while True:
    break
    
    location=[1143,177]
    print(get_average_Hue(img,location))
    # out = signal.convolve2d(img[:,:,0], kernel, boundary='wrap', mode='same')/kernel.sum()
    cv2.imshow("image",img)
    # cv2.imshow("avg",out)
    key=cv2.waitKey(10)
    if key==ord('q'):
        cv2.destroyAllWindows()
        break