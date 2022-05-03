import cv2
from time import sleep
from cv2 import COLOR_BGR2GRAY
from cv2 import MORPH_RECT
from cv2 import MORPH_CROSS
from cv2 import MORPH_ELLIPSE
import math
from cv2 import edgePreservingFilter
import numpy as np
import imutils
# from scipy import signal

import rospy
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from time import sleep


from cv_bridge import CvBridge, CvBridgeError
import message_filters

import apriltag

cv_bridge=CvBridge()

image=1
depth_image=1

def depth_image_func(msg):
     global depth_image
     try:
        # im = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
        cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
        depth_array = np.array(cv_image,dtype=np.float32)
        # depth_image=depth_array.astype('uint8')
        # depth_image=im
        depth_image=depth_array

     except CvBridgeError as e:
        print(e)
     return depth_image



def listener():
    rospy.init_node('listener', anonymous=True)
    #rospy.Subscriber("node path",file_type (data i.e. input to function),function_to_run)
    
    rospy.Subscriber('/cam_1/color/image_raw',Image,color_image_func)
    rospy.Subscriber("/cam_1/aligned_depth_to_color/image_raw",Image,depth_image_func)
    print('listened')

def color_image_func(msg_color):
    global image
    try:

        # im = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
        cv_image = cv_bridge.imgmsg_to_cv2(msg_color, "bgr8")
        image=cv_image
        # image=im

        
    except CvBridgeError as e:
        print(e)
    return image

color='yellow_'
number=1
listener()
sleep(1)

cv2.imshow('col image',image)
cv2.imshow('depth image',depth_image)

if number-1 !=1:
    file='depth_image_'+color+str(number-1)+'.npy'
    depth_loaded=np.load(file,allow_pickle=True,fix_imports=True)
    cv2.imshow('previous depth image',depth_loaded)
    print(depth_loaded[0:2,0:2])

key=cv2.waitKey(0)

if key==ord('s'):
    cv2.imwrite('color_image_'+color+str(number)+'.png',image)
    np.save('depth_image_'+color+str(number),depth_image, allow_pickle=True,fix_imports=True)