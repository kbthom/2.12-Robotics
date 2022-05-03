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

###############################################

def callback(value):
    pass

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


def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
        # print('[min,max]',[min,max])
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(img, str(x) + ',' +
        #             str(y), (x,y), font,
        #             1, (255, 0, 0), 2)
        cv2.imshow('image', img)
 
    # checking for right mouse clicks    
    if event==cv2.EVENT_RBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        if len(img.shape)==3:
            b = img_depth[y, x, 0]
            g = img_depth[y, x, 1]
            r = img_depth[y, x, 2]
            # cv2.putText(img, str(b) + ',' +
            #             str(g) + ',' + str(r),
            #             (x,y), font, 1,
            #             (255, 255, 0), 2)
            print('val: '+str(b) + ',' +
                        str(g) + ',' + str(r))
        elif len(img.shape)==2:
            val = img_depth[y, x]
            cv2.putText(img, str(val) ,
                        (x,y), font, 1,
                        (255, 255, 0), 2)
            print('val: '+str(val))
        cv2.imshow('image', img)
###############################################################

cv2.namedWindow("trackbars", 0)
cv2.createTrackbar("alpha", "trackbars", 6, 200, callback)

# image_file_path="bag_images/rgb/frame000000.png"
image_file_path="kyle_depth_1.png"
# image_file_path="frame000000.png"

listener()

img=image
img_depth=depth_image


while True:

    
    
    # alph=get_trackbar_values('depth')[0]
    # # img = cv2.applyColorMap(cv2.convertScaleAbs(img,alpha=alph),cv2.COLORMAP_HOT)
    # img2=cv2.imread("kyle_color_1.png")


    cv2.imshow("image",img)
    # cv2.imshow("color image",img2)


    cv2.setMouseCallback('image', click_event)
    key=cv2.waitKey(10)
    if key==ord('q'):
        
        cv2.destroyAllWindows()
        break