import cv2
import numpy as np
from scipy import signal


###############################################
min=10000
max=0
def callback(value):
    pass

def get_trackbar_values(operation):
    values=[]
    for i in track_vals[operation]:
        v=cv2.getTrackbarPos(i,"trackbars")
        values.append(v)
    return values
track_vals={"depth":["alpha"]}
def click_event(event, x, y, flags, params):
    global max
    global min
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
        val=get_average_Hue(img2,[x,y])
        if val>max:
            max=val
        if val<min:
            min=val
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
            b = img[y, x, 0]
            g = img[y, x, 1]
            r = img[y, x, 2]
            # cv2.putText(img, str(b) + ',' +
            #             str(g) + ',' + str(r),
            #             (x,y), font, 1,
            #             (255, 255, 0), 2)
            print('val: '+str(b) + ',' +
                        str(g) + ',' + str(r))
        elif len(img.shape)==2:
            val = img[y, x]
            cv2.putText(img, str(val) ,
                        (x,y), font, 1,
                        (255, 255, 0), 2)
            print('val: '+str(val))
        cv2.imshow('image', img)
###############################################################3
def get_average_Hue(img,location):
    kernel=np.ones((3,3))
    out = signal.convolve2d(img[:,:,0], kernel, boundary='wrap', mode='same')/kernel.sum()
    return out[location[1],location[0]]

cv2.namedWindow("trackbars", 0)
cv2.createTrackbar("alpha", "trackbars", 6, 200, callback)

# image_file_path="bag_images/rgb/frame000000.png"
image_file_path="kyle_depth_1.png"
# image_file_path="frame000000.png"
img=cv2.imread(image_file_path)
img=img[:,257:,:]
img2=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

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