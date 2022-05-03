#comment purpose of each argument
#consider putting brick detection into a function of its own with a separate group case



import cv2
from cv2 import COLOR_BGR2GRAY
from cv2 import MORPH_RECT
from cv2 import MORPH_CROSS
from cv2 import MORPH_ELLIPSE
import math
from cv2 import edgePreservingFilter
import numpy as np
import imutils
from scipy import signal

# import rospy
# from sensor_msgs.msg import Image, CameraInfo

color='blue'

# image=cv2.imread("kyle_color_1.png")
# depth_image=cv2.imread("kyle_depth_1.png")


def convertToOpenCVHSV(H,S,V):
    return np.array([H//2, S*2.55, V*2.55])

def create_kernel(size1,size2):
    return np.ones(size1,size2)

def callback(value):
    pass

########################################################################
#####-----------Pixel Coordinaate Transfom-----------###################
########################################################################

depth_to_meter=1 #conversion from depth units to meters

#____color camera values___#
cx_color=649.312255859375
cy_color=366.5015563964844
focal_x_color=909.676025390625
focal_y_color=909.7433471679688

#____depth camera values___#
cx_depth=422.40277099609375
cy_depth=244.71372985839844
focal_x_depth=425.2949523925781
focal_y_depth=425.2949523925781

def convert_pixel_color(u,v,z):
    """Converts Pixel location in RGB image to distance 
        from camera in the world coordinate

    Args:
        u (int): pixel x location
        v (int): pixel y location
        d (float): depth distance from camera
    
    Returns:
        (x,y,z) : tuple
            x (float): x location from camera
            y (float): y location from camera
            z (float): z location from camera
    """

    x_over_z=(cx_color-u)/focal_x_color
    y_over_z=(cy_color-v)/focal_y_color
    x=x_over_z*z
    y=y_over_z*z
    return (x,y,z)


def convert_pixel_depth(u,v,z):
    """Converts Pixel in depth image location to distance 
        from camera in the world coordinate

    Args:
        u (int): pixel x location
        v (int): pixel y location
        d (float): depth distance from camera
    Returns:
        (x,y,z) : tuple
            x (float): x location from camera
            y (float): y location from camera
            z (float): z location from camera
    """
    x_over_z=(cx_depth-u)/focal_x_depth
    y_over_z=(cy_depth-v)/focal_y_depth
    x=x_over_z*z
    y=y_over_z*z
    return (x,y,z)

########################################################################
#####-----------Pixel Coordinaate Transfom-----------###################
########################################################################

def depth(data):
    "returns alligned depth image"
    return data

def get_pixel_depth(x,y,data):
    "grabs depth of pixel (x,y) from color image"
    return data[x,y]

def get_highest_brick():
    data=depth()
    kernel=np.ones((12,12))
    out = signal.convolve2d(data, kernel, boundary='wrap', mode='same')/kernel.sum()
    sz=out.shape
    location=np.where(out==np.amin(out))[0]
    return location
            

def get_average_Hue(img,location):
    kernel=np.ones((12,12))
    out = signal.convolve2d(img[:,:,0], kernel, boundary='wrap', mode='same')/kernel.sum()
    return out[location[0],location[1]]


color_hue_map=[['red',0,7],['yellow',7,40],['green',40,92],['blue',92,128],['red',128,255]]
def highest_color(img):
    Hue=get_average_Hue(img,get_highest_brick())
    for col in color_hue_map:
        if Hue>col[1] and Hue<col[2]:
            color=col[0]
            return color

color_search_order=['blue','green','red','yellow']
color_search_order=['yellow','blue']
# color_search_order=['blue','yellow']

def get_paramaters(color):
    """
    Grab Parameters for color
    
    Returns: list of parameters in order of--

    H_min, S_min, V_min, H_max, S_max, V_max,
    erode_size, iter_erode, dilate_size, iter_dilate,shape,
    close_size1, close_size2, shape_num,
    low_area, up_area, padding,
    lower_threshold, upper_threshold, aper,
    arg1, arg2, min_distance, dp_100,min_rad, max_rad,




    """
    yellow=[22,105,192,41,255,255,
            6,1,10,1,MORPH_ELLIPSE,
            30,48,1,
            8372,23993,2,
            201,127,3,
            124,28,10,100,16,50]
    blue=[102,129,101,129,255,255,
            6,1,10,1,MORPH_ELLIPSE,
            30,48,1,
            8372,29070,2,
            201,127,3,
            124,28,10,100,16,50]

    color_vals={'blue':blue,'green':[],'yellow':yellow,'red':[]}
    return color_vals[color]

# def depth_image_func(msg):
#     global depth_image
#     try:
#         cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
#         depth_image=cv_image
        
#     except CvBridgeError as e:
#         print(e)


# def color_image_func(msg):
#     global image
#     try:
#         cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
#         image=cv_image

#     except CvBridgeError as e:
#         print(e)
#     return cv_image

# def listener():
#     #rospy.Subscriber("node path",file_type (data i.e. input to function),function_to_run)

#     rospy.Subscriber("/camera/aligned_depth_to_color/image_rect_raw",Image,depth_image_func)
#     rospy.Subscriber('/camera/rgb/image_rect_color',Image,color_image_func)


def find_brick_center():
    # global image
    # global depth_image
    image=cv2.imread("kyle_color_1.png")
    depth_image=cv2.imread("kyle_depth_1.png")
    # image=image[:,572:]
    # depth_image=depth_image[:,572:]
    
    bricks=[]
    brick_centers=[]
    brick_angles=[]
    brick_colors=[]
    brick_images=[]
    brick_local_centers=[]
    brick_names=[]
    brick_offset=[]

    for color in color_search_order:
        #find bricks (organized by depth)

        paramaters=get_paramaters(color)

#___________________HSV Converstion and HSV removal Mask__________-#        
        H_min, S_min, V_min, H_max, S_max, V_max=paramaters[0:6]
        pre_mask=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(pre_mask, (H_min, S_min, V_min), (H_max, S_max, V_max))

#_________________Erode and Dilate after HSV Removal______________#
        erode_size, iter_erode, dilate_size, iter_dilate,shape=paramaters[6:11]

        erod_kernel=cv2.getStructuringElement(shape,(erode_size,erode_size))
        mask_erode =cv2.erode(mask, erod_kernel, iterations=iter_erode)

        dilate_kernel=cv2.getStructuringElement(shape,(dilate_size,dilate_size))
        mask_dilate= cv2.dilate(mask_erode, dilate_kernel, iterations=iter_dilate)
    
#_____________________Close Holes_____________####
        close_size1,close_size2,shape_num=paramaters[11:14]

        kernel_close1=cv2.getStructuringElement(shape,(close_size1,close_size1))
        kernel_close2=cv2.getStructuringElement(shape,(close_size2,close_size2))

        mask_close=cv2.dilate(mask_dilate,kernel_close1,iterations=1)
        mask_close=cv2.erode(mask_close,kernel_close2,iterations=1)

#_____________grabs only objects from eroded dilated binary mask from original image____________#
        disp_image=cv2.bitwise_and(image,image, mask=mask_dilate)
        disp_gray=cv2.cvtColor(disp_image, COLOR_BGR2GRAY)

    #_________Locate Brick Rectangles_______________________###

        # brick_local_centers=[] #center in local brick frame
        # brick_images=[]
        # brick_names=[]
        # brick_offset=[]

        grouped_brick_offset=[]
        grouped_brick_local_centers=[] #center in local brick frame
        grouped_brick_images=[]
        grouped_brick_names=[]

        final_bricks=[] #list containing all bricks x,y center location and angle in radians

        grab_image=image.copy()
        low_area, up_area, padding = paramaters[14:17]

        ##__Grabs contours of HSV removal mask after noise was removed##
        contours=cv2.findContours(mask_dilate.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        # contours=imutils.grab_contours(contours)
        contours=contours[0]

        for contr in contours:
            M=cv2.moments(contr)
            if M['m00']!=0:
                cx=int(M['m10']/M['m00'])
                cy=int(M['m01']/M['m00'])
            area=cv2.contourArea(contr)
            rect = cv2.minAreaRect(contr)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            if area>low_area and area<up_area: #If area of contour found is in correct regime
                M=cv2.moments(contr)
                cx=int(M['m10']/M['m00'])
                cy=int(M['m01']/M['m00'])
                #############draw line orientation
                angle=180-rect[2]
                orient=angle-90
                if rect[1][0]>rect[1][1]:
                    orient=angle
                theta=orient*np.pi/180

                brick_angles.append(theta)

                #### add contour to bricks list
                bricks.append(contr)

                x,y,w,h = cv2.boundingRect(contr)
                brick_local_centers.append((cx-x+padding,cy-y+padding))
                brick_image_current=grab_image[y-padding:y+h+padding,x-padding:x+w+padding]
                brick_offset.append((x-padding,y-padding))
                brick_images.append(brick_image_current)
                brick_names.append("brick_"+str((cx,cy)))
                brick_colors.append(color)

            elif area>up_area:  
                #looks at larger contour areas incase bricks were grouped into a larger group
                M=cv2.moments(contr)
                cx=int(M['m10']/M['m00'])
                cy=int(M['m01']/M['m00'])

                ####___--if cx and cy in certatin regime add them
                x,y,w,h = cv2.boundingRect(contr)
                grouped_brick_local_centers.append((cx-x+padding,cy-y+padding))
                grouped_brick_image_current=disp_gray.copy()[y-padding:y+h+padding,x-padding:x+w+padding]
                grouped_brick_offset.append((x-padding,y-padding))
                grouped_brick_images.append(grouped_brick_image_current)
                grouped_brick_names.append("brick_"+str((cx,cy)))

        lower_threshold, upper_threshold,aper = paramaters[17:20]

        for i in range(len(grouped_brick_images)):
        #Going through these potential groupings of bricks for more intensive check
            x_offset,y_offset=grouped_brick_offset[i]
            cannyblob = cv2.Canny(cv2.equalizeHist(grouped_brick_images[i]),lower_threshold,upper_threshold,apertureSize = aper)

            contours=cv2.findContours(cannyblob.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            contours=contours[0]
            # contours=imutils.grab_contours(contours)

            for contr in contours:
                M=cv2.moments(contr)
                if M['m00']!=0:
                    cx=int(M['m10']/M['m00'])
                    cy=int(M['m01']/M['m00'])
                area=cv2.contourArea(contr)
                rect = cv2.minAreaRect(contr)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                if area>low_area and area<up_area:
                    M=cv2.moments(contr)
                    cx=int(M['m10']/M['m00'])
                    cy=int(M['m01']/M['m00'])
                    
                    angle=180-rect[2]
                    orient=angle-90
                    if rect[1][0]>rect[1][1]:
                        orient=angle

                    theta=orient*np.pi/180
                    brick_angles.append(theta)
                    ####___--if cx and cy in certatin regime add them
                    bricks.append(contr)
                    x,y,w,h = cv2.boundingRect(contr)
                    brick_local_centers.append((cx-x+padding+x_offset,cy-y+padding+y_offset))
                    
                    brick_image_current=grab_image[y-padding+y_offset:y+h+padding+y_offset,x-padding+x_offset:x+w+padding+x_offset]
                    brick_offset.append((x-padding+x_offset,y-padding+y_offset))
                    brick_images.append(brick_image_current)
                    brick_colors.append(color)
                    brick_names.append("brick_"+str((cx+x_offset,cy+y_offset)))

#______________sort bricks by decreasing depth____________##
    
    depth_brick_indicies=[]
    depth_avg=[]

    for i in range(len(bricks)):
        empty_img=np.zeros_like(image[:,:,0])
        cv2.drawContours(empty_img,bricks,i,color=255,thickness=-1)
        avg=np.average(depth_image[np.where(empty_img==255)][:,:])
        depth_avg.append(avg)
        depth_brick_indicies.append(i)

    zipped_list=zip(depth_avg,depth_brick_indicies)
    sorted_zipped_list=sorted(zipped_list)

    sorted_bricks_indices=[element for _, element in sorted_zipped_list]

    sorted_brick_images=[]
    sorted_depth_averages=[]
    sorted_brick_angles=[]
    for index in sorted_bricks_indices:
        # brick_center_x=brick_centers[index][0]
        # brick_center_y=brick_centers[index][1]
        brick_angle=brick_angles[index]
        brick_image=brick_images[index]
        sorted_brick_images.append(brick_image)
        # final_bricks.append((brick_center_x,brick_center_y,brick_angle,brick_image))

#____________________locate center hole in bricks with hough circle detection_____________#
        
    arg1,arg2,min_distance,dp_100,min_rad,max_rad=paramaters[20:26]

    for brick_num in range(len(sorted_brick_images)):
        brk_img=sorted_brick_images[brick_num]

        cv2.imshow("brick "+str(brick_num),brk_img)
        circle_image=cv2.cvtColor(brk_img.copy(),cv2.COLOR_BGR2GRAY)
        circle_image=cv2.equalizeHist(circle_image)

        dp=dp_100/100

        circles = cv2.HoughCircles(circle_image,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2,minRadius=min_rad,maxRadius=max_rad)

        if circles is not None:

            circles = (np.around(circles))
            min_dist=float('inf')

            for i in circles[0,:]:
                brick_local_centers[brick_num][0]
                brick_local_centers[brick_num][1]
                dist=(int(i[0])-brick_local_centers[brick_num][0])**2+(int(i[1])-brick_local_centers[brick_num][1])**2

                if dist<min_dist:
                    min_dist=dist
                    closest_circle=i

            brick_x_offset,brick_y_offset=brick_offset[brick_num]
            brick_centers.append((int(closest_circle[0])+brick_x_offset,int(closest_circle[1])+brick_y_offset))

    cv2.waitKey(0)
    

    return final_bricks



destination=find_brick_center()
# cv2.imshow('highest brick', destination[3])


# def locate_center_of_bricks(dilated_mask,bricks,grouping):
#     """Finds the center location of the bricks

#     Args:
#         mask to be used (image): 
#         bricks (list): list of bricks
#         groupings (bool): whether looking through groups

#     Returns:
#         bricks (list): list of all bricks found sorted by depth
#                         Bricks are tuples of (x,y,z,theta)
#     """
#     global paramaters
#     global grab_image
#     low_area, up_area, padding = paramaters[14:17]

#     if not grouping:
#      ##__Grabs contours of HSV removal mask after noise was removed##
#         contours=cv2.findContours(mask_dilate.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
#         contours=imutils.grab_contours(contours)

#         for contr in contours:
#             M=cv2.moments(contr)
#             if M['m00']!=0:
#                 cx=int(M['m10']/M['m00'])
#                 cy=int(M['m01']/M['m00'])
#             area=cv2.contourArea(contr)
#             rect = cv2.minAreaRect(contr)
#             box = cv2.boxPoints(rect)
#             box = np.int0(box)

#             if area>low_area and area<up_area: #If area of contour found is in correct regime
#                 M=cv2.moments(contr)
#                 cx=int(M['m10']/M['m00'])
#                 cy=int(M['m01']/M['m00'])
#                 #############draw line orientation
#                 angle=180-rect[2]
#                 orient=angle-90
#                 if rect[1][0]>rect[1][1]:
#                     orient=angle
#                 theta=orient*np.pi/180

#                 #### add contour to bricks list
#                 bricks.append(contr)

#                 x,y,w,h = cv2.boundingRect(contr)
#                 brick_local_centers.append((cx-x+padding,cy-y+padding))
#                 brick_image_current=grab_image[y-padding:y+h+padding,x-padding:x+w+padding]
#                 brick_offset.append((x-padding,y-padding))
#                 brick_images.append(brick_image_current)
#                 brick_names.append("brick_"+str((cx,cy)))   


#     return bricks
