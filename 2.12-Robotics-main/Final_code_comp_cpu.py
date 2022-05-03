#comment purpose of each argument
#consider putting brick detection into a function of its own with a separate group case

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


from cv_bridge import CvBridge, CvBridgeError
import message_filters

import apriltag

cv_bridge=CvBridge()

color='blue'

image=cv2.imread("kyle_color_1.png")
depth_image=cv2.imread("kyle_depth_1.png")
image=1
depth_image=1


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


def get_pixel_depth(x,y,data):
    "grabs depth of pixel (x,y) from color image"
    return data[x,y]

# def get_highest_brick():
#     data=depth()
#     kernel=np.ones((12,12))
#     out = signal.convolve2d(data, kernel, boundary='wrap', mode='same')/kernel.sum()
#     sz=out.shape
#     location=np.where(out==np.amin(out))[0]
#     return location
            

# def get_average_Hue(img,location):
#     kernel=np.ones((12,12))
#     out = signal.convolve2d(img[:,:,0], kernel, boundary='wrap', mode='same')/kernel.sum()
#     return out[location[0],location[1]]


# color_hue_map=[['red',0,7],['yellow',7,40],['green',40,92],['blue',92,128],['red',128,255]]
# def highest_color(img):
#     Hue=get_average_Hue(img,get_highest_brick())
#     for col in color_hue_map:
#         if Hue>col[1] and Hue<col[2]:
#             color=col[0]
#             return color

color_search_order=['blue','green','red','yellow']
color_search_order=['yellow','blue']

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

detector=apriltag.Detector()

def april_tag_info(id):
    """Finds the center location and angle in image frame of april tag
    Args:
        id (int): id of april tag whose information you would like
    Returns:
        info (tuple): (centerx,centery,angle) center x, y are floats
    """
    global image
    img=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    result=detector.detect(img)
    for index in range(len(result)):
        if result[index].tag_id==id:
            center_loc=result[index].center
            center_x=center_loc[0]
            center_y=center_loc[1]
            top_right_loc=result[index].corners[1]
            top_right_x=top_right_loc[0]
            top_right_y=top_right_loc[1]
            angle=math.atan2(center_y-top_right_y,top_right_x-center_x)

            return (center_x,center_y,angle)

brick_height=82.5
table_to_camera=1028
safety_net=12

levels=[[table_to_camera+15,table_to_camera-brick_height-safety_net],[table_to_camera-brick_height-safety_net,table_to_camera-2*brick_height-safety_net],[table_to_camera-2*brick_height-safety_net,table_to_camera-3*brick_height-safety_net],[table_to_camera-3*brick_height-safety_net,table_to_camera-4*brick_height-safety_net]] #ranges of depths for brick to be expected

def create_level_image(depth_array,image):
    """creates a mask of depth level
    Args:
        depth_array (array): array of depth in mm for x_y locations
        image (image): image of course
    Returns:
        tuple: (new image only in level, level at)
    """
    level=3
    while level>-1:
        depth_range=levels[level]
        level_image=np.where(depth_array>depth_range[0] and depth_array<depth_range[1],0,255)
        level_image = level_image.astype('uint8')

        if np.count_nonzero(level_image)>50000:
            return (level_image,level)
        level-=1
    print('depth failure')

 

def find_brick_height_from_average_depth(depth):
    if depth<740:
        return False
    elif depth>levels[3][0] and depth<levels[3][1]:
        return 4
    elif depth>=levels[2][0] and depth<levels[2][1]:
        return 3
    elif depth>=levels[1][0] and depth<levels[1][1]:
        return 2
    elif depth>=levels[0][0] and depth<levels[0][1]:
        return 1
    return False
    

def find_brick_center():
    global image
    global depth_image

    end_effector_tag=april_tag_info(end_effector_id)

    # image=image[:,end_effector_tag[0]:]


    print('starting main')
    # image=cv2.imread("kyle_color_1.png")
    # depth_image=cv2.imread("kyle_depth_1.png")
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

        # ##__Grabs contours of HSV removal mask after noise was removed##
        # im3,contours,hierarchy=cv2.findContours(mask_dilate.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        contours=cv2.findContours(mask_dilate.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contours=imutils.grab_contours(contours)
        # contours=contours[0]

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
            contours=imutils.grab_contours(contours)
            # contours=contours[0]

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

    
#____________________locate center hole in bricks with hough circle detection_____________#
        
    arg1,arg2,min_distance,dp_100,min_rad,max_rad=paramaters[20:26]

    for brick_num in range(len(brick_images)):
        brk_img=brick_images[brick_num]

        # cv2.imshow("brick "+str(brick_num),brk_img)
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

#______________sort bricks by decreasing depth____________##
    
    depth_brick_indicies=[]
    depth_avg=[]

    for i in range(len(bricks)):
        empty_img=np.zeros_like(image[:,:,0])
        cv2.drawContours(empty_img,bricks,i,color=255,thickness=-1)
        avg=np.average(depth_image[np.where(empty_img==255)])
        depth_avg.append(avg)
        depth_brick_indicies.append(i)

    zipped_list=zip(depth_avg,depth_brick_indicies)
    sorted_zipped_list=sorted(zipped_list)

    sorted_bricks_indices=[element for _, element in sorted_zipped_list]

    sorted_brick_images=[]
    sorted_depth_averages=depth_avg
    sorted_brick_angles=[]
    sorted_brick_centers=[]
    
    end_effector_x=end_effector_tag[0]
    end_effector_y=end_effector_tag[1]
    end_effector_angle=end_effector_tag[2]

    end_effector_x,end_effector_y,z=convert_pixel_color(end_effector_x,end_effector_y,depth_image[int(end_effector_y),int(end_effector_x)])
    for index in sorted_bricks_indices:
        brick_center_x=brick_centers[index][0]
        brick_center_y=brick_centers[index][1]
        brick_angle=brick_angles[index]
        brick_image=brick_images[index]

        brick_image=cv2.circle(brick_image,(brick_center_x,brick_center_y),2,(0,0,255),3)
        depth=sorted_depth_averages[index]
        brick_z=find_brick_height_from_average_depth(depth)
        sorted_brick_images.append(brick_image)

        brick_center_x, brick_center_y,depth=convert_pixel_color(brick_center_x,brick_center_y,depth)



        final_bricks.append((brick_center_x- end_effector_x,brick_center_y- end_effector_y,brick_z,brick_angle-end_effector_angle,brick_image))  

    return final_bricks


if __name__=='__main__':
    # rospy.init_node('tester',anonymous=True)
    # end_effector_id=10
    end_effector_id=9
    listener()
    sleep(1)
    cv2.imwrite('depth_im.png',depth_image)
    cv2.imwrite('col_im.png',image)
    cv2.imshow('im',image)
    cv2.imshow('depth',depth_image)
    final_bricks=find_brick_center()
    
    destination=False
    z=False
    brk_num=0
    
    while z==False and brk_num<len(final_bricks):
        print(brk_num)
        destination=final_bricks[brk_num][0:4]
        z=destination[2]
        brk_num+=1


    end_effector_tag=april_tag_info(end_effector_id)

    print(final_bricks)

    for brk in final_bricks:
        cv2.imshow(str(brk[2])+'  '+str(brk[0]),brk[4])

    cv2.waitKey(0)

    # if destination:
    #     print(destination)
    #     print(final_bricks[0][2])
    #     cv2.imshow('brick',final_bricks[0][4])
    #     # cv2.imshow('highest brick', destination[4])
    #     cv2.waitKey(0)


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