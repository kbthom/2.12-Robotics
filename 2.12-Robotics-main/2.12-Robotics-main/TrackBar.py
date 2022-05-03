import cv2
from cv2 import COLOR_BGR2GRAY
from cv2 import MORPH_RECT
from cv2 import MORPH_CROSS
from cv2 import MORPH_ELLIPSE
import math
from cv2 import edgePreservingFilter
import numpy as np
import imutils

color='blue'

def convertToOpenCVHSV(H,S,V):
    return np.array([H//2, S*2.55, V*2.55])

def create_kernel(size1,size2):
    return np.ones(size1,size2)

def callback(value):
    pass

########################################################################
#####---------------TrackBars-------------##############################
########################################################################
def create_HSV_trackbars(color):
    
    cv2.createTrackbar("H_MIN", "trackbars", color[0], 255, callback)
    cv2.createTrackbar("S_MIN", "trackbars", color[1], 255, callback)
    cv2.createTrackbar("V_MIN", "trackbars", color[2], 255, callback)
    cv2.createTrackbar("H_MAX", "trackbars", color[3], 255, callback)
    cv2.createTrackbar("S_MAX", "trackbars", color[4], 255, callback)
    cv2.createTrackbar("V_MAX", "trackbars", color[5], 255, callback)

def create_Erode_Dilate_trackbars():
        cv2.createTrackbar("Erode_Size", "trackbars", 6, 100, callback)
        cv2.createTrackbar("Erode_Iterations","trackbars", 1, 30, callback)
        cv2.createTrackbar("Dilate_Size", "trackbars", 10, 100, callback)
        cv2.createTrackbar("Dilate_Iterations","trackbars", 1, 30, callback)
    
def create_Canny_trackbars():
    cv2.createTrackbar("lower_threshold", "trackbars", 201, 1000, callback)
    cv2.createTrackbar("upper_threshold", "trackbars", 127, 1000, callback)
    cv2.createTrackbar("aperature", "trackbars", 3, 25, callback)

def create_area_trackbars():
    cv2.createTrackbar("lower_area", "trackbars", 8372, 30000, callback)
    cv2.createTrackbar("upper_area", "trackbars", 29070, 50000, callback)
    cv2.createTrackbar("pad", "trackbars", 2, 100, callback)

def create_circle_trackbars():
    cv2.createTrackbar("arg1 (higher canny thresh)", "trackbars", 124, 800, callback)
    cv2.createTrackbar("arg2 (accumulator thresh)", "trackbars", 28, 200, callback)
    cv2.createTrackbar("min_distance", "trackbars", 10, 100, callback)
    cv2.createTrackbar("dp*100", "trackbars", 100, 300, callback)
    cv2.createTrackbar("Min Radius", "trackbars", 16, 300, callback)
    cv2.createTrackbar("Max Radius", "trackbars", 50, 300, callback)

def create_line_trackbars():
    cv2.createTrackbar("min_intersections", "trackbars", 0, 1000, callback)

def create_close_trackbars():
    cv2.createTrackbar("close_size(dilate)", "trackbars", 30, 100, callback)
    cv2.createTrackbar("close_size(erode)", "trackbars", 48, 100, callback)
    cv2.createTrackbar("shape", "trackbars", 1, 2, callback)

def get_trackbar_values(operation):
    values=[]
    for i in track_vals[operation]:
        v=cv2.getTrackbarPos(i,"trackbars")
        values.append(v)
    return values

HSV_track_names= ["H_MIN","S_MIN","V_MIN","H_MAX","S_MAX","V_MAX"]
Erode_Dilate_track_names=["Erode_Size", "Erode_Iterations","Dilate_Size", "Dilate_Iterations"]
Canny_track_names=["lower_threshold","upper_threshold","aperature"]
Area_track_names=["lower_area","upper_area","pad"]
line_track_names=["min_intersections"]
hough_circ_track_names=["arg1 (higher canny thresh)","arg2 (accumulator thresh)","min_distance","dp*100","Min Radius", "Max Radius"]
close_track_names=["close_size(dilate)","close_size(erode)","shape"]
track_vals={"HSV":HSV_track_names, "E/D":Erode_Dilate_track_names, "Canny":Canny_track_names,'Area':Area_track_names, "Hough Circle":hough_circ_track_names,"lines":line_track_names,'close':close_track_names}

    
colors={'b':[102,129,101,129,255,255],'r':[],'g':[],'y':[22,86,70,41,255,255]}

def create_trackbars():
    create_HSV_trackbars(colors['y'])
    create_Erode_Dilate_trackbars()
    create_Canny_trackbars()
    create_area_trackbars()
    create_circle_trackbars()
    create_line_trackbars()
    create_close_trackbars()


cv2.namedWindow("orig", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("mask", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("erode", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("object detect", cv2.WINDOW_AUTOSIZE)

cv2.namedWindow("trackbars", 0)

create_trackbars()
canny=False
hough=False
trace=False
track_add=False
show_bricks=False
hough_lines=False

while True:
    imagelink="bag_images/rgb/frame000000.png"
    image=cv2.imread(imagelink)

    

    # image=cv2.flip(image, 2) #flip across yaxis
    # image=cv2.flip(image, 0)  #flip across x-axis
    x_halfway=585
    image=image[:,x_halfway:,:]
    rect_bound=image.copy()
    height,width=image.shape[0:2]
    hough_line_img=np.zeros((height,width,1), np.uint8)
    
    # blur=cv2.GaussianBlur(image, [11,11],0)

#_________-_________HSV Converstion and HSV removal Mask__________-#
    pre_mask=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    H_min, S_min, V_min, H_max, S_max, V_max= get_trackbar_values("HSV")
    mask=cv2.inRange(pre_mask, (H_min, S_min, V_min), (H_max, S_max, V_max))
    
    # blurred=cv2.GaussianBlur(mask, [11,11],0)

#_________________Erode and Dilate after HSV Removal______________#
    erode_size, iter_erode, dilate_size, iter_dilate = get_trackbar_values("E/D")
    erode_size=max(1,erode_size)
    dilate_size=max(1,dilate_size)


    # shape=MORPH_RECT #change kernel shape MORPH_    RECT or CROSS or ELLIPSE
    # shape=MORPH_CROSS
    shapes=[MORPH_RECT,MORPH_ELLIPSE,MORPH_CROSS]
    shape=MORPH_ELLIPSE

    erod_kernel=cv2.getStructuringElement(shape,(erode_size,erode_size))
    mask_erode =cv2.erode(mask, erod_kernel, iterations=iter_erode)

    dilate_kernel=cv2.getStructuringElement(shape,(dilate_size,dilate_size))
    mask_dilate= cv2.dilate(mask_erode, dilate_kernel, iterations=iter_dilate)

#_____________________Close Holes_____________####
    close_size1,close_size2,shape_num = get_trackbar_values("close")
    shape=shapes[shape_num]
    close_size1=max(1,close_size1)
    close_size2=max(1,close_size2)

    kernel_close1=cv2.getStructuringElement(shape,(close_size1,close_size1))
    kernel_close2=cv2.getStructuringElement(shape,(close_size2,close_size2))

    
    mask_close=cv2.dilate(mask_dilate,kernel_close1,iterations=1)
    mask_close=cv2.erode(mask_close,kernel_close2,iterations=1)
    cv2.imshow("closing",mask_close)


#_____________grabs only objects from eroded dilated binary mask from original image____________#
    disp_image=cv2.bitwise_and(image,image, mask=mask_dilate)
    disp_gray=cv2.cvtColor(disp_image, COLOR_BGR2GRAY)

#________________Centroid and Moment________________#
    if trace:
        contours=cv2.findContours(mask_dilate.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contours=imutils.grab_contours(contours)
        cv2.drawContours(rect_bound, contours, -1, (255,0,0), 3)
        bricks=[]
        brick_centers=[]
        brick_local_centers=[] #center in local brick frame
        brick_images=[]
        brick_names=[]
        grab_image=image.copy()
        for contr in contours:
            M=cv2.moments(contr)
            if M['m00']!=0:
                cx=int(M['m10']/M['m00'])
                cy=int(M['m01']/M['m00'])
            area=cv2.contourArea(contr)
            perimeter= cv2.arcLength(contr,True)
            rect = cv2.minAreaRect(contr)
            box = cv2.boxPoints(rect)
            box = np.int0(box)



            low_area, up_area, padding = get_trackbar_values("Area")
            if area>low_area and area<up_area:
                cv2.drawContours(rect_bound,[box],0,(0,0,255),2)
                M=cv2.moments(contr)
                cx=int(M['m10']/M['m00'])
                cy=int(M['m01']/M['m00'])
                #############draw line orientation
                angle=180-rect[2]
                orient=angle-90
                if rect[1][0]>rect[1][1]:
                    orient=angle
                theta=orient*np.pi/180
                length=50
                p1x=round(cx+length*math.cos(theta))
                p1y=round(cy-length*math.sin(theta))
                cv2.line(rect_bound,(cx,cy),(p1x,p1y),[255,192,203],2)

                cv2.circle(rect_bound,(cx,cy),2,(0,0,255),3)

                ####___--if cx and cy in certatin regime add them
                bricks.append(contr)
                brick_centers.append((cx,cy))
                x,y,w,h = cv2.boundingRect(contr)
                brick_local_centers.append((cx-x+padding,cy-y+padding))
                cv2.rectangle(rect_bound,(x-padding,y-padding),(x+w+padding,y+h+padding),(0,255,0),2)
                x-w-padding
                brick_image_current=grab_image[y-padding:y+h+padding,x-padding:x+w+padding]
                brick_images.append(brick_image_current)
                brick_names.append("brick_"+str((cx,cy)))
                if show_bricks:
                    for i in range(len(brick_images)):
                        cv2.imshow(brick_names[i],brick_images[i])

#_____________Hough on just bricks_______________########
        name=0
        for brk_img in brick_images:
            if hough:
                circle_image=cv2.cvtColor(brk_img.copy(),cv2.COLOR_BGR2GRAY)
                circle_image=cv2.equalizeHist(circle_image)
                arg1,arg2,min_distance,dp_100,min_rad,max_rad=get_trackbar_values("Hough Circle")
                arg1=max(1,arg1)                    # Upper threshold for Canny edge detection
                arg2=max(1,arg2)                    # Threshold for center detection.
                min_distance=max(1, min_distance)   # Minimum distance between new circle centers. 
                dp_100=max(1,dp_100)                # How accepting to degradation of circles are you willing to be
                min_rad=max(1,min_rad)              # min circle radius
                max_rad=max(1,max_rad)              # max circle radius  
                dp=dp_100/100

                circles = cv2.HoughCircles(circle_image,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2,minRadius=min_rad,maxRadius=max_rad)
                # print(circles)
                if circles is not None:
                    circles = (np.around(circles))
                    circle_distance_from_center=[]
                    min_dist=float('inf')
                    for i in circles[0,:]:
                        # draw circle
                        
                        dist=(int(i[0])-brick_local_centers[name][0])**2+(int(i[1])-brick_local_centers[name][1])**2
                        if dist<min_dist:
                            min_dist=dist
                            closest_circle=i

                    cv2.circle(brk_img,((int(closest_circle[0]),int(closest_circle[1]))),int(closest_circle[2]),(0,255,0),2)
                    cv2.circle(brk_img,((int(closest_circle[0]),int(closest_circle[1]))),2,(0,0,255),3)
                if show_bricks:
                        # eq_brk=cv2.equalizeHist(cv2.cvtColor(brk_img,cv2.COLOR_BGR2GRAY))
                        cv2.imshow(brick_names[name],cv2.cvtColor(brk_img,cv2.COLOR_BGR2GRAY))
                name+=1
#_______________Canny Edge Detection____________#
    if canny:

        lower_threshold, upper_threshold,aper = get_trackbar_values("Canny")
        aper=max(1,aper)

        cannyIm = cv2.Canny(disp_gray,lower_threshold,upper_threshold,apertureSize = aper)


#__________________Hough Lines_______________#
    if hough_lines and canny:
        min_intersections=get_trackbar_values("lines")[0]
        min_intersections=max(1,min_intersections)

        edge_image=cv2.bitwise_and(cannyIm,cannyIm, mask=cv2.bitwise_not(mask_close))
        cv2.imshow("canny_close",edge_image)
        lines = cv2.HoughLines(edge_image,1,np.pi/180,min_intersections)
        
        num_lines = 0;

        shape=None
        if lines is not None:
            shape=lines.shape

        if shape is not None:
            for j in range(shape[0]):                         # Plot lines over original feed
                for rho,theta in lines[j]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    cv2.line(hough_line_img,(x1,y1),(x2,y2),(255),2)
                    num_lines += 1
        contours=cv2.findContours(hough_line_img.copy(),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contours=imutils.grab_contours(contours)
        cv2.drawContours(hough_line_img, contours, -1, (255,0,0), 3)



    

    contr=cv2.findContours(mask_dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contr=imutils.grab_contours(contr)
    if len(contr)>0:
        track=image.copy()
        c=max(contr, key=cv2.contourArea)
        ((x,y),radius) = cv2.minEnclosingCircle(c)
        cv2.circle(track, (int(x), int(y)), int(radius), (0, 255, 255), 2)

    cv2.imshow("orig",image)
    cv2.imshow("mask",mask)
    cv2.imshow("erode",mask_erode)
    cv2.imshow("dilate after erode",mask_dilate)
    # cv2.imshow("disp",disp_image)
    cv2.imshow("rect_bound",rect_bound)
    cv2.imshow("disp",disp_gray)
    if canny:
        cv2.imshow("Canny_Image", cannyIm)
    # cv2.imshow("Hough Circle Detection", circle_image)

    if hough_lines:
        cv2.imshow("lines",hough_line_img)

    key=cv2.waitKey(1)
    if key & 0xFF is ord('q'):
        break
    elif key==ord('s'):
        cv2.imwrite("final_display3.png",disp_gray)
        cv2.imwrite("final_canny3.png",cannyIm)
    elif key==ord('c'):
        canny=True
        hough=False
        track_add=True
    elif key==ord('h'):
        canny=False
        hough=True
    elif key==ord("t"):
        trace=True
    elif key==ord("b"):
        show_bricks=True
    elif key==ord("l"):
        hough_lines=True


