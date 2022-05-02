import cv2

depth_image = cv2.imread("frame000000.png")

depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=20),cv2.COLORMAP_HOT)
cv2.imshow(cv2.namedWindow('RealSense',cv2.WINDOW_AUTOSIZE), depth_colormap)
cv2.waitKey(0)