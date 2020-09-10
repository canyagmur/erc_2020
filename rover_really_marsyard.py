#!/usr/bin/env python

from datetime import datetime
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import time
i=0
j=0
bridge = CvBridge()

def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show: 
        cv2.imshow("RGB Image",rgb_image)
    return rgb_image

def convert_rgb_to_gray(rgb_image,show):
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    if show: 
        cv2.imshow("Gray Image",gray_image)
    return gray_image

def convert_gray_to_binary(gray_image, adaptive, show):
    if adaptive: 
        binary_image = cv2.adaptiveThreshold(gray_image, 
                            255, 
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                            cv2.THRESH_BINARY_INV, 115, 2)
    else:
        _,binary_image = cv2.threshold(gray_image,110,255,cv2.THRESH_BINARY_INV)
    if show:
        cv2.imshow("Binary Image", binary_image)
    return binary_image    

def draw_contours(black_image,image, contours,color_of_contour,area_min,area_max,text,thick):
    index = -1 #means all contours
    color = color_of_contour #color of the contour line
    font = cv2.FONT_HERSHEY_SIMPLEX
    for c in contours:
        area=cv2.contourArea(c)
        perimeter=cv2.arcLength(c,True)
        cx, cy = get_contour_center(c)
        #((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>area_min and area<area_max):
            cv2.drawContours(image, [c], -1, color, thickness=thick)
            cv2.drawContours(black_image, [c], -1, (150,250,150), thickness=thick)
            if(area>area_min and area<area_max):
                #x,y,w,h = cv2.boundingRect(c)
                #cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
                #cv2.putText(image,text,(cx-100,cy), font, 1,(0,0,0),2)
                cv2.putText(black_image,text,(cx-100,cy), font, 1,(255,255,255),2)
            
            #cv2.circle(image, (cx,cy),(int)(2),(0,0,255),thickness=2)
            #cv2.circle(image, (cx,cy),(int)(2),(0,0,255),thickness=2)
            cv2.circle(black_image, (cx,cy),(int)(2),(0,0,255),thickness=1)
            #print ("Area: {}, Perimeter: {}".format(area, perimeter))
def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy


def getContours(binary_image):     
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)[-2:]
    return contours

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",hsv_image)
    #cv2.imshow("RGB Image",rgb_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask

def detectIsland(image_frame,black_image):
    IslandLower =(16, 22, 224)
    IslandUpper = (25, 44, 252)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, IslandLower, IslandUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(0,255,0),1000,500000,"WHITE SOIL",thick=2)

def detectGray(image_frame,black_image):
    grayLower =(13, 35, 181)
    grayUpper = (15, 43, 201)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, grayLower, grayUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(0, 0,255),100,500000,"GRAY SOIL",thick=2)
def detectYellow(image_frame,black_image):
    grayLower =(16, 52, 188)
    grayUpper = (255, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, grayLower, grayUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(255, 255,0),1000,500000,"",thick=1)
def detectRocks(image_frame,black_image):
    rockLower =(105, 13, 35)
    rockUpper = (124, 71, 94)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, rockLower, rockUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(255, 0,0),500,3000,"rocks",thick=1)
def detectRed(image_frame,black_image):
    rockLower =(0, 59, 0)
    rockUpper = (11, 125, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, rockLower, rockUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(0, 255,255),600,500000,"TERRAIN",thick=2)
def detectOldRocks(image_frame,black_image):
    grayLower =(40, 10, 50)
    grayUpper = (81, 255, 139)
    #yellowLower =(30, 10, 52)
    #yellowUpper = (95, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, grayLower, grayUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(255, 0, 255),0,"",thick=1)
def detectRight(image_frame,black_image):
    grayLower =(0, 16, 110)
    grayUpper = (32, 51, 220)
    #yellowLower =(30, 10, 52)
    #yellowUpper = (95, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, grayLower, grayUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(0, 0, 0),1000,500000,"RIGHT",thick=3)
def detectWhite(image_frame,black_image):
    grayLower =(20, 0, 255)
    grayUpper = (255, 55, 255)
    #yellowLower =(30, 10, 52)
    #yellowUpper = (95, 255, 255)
    rgb_image = image_frame
    #cv2.imshow("original",rgb_image)
    
    crop_rgb_image=image_frame[200:rgb_image.shape[0],0:rgb_image.shape[1]]
    
    #cv2.imshow("cropped",crop_rgb_image)

    crop_black_image=image_frame[200:black_image.shape[0],0:black_image.shape[1]]
    
    binary_image_mask = filter_color(crop_rgb_image, grayLower, grayUpper)
    contours = getContours(binary_image_mask)
    draw_contours(crop_black_image,crop_rgb_image, contours,(0, 0, 0),100,500000,"WHITE",thick=2)


def image_callback(ros_image):
  #print 'got an image'
  global i
  global j
  global bridge
  j+=1
  i+=1                    #WARNNINGGGGGG FOR TAKING PHOTOS
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv---------
  #cv2.imshow("RGB Image",cv_image)
  frame=cv_image
  black_image = np.zeros([frame.shape[0],frame.shape[1],3],'uint8')
  
  if cv2.waitKey(1) == ord('r') or i>=70:
   cv2.imwrite("/mars/savedImage{}.jpg".format(datetime.now()),frame)
   print("raw image is saved!")
   i=0
   
  
  
  detectIsland(frame,black_image) #I will turn back to you xd

  detectGray(frame,black_image) #I will turn back to you xd

  detectRocks(frame,black_image) #works good!

  detectRed(frame,black_image) #works good!

  detectWhite(frame,black_image)

  #detectOldRocks(frame,black_image) #this is problematic! (trees)

  #detectRight(frame,black_image) #this is problematic! detects everything xd

  detectYellow(frame,black_image)  #I will turn back to you xd

  cv2.imshow("RGB Image Contours",frame)
  cv2.imshow("Black Image Contours",black_image)
  if(cv2.waitKey(1) == ord(' ') or j>=70):
    cv2.imwrite("/mars/savedImage{}.jpg".format(datetime.now()),frame)
    cv2.imwrite("/mars/savedImage{}.jpg".format(datetime.now()),black_image)
    print("proccesed images are saved!")
    j=0

  
def main(args):
  print("press 'r' for taking raw iamges")
  print("press SPACE button for taking processed iamges")
  rospy.init_node('marsyard_image_proccessing', anonymous=True)
  image_sub = rospy.Subscriber("zed2/left/image_rect_color",Image, image_callback) #zed2/left_raw/image_raw_color 26   /zed2/left/image_rect_color 31   /zed2/right/image_rect_color 34
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
