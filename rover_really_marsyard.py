#!/usr/bin/env python3

from datetime import datetime
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import time
import argparse
import argparser
import os.path
from pynput.keyboard import Listener  #used for save image from terminal

spacePressed = False

#THIS SCRIPT TAKES THREE ARGUMENTS : dir,subs, show
#ARGUMENTS CAN BE GIVEN AS FOLLOW : python marsyard_detection.py --kwargs dir=<valid_string_directory> subs=<valid_string_topic> show=<valid_boolean_decision>
#ORDER OF ARGUMENTS IS NOT IMPORTANT !


#If i = 1 , j must be 7 to save images concurrently.
i=0 
j=0

def keyPressed(key):
    global spacePressed
    if(str(key) == "Key.space"):
        spacePressed = True

listener = Listener(on_press=keyPressed)
listener.start()



DIRECTORY = "/mars"
sub_topic = "/zed2/left_raw/image_raw_color"
show_image = False

# creating parser pbject
parser = argparse.ArgumentParser()

# adding an arguments
parser.add_argument('--kwargs',
                    nargs='*',
                    action = argparser.keyvalue)

#parsing arguments
args = parser.parse_args()

# show the dictionary
if(bool(args.kwargs)!=0):
    if(args.kwargs.has_key("dir")):
        DIRECTORY = args.kwargs["dir"]
    if(args.kwargs.has_key("subs")):
        sub_topic = args.kwargs["subs"]
    if(args.kwargs.has_key("show")):
        show_image = bool(args.kwargs["show"])
else:
    print("No arguments passed. Default values will be used.")

if(os.path.isdir(DIRECTORY)!=1):
    print("Directory is not a valid one!")
    print("Exiting...")
    sys.exit(0)

print("Subscribed Topic : "+sub_topic)
print("Treasure Directory : "+DIRECTORY)
print("Show Images : "+ str(show_image))


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

def draw_contours(black_image,image, contours,color_of_contour,limit_area,text,thick):
    global j
    j+=1
    index = -1 #means all contours
    color = color_of_contour #color of the contour line
    font = cv2.FONT_HERSHEY_SIMPLEX
    for c in contours:
        area=cv2.contourArea(c)
        perimeter=cv2.arcLength(c,True)
        cx, cy = get_contour_center(c)
        #((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>limit_area):
            if(text!="RIGHT"):
                cv2.drawContours(image, [c], -1, color, thickness=thick)
                cv2.drawContours(black_image, [c], -1, (150,250,150), thickness=thick)
                if(area>30000):
                    #x,y,w,h = cv2.boundingRect(c)
                    #cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
                    #cv2.putText(image,text,(cx-100,cy), font, 1,(0,0,0),2)
                    cv2.putText(black_image,text,(cx-100,cy), font, 1,(255,255,255),2)
            elif area<30000 and area>limit_area:
                cv2.drawContours(image, [c], -1, color, thickness=thick)
                cv2.drawContours(black_image, [c], -1, (150,250,150), thickness=thick)
                cv2.putText(black_image,text,(cx-100,cy), font, 1,(255,255,255),2)


            
            

                

            #cv2.circle(image, (cx,cy),(int)(2),(0,0,255),thickness=2)
            #cv2.circle(image, (cx,cy),(int)(2),(0,0,255),thickness=2)
            cv2.circle(black_image, (cx,cy),(int)(2),(0,0,255),thickness=1)
            #print ("Area: {}, Perimeter: {}".format(area, perimeter))
    if(j>175):
        cv2.imwrite(DIRECTORY+"/savedImage{}.jpg".format(datetime.now()),image)#CHECK DIRECTORY
        cv2.imwrite(DIRECTORY+"/savedImage{}.jpg".format(datetime.now()),black_image)#CHECK DIRECTORY
        print("proccesed images are saved!")
        j=0
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
    draw_contours(black_image,rgb_image, contours,(0,255,0),100,"WHITE SOIL",thick=2)

def detectGray(image_frame,black_image):
    grayLower =(13, 35, 181)
    grayUpper = (15, 43, 201)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, grayLower, grayUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(0, 0,255),100,"GRAY SOIL",thick=2)
def detectYellow(image_frame,black_image):
    grayLower =(16, 52, 188)
    grayUpper = (255, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, grayLower, grayUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(255, 255,0),5000,"YELLOW SOIL",thick=2)
def detectRocks(image_frame,black_image):
    rockLower =(82, 8, 24)
    rockUpper = (255, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, rockLower, rockUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(255, 0,0),1,"ROCKS",thick=1)
def detectRed(image_frame,black_image):
    rockLower =(0, 59, 0)
    rockUpper = (11, 125, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, rockLower, rockUpper)
    contours = getContours(binary_image_mask)
    draw_contours(black_image,rgb_image, contours,(0, 255,255),500,"TERRAIN",thick=1)
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
    draw_contours(black_image,rgb_image, contours,(0, 0, 0),1000,"RIGHT",thick=3)

def save_callback(msg):
    try:
        image = rospy.wait_for_message(msg.data, Image, timeout=5.0)
    except rospy.ROSException as e:
        rospy.logerr("Failed to retrieve image: %s" % (e,))
        return

    try:
        cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Failed to convert image: %s" % (e,))
        return



    cv2.imwrite(DIRECTORY+"/savedImageByUser{}.jpg".format(datetime.now()),cv2_img)# CHECK DIRECTORY

    rospy.loginfo("Saved image from %s topic to %s directory", msg.data, DIRECTORY)
    #print("Saved image from %s topic to %s directory", msg.data,DIRECTORY)



def image_callback(ros_image):
  #print 'got an image'
  global i
  global bridge
  global spacePressed
  i+=1
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv---------
  if(show_image):
    cv2.imshow("RGB Image",cv_image)
    cv2.waitKey(1)
  frame=cv_image
  black_image = np.zeros([frame.shape[0],frame.shape[1],3],'uint8')
  
  if  i>=25:
   cv2.imwrite(DIRECTORY+"/savedImage{}.jpg".format(datetime.now()),frame)# CHECK DIRECTORY
   print("raw image is saved!")
   i=0

  if spacePressed:
    cv2.imwrite(DIRECTORY+"/savedImageByUser{}.jpg".format(datetime.now()),frame)# CHECK DIRECTORY
    print("USER : raw image is saved!")
    spacePressed = False


   
  
  
  detectIsland(frame,black_image)

  detectGray(frame,black_image)

  detectRocks(frame,black_image)

  detectRed(frame,black_image)

  detectOldRocks(frame,black_image)

  detectRight(frame,black_image)

  detectYellow(frame,black_image)
  if(show_image):
    cv2.imshow("RGB Image Contours",frame)
    cv2.imshow("Black Image Contours",black_image)
    cv2.waitKey(1)

  
def main(args):
  rospy.init_node('marsyard_image_proccessing', anonymous=True)
  rospy.Subscriber(sub_topic,Image, image_callback) #Check topic /zed2/right_raw/image_raw_color
  rospy.Subscriber("/image_saver/save", String, save_callback)
  #print(type(rospy.get_published_topics()))
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
