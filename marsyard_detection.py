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
import argparse
import argparser
import os.path
from pynput.keyboard import Listener  #used for save image from terminal
from matplotlib import pyplot as plt
from numpy.lib.type_check import imag


spacePressed = False
p_Pressed = False

#THIS SCRIPT TAKES FOUR ARGUMENTS : dir,subs, showIP, showPH
#ARGUMENTS CAN BE GIVEN AS FOLLOW : python marsyard_detection.py --kwargs dir=<valid_string_directory> subs=<valid_string_topic> showIP=<valid_boolean_decision>,showPH=<valid_boolean_decision>
#ORDER OF ARGUMENTS IS NOT IMPORTANT !
intialTracbarVals = [153,498,160,670]


#If i = 1 , j must be 7 to save images concurrently.
i=0 
j=0

def keyPressed(key):
    global spacePressed
    global p_Pressed
    if(str(key) == "Key.space"):
        spacePressed = True
    if(str(key) == "u'p'"):
        p_Pressed = True

listener = Listener(on_press=keyPressed)
listener.start()



DIRECTORY = "/mars"
sub_topic = "/zed2/left_raw/image_raw_color"
show_image = False
show_PH = False

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
    if(args.kwargs.has_key("showIP")):
        show_image = bool(args.kwargs["showIP"])
    if(args.kwargs.has_key("showPH")):
        show_PH = bool(args.kwargs["showPH"])
else:
    print("No arguments passed. Default values will be used.")

if(os.path.isdir(DIRECTORY)!=1):
    print("Directory is not a valid one!")
    print("Exiting...")
    sys.exit(0)

print("Subscribed Topic : "+sub_topic)
print("Treasure Directory : "+DIRECTORY)
print("Show Image Processes  : "+ str(show_image))
print("Show PH : "+str(show_PH))


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
  #print('got an image')
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

""" BELOW IS BELONG TO SALIH  BORDER"""

def graph(formula, x_range,percent_of_Red):

    x = np.array(x_range)  
    y = eval(formula)
    
        # naming the x axis
    plt.xlabel('Red (%)')
    # naming the y axis
    plt.ylabel('pH Values')
    plt.plot(x, y)  
    plt.plot(percent_of_Red,0.0956 * percent_of_Red + 4.2722 ,'ro') 
    plt.annotate("Predicted pH: "+  str(0.0956 * percent_of_Red + 4.2722), (percent_of_Red,0.0956 * percent_of_Red + 4.2722 ))
    plt.show()
def image_processor(Image):

    try:
      img_cv2 = bridge.imgmsg_to_cv2(Image, "bgr8")
    except CvBridgeError as e:
      print(e)

    # image wrapping
    img_wrapped = warpImg(img_cv2,valTrackbars(),1000,800) 
    # image_bgr = cv2.imread('/home/salih/Desktop/Mars.jpg', cv2.IMREAD_COLOR)


    channels = cv2.mean(img_wrapped)
    # Swap blue and red values (making it RGB, not BGR)
    observation = np.array([(channels[2], channels[1], channels[0])])
    #print(observation)

    mean_of_Red   = channels[2]
    mean_of_Green = channels[1]
    mean_of_Blue  = channels[0]

    # red percent will be used for forecasting pH
    percent_of_Red = mean_of_Red * 100 /(mean_of_Red+mean_of_Blue+mean_of_Green)
   # print(str(percent_of_Red)+" %")

    # equation taken from : http://przyrbwn.icm.edu.pl/APP/PDF/132/app132z3-IIp086.pdf & https://core.ac.uk/download/pdf/158352623.pdf

    # y = 0.0956 * x + 4.2722 ==> y: pH and x: Red Values (%)

    #graph('0.0956*x+4.2722', range(0, 100),percent_of_Red)
    predicted_pH = 0.0956 * percent_of_Red + 4.2722
    
   
    # Text Settings:
        # font
    font = cv2.FONT_HERSHEY_SIMPLEX
      
    # org
    org = (250, 500)
      
    # fontScale
    fontScale = 1
      
    # Blue color in BGR
    color = (255, 0, 0)
      
    # Line thickness of 2 px
    thickness = 2
      
    # Using cv2.putText() method
    image = cv2.putText(img_cv2, 'Predicted pH: '+str(predicted_pH), org, font, 
                      fontScale, color, thickness, cv2.LINE_AA)
    img_drawn = drawPoints(image,valTrackbars())
    if(show_PH):
      cv2.imshow("Image",img_drawn)
      cv2.waitKey(1)
      
    else:
      global p_Pressed
      #rospy.loginfo("Predicted pH: "+ str(predicted_pH ))
      if ( p_Pressed== True):
        cv2.imwrite(DIRECTORY+"/savedImage_PH{}.jpg".format(datetime.now()),img_drawn)    #Need to be checked if location is fine ? 
        print("pH Forecasted image saved SUCCESSFULLY!")
        p_Pressed = False

def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

def nothing():
  pass

def initializeTrackbars(intialTracbarVals, wT=1000, hT=800):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0], wT // 2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2], wT // 2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)


def valTrackbars(wT=1000, hT=600):

  if show_PH==False:
    
    widthTop = intialTracbarVals[0]
    heightTop = intialTracbarVals[1]
    widthBottom = intialTracbarVals[2]
    heightBottom = intialTracbarVals[3]
  
  
  else :
     widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
     heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
     widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
     heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
     
  
  points = np.float32([(widthTop, heightTop), (wT - widthTop, heightTop),
                    (widthBottom, heightBottom), (wT - widthBottom, heightBottom)])
  return points


         
    


def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
    return img

""" BORDER """
def main():
  rospy.init_node('marsyard_image_proccessing', anonymous=True)
  rospy.Subscriber(sub_topic,Image, image_processor)
  rospy.Subscriber(sub_topic,Image, image_callback) #Check topic /zed2/right_raw/image_raw_color
  rospy.Subscriber("/image_saver/save", String, save_callback)
  #print(type(rospy.get_published_topics()))
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    if(show_PH == True):
        initializeTrackbars(intialTracbarVals)
    main()

