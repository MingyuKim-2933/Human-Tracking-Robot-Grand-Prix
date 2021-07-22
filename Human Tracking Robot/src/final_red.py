#!/usr/bin/env python
# -*coding:utf-8-*-

from __future__ import print_function
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import sys, select, termios, tty
import time
import datetime

# COLOR INFO
BLUE = ((94,80,20), (126,255,255))
RED = ((161,155,84), (179,255,255))
RED_1 = ((170,50,50), (10,255,255))
YELLOW = ((22,93,0), (45,255,255))
GREEN = ((95,50,50),(110,255,255))
PINK = ((160,80,50),(170,255,255))

# CAMERA RES
RES_WIDTH = 1280
RES_HEIGHT = 720

# DRIVE VARIABLES
TURN = 0.65
TURN_30 = 0.8
TURN_60 = 0.45
SPEED = 0.65

# LIDAR VARIABLES 
AVOID_DISTANCE = 0.7

LEFT_DEGREE = 60
RIGHT_DEGREE = 3//60-LEFT_DEGREE

# YOLO VARIABLES
YOLO_LEFT = 500
YOLO_RIGHT = RES_WIDTH-YOLO_LEFT
YOLO_WEIGHT = 1

# CAMERA VARIABLES
HSV = np.zeros((RES_HEIGHT, RES_WIDTH, 3))
BGR = np.zeros((RES_HEIGHT, RES_WIDTH, 3))

# RECOGNITION DISTANCE 
PERSON_WIDTH = 150
TRAFFIC_WIDTH = 35
STOP_WIDTH = 120

# DRIVE CONSTANTS
STEER_PUB = None
X = 1
Y = 0
Z = 0
RIGHT_TH = -0.7
LEFT_TH = -1 * RIGHT_TH
GO_TH = 0
POS = 'MIDDLE'

# LIDAR CONTAINERS
LIDAR = []
RIGHT_LIDAR_30 = False
LEFT_LIDAR_30 = False
RIGHT_LIDAR_60 = False
LEFT_LIDAR_60 = False

# YOLO CONTAINER
BOX_DATA = []

# Behavior 
stop = False
exit = False
stop_check = 0
exit_check = 0
second_phase = False

def init_node():
    global STEER_PUB
    rospy.init_node("teleop_twist_keyboard")
    rospy.Subscriber("/scan", LaserScan, callback_lidar)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback_yolo)
    rospy.Subscriber('/usb_cam/image_raw', Image, callback_camera)
    STEER_PUB = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def callback_lidar(data):
    global LIDAR
    LIDAR = data.ranges

def callback_yolo(msg):
    global BOX_DATA
    BOX_DATA = msg

def callback_camera(data):
    global HSV
    bridge = CvBridge()
    try:
	    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as e:
	    print("converting error")
	    print(e)
    
    HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    

def check_color(y, x, h1, h2, y_min, y_max):
    if y-40 <= 0 or y+40 >= RES_HEIGHT or x-40 <= 0 or x+40 >= RES_WIDTH:
	return False
    y = int(y)
    x = int(x)
    check = 0

    for i in [y-40,y-30,y-20,y-10,y,y+10,y+20, y+30, y+40]: # 9px
        for j in [x-40, x-30,x-20,x-10,x,x+10,x+20, x+30, x+40]: # 9px
            
            if (h1[0] <= HSV[i][j][0] <= 180 or 0 <= HSV[i][j][0] <= h2[0]) and h1[1] <= HSV[i][j][1] <= h2[1] and h1[2] <= HSV[i][j][2] <= h2[2]:
                check+=1
	    if check > 30:
		return True
    if check <= 30:
	return False
    
def drive(x, y, z, th, speed, turn):
    global STEER_PUB

    twist = Twist()
    twist.linear.x = x * speed
    twist.linear.y = y * speed
    twist.linear.z = z * speed
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = th * turn

    STEER_PUB.publish(twist)

def detect_lidar():
    global LIDAR
    global RIGHT_LIDAR_30
    global RIGHT_LIDAR_60
    global LEFT_LIDAR_30
    global LEFT_LIDAR_60

    RIGHT_LIDAR_30 = False
    RIGHT_LIDAR_60 = False
    LEFT_LIDAR_30 = False
    LEFT_LIDAR_60 = False
    check = 0
    
    if len(LIDAR) != 360:
	    return True

    for i in range(360):                                                                                   
        if 30 < i < 60: # left_60
            if LIDAR[i] <= AVOID_DISTANCE:      
                RIGHT_LIDAR_30 = False
                LEFT_LIDAR_30 = False
                RIGHT_LIDAR_60 = False
                LEFT_LIDAR_60 = True		
                check += 1

        if 300 < i < 330: # right_60 
            if LIDAR[i] <= AVOID_DISTANCE:
                RIGHT_LIDAR_30 = False
                LEFT_LIDAR_30 = False
                RIGHT_LIDAR_60 = True
                LEFT_LIDAR_60 = False		
                check += 1

        if 0 <= i <= 30: # left_30
            if LIDAR[i] <= AVOID_DISTANCE: 
                RIGHT_LIDAR_30 = False
                LEFT_LIDAR_30 = True
                RIGHT_LIDAR_60 = False	
                LEFT_LIDAR_60 = False		
                check += 1

        if  330 <= i <= 360: # right_30 
            if LIDAR[i] <= AVOID_DISTANCE:
                RIGHT_LIDAR_30 = True
                LEFT_LIDAR_30 = False
                RIGHT_LIDAR_60 = False
                LEFT_LIDAR_60 = False		
                check += 1

        if check > 3:
            return True
    return False
        
if __name__=="__main__":
    init_node()
    print("!!Start Running!!")
    follow_pos = 600

    while not rospy.is_shutdown():
	#second_phase = True 
        # INIT
        x,y,z,th,speed,turn = X,Y,Z,GO_TH,SPEED,TURN        

        boxes = BOX_DATA
        person_width = 0

        # DETECT OBJECTS
        if boxes: 
            for i in range(len(boxes.bounding_boxes)):
                if boxes.bounding_boxes[i].Class == 'person':
                    center_x = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin)/2
                    center_y = (boxes.bounding_boxes[i].ymax + boxes.bounding_boxes[i].ymin)/2
		    if abs(boxes.bounding_boxes[i].xmax - boxes.bounding_boxes[i].xmin) > 100:
                        if check_color(center_y, center_x, RED_1[0], RED_1[1], boxes.bounding_boxes[i].ymin, boxes.bounding_boxes[i].ymax):
                            follow_pos = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin)/2
                            person_width = abs(boxes.bounding_boxes[i].xmax - boxes.bounding_boxes[i].xmin)
                if boxes.bounding_boxes[i].Class == 'traffic light' and not second_phase:
                    if abs(boxes.bounding_boxes[i].xmax - boxes.bounding_boxes[i].xmin) > TRAFFIC_WIDTH:
                        traffic_light_pos = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin)/2
                        stop_check += 1
			print(stop_check)
                if boxes.bounding_boxes[i].Class == 'stop sign' and second_phase:
                    if abs(boxes.bounding_boxes[i].xmax - boxes.bounding_boxes[i].xmin) > STOP_WIDTH:
                        exit_check += 1
	
        if stop_check > 20 and not second_phase:
            stop = True
            x,y,z,th,speed,turn = X,Y,Z,-1,0,TURN
            i = 0
            while i < 16000:
                drive(x,y,z,th,speed,1)
                person_width = 0
                i+=1
            second_phase = True
            print("SECOND PHASE")

        if exit_check > 350 and second_phase:
            exit = True

        # ACTIONS WHEN SEE SIGNS
        if stop and second_phase:
            if person_width > PERSON_WIDTH:
                stop = False
                print('Go Again')
            continue
        if exit:
            print("EXIT")
            break

        # DECIDE PERSON'S SIDE
        YOLO_WEIGHT = 0.001*(abs(RES_WIDTH/2 - follow_pos))+0.56 # maximum 1.2 / minimum 0.8
        if follow_pos < YOLO_LEFT:
            POS = 'LEFT'
        elif follow_pos > YOLO_RIGHT: 
            POS = 'RIGHT'
        else:
            POS = 'MIDDLE'
            YOLO_WEIGHT = 1
        
        # IF NO OBSTACLES
        if not detect_lidar():
            if POS == "RIGHT":
                x,y,z,th,speed,turn = X,Y,Z,RIGHT_TH,SPEED,TURN * YOLO_WEIGHT
            if POS == "LEFT":
                x,y,z,th,speed,turn = X,Y,Z,LEFT_TH,SPEED,TURN * YOLO_WEIGHT
            if POS == "MIDDLE":
                x,y,z,th,speed,turn = X,Y,Z,GO_TH,SPEED,TURN * YOLO_WEIGHT
        # IF OBSTACLES
        else:
            if RIGHT_LIDAR_30:     
                x,y,z,th,speed,turn = X,Y,Z,LEFT_TH,SPEED,TURN_30
            elif LEFT_LIDAR_30:
                x,y,z,th,speed,turn = X,Y,Z,RIGHT_TH,SPEED,TURN_30
            elif RIGHT_LIDAR_60:     
                x,y,z,th,speed,turn = X,Y,Z,LEFT_TH,SPEED,TURN_60
            elif LEFT_LIDAR_60:
                x,y,z,th,speed,turn = X,Y,Z,RIGHT_TH,SPEED,TURN_60

	#DRIVE!
    	drive(x,y,z,th,speed,turn)
    

