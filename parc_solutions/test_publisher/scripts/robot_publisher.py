#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Image, LaserScan
from parc_robot.gps2cartesian import gps_to_cartesian
from cv_bridge import CvBridge, CvBridgeError

import time
import math
import cv2
import numpy as np

l_cam_bridge = CvBridge()
r_cam_bridge = CvBridge()
c_cam_bridge = CvBridge()

lettuce_left_bottom = int(-1)
lettuce_right_top = int(-1)
lettuce_center_left = int(-1)
lettuce_center_right = int(-1)

def image_callback_l(msg):
    global lettuce_left_bottom
    try:
        cv2_img = l_cam_bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2.imshow("left camera raw", cv2_img)
        #cv2.waitKey(20)

        # convert to hsv colorspace
        hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

        # lower bound and upper bound for Green color
        lower_bound = np.array([40, 15, 20])	 
        upper_bound = np.array([110, 255, 255])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        #define kernel size  
        kernel = np.ones((7,7),np.uint8)

        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        #cv2.imshow("left camera post processed", mask)
        #cv2.waitKey(0)

        lettuce_left_bottom = max(np.where(mask == 255)[:][0])
        print("left side lettuce is here: " + str(lettuce_left_bottom))
    except CvBridgeError as e:
        print(e)

def image_callback_r(msg):
    global lettuce
    try:
        cv2_img = r_cam_bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2.imshow("right camera raw", cv2_img)
        #cv2.waitKey(20)

        # convert to hsv colorspace
        hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

        # lower bound and upper bound for Green color
        lower_bound = np.array([40, 15, 20])	 
        upper_bound = np.array([110, 255, 255])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        #define kernel size  
        kernel = np.ones((7,7),np.uint8)

        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        #cv2.imshow("right camera post processed", mask)
        #cv2.waitKey(0)

        lettuce_right_top = min(np.where(mask == 255)[:][0])
        print("right side lettuce is here: " + str(lettuce_right_top))
    except CvBridgeError as e:
        print(e)

def image_callback_c(msg):
    global lettuce_center_left
    global lettuce_center_right    
    try:
        cv2_img = c_cam_bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2.imshow("center camera raw", cv2_img)
        #cv2.waitKey(20)

        # convert to hsv colorspace
        hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

        # lower bound and upper bound for Green color
        lower_bound = np.array([40, 15, 20])	 
        upper_bound = np.array([110, 255, 255])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        #define kernel size  
        kernel = np.ones((7,7),np.uint8)

        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        cv2.imshow("center camera post processed", mask)
        cv2.waitKey(20)

        bottom_row = mask[max(np.where(mask == 255)[:][0])-2][:]
        #lettuce_center_left = min(np.where(mask == 255)[-1][:])
        #lettuce_center_right = max(np.where(mask == 255)[-1][:])

        #print(mask[max(np.where(mask == 255)[:][0])][:])

        for x in range(np.size(bottom_row)-1):
            if (bottom_row[x] == 255) and (bottom_row[x+1] == 0):
                lettuce_center_left = x            
        print(lettuce_center_left)

        for x in range(lettuce_center_left + 10, np.size(bottom_row)-1):
            if (bottom_row[x] == 0) and (bottom_row[x+1] == 255):
                lettuce_center_right = x        
        #print(lettuce_center_right)

        #print("center left lettuce is here: " + str(lettuce_center_left))
        #print("center right lettuce is here: " + str(lettuce_center_right))
    except CvBridgeError as e:
        print(e)

def lidar_callback(msg):
    print (msg)

def gps_based_movement_calculator():
    #calculate the movement necessary to get to the origin from current location
    x_time = abs(x / 0.3)
    y_time = abs(y / 0.3)
    turn_time = math.pi / 0.2
    rospy.loginfo("X time is {:.3f} sec.".format(x_time))
    rospy.loginfo("Y time is {:.3f} sec.".format(y_time))
    rospy.loginfo("Turn time is {:.3f} sec.".format(turn_time))

#def process_image(image):
#    #TBD
#    cv2.fastNlMeansDenoisingColored(image)

def other_things():
    ######## Move Straight ########
    print("Moving Straight to get X in right place")
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    while time.time() - now < x_time:
        robot_cmd_pub.publish(move_cmd)           # publish to Robot
        rate.sleep()

    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    robot_cmd_pub.publish(move_cmd)
    rate.sleep()

    ######## Rotating Counterclockwise ########
    print("Rotating")
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.2            # rotate at 0.2 rad/sec

    now = time.time()
    while time.time() - now < turn_time:
        robot_cmd_pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
    
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    robot_cmd_pub.publish(move_cmd)
    rate.sleep()

    ######## Move Straight ########
    print("Moving Straight to get Y in right place")
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    while time.time() - now < y_time:
        robot_cmd_pub.publish(move_cmd)           # publish to Robot
        rate.sleep()

    ######## Stop ########
    print("Stopping")
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0            # Giving both zero will stop the robot

    gps = rospy.wait_for_message('gps/fix', NavSatFix) # subscribe to the gps topic once.
    x, y = gps_to_cartesian(gps.latitude, gps.longitude) # get the cartesian coordinates from the GPS coordinates.
    rospy.loginfo("The translation from the origin (0,0) to the gps location provided is {:.3f}, {:.3f} m.".format(x, y))

    now = time.time()
    # For the next 1 seconds publish cmd_vel move commands
    while time.time() - now < 1:
        robot_cmd_pub.publish(move_cmd)           # publish to Robot
        rate.sleep()

    print("Exit")

def main_robot_code():
    global lettuce_left_bottom
    global lettuce_right_top
    robot_cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #robot_c_cam_sub = rospy.Subscriber("/camera/image_raw", Image, callback=image_callback_c)
    robot_r_cam_sub = rospy.Subscriber("/right_camera/image_raw", Image, callback=image_callback_r)
    robot_l_cam_sub = rospy.Subscriber("/left_camera/image_raw", Image, callback=image_callback_l)

    # One time read of the GPS data
    #gps = rospy.wait_for_message('gps/fix', NavSatFix) # subscribe to the gps topic once.
    #x, y = gps_to_cartesian(gps.latitude, gps.longitude) # get the cartesian coordinates from the GPS coordinates.
    #rospy.loginfo("The translation from the origin (0,0) to the gps location provided is {:.3f}, {:.3f} m.".format(x, y))

    move_cmd = Twist()

    # Set publish rate at 10 Hz
    rate = rospy.Rate(10)
    now = time.time()  

    # Main loop
    while True:
        move_cmd.linear.x = 0.1
        if (lettuce_left_bottom != 0) and (lettuce_right_top != 0):
            if lettuce_left_bottom > lettuce_right_top:
                    move_cmd.angular.z = -0.005
                    print("moving left")
            else:
                    move_cmd.angular.z = +0.005
                    print("moving right")
        else:
                    move_cmd.angular.z = 0.0

        robot_cmd_pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('robot_publisher', anonymous=False)
    try:
        main_robot_code()
    except rospy.ROSInterruptException:
        pass