#!/usr/bin/env python
"""
Script to move Robot
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from parc_robot.gps2cartesian import gps_to_cartesian
import time
import math

def move_robot():
    rospy.init_node('robot_publisher', anonymous=True)
    # Create a publisher which can "talk" to Robot and tell it to move
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set publish rate at 10 Hz
    rate = rospy.Rate(10)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()

    # Run GPS test code to make sure it works
    gps = rospy.wait_for_message('gps/fix', NavSatFix) # subscribe to the gps topic once.
    x, y = gps_to_cartesian(gps.latitude, gps.longitude) # get the cartesian coordinates from the GPS coordinates.
    rospy.loginfo("The translation from the origin (0,0) to the gps location provided is {:.3f}, {:.3f} m.".format(x, y))

    #calculate the movement necessary to get to the origin from current location
    x_time = abs(x / 0.3)
    y_time = abs(y / 0.3)
    turn_time = math.pi / 0.2
    rospy.loginfo("X time is {:.3f} sec.".format(x_time))
    rospy.loginfo("Y time is {:.3f} sec.".format(y_time))
    rospy.loginfo("Turn time is {:.3f} sec.".format(turn_time))
    

    ######## Move Straight ########
    print("Moving Straight to get X in right place")
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    while time.time() - now < x_time:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()

    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rate.sleep()

    ######## Rotating Counterclockwise ########
    print("Rotating")
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.2            # rotate at 0.2 rad/sec

    now = time.time()
    while time.time() - now < turn_time:
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()
    
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rate.sleep()

    ######## Move Straight ########
    print("Moving Straight to get Y in right place")
    move_cmd.linear.x = 0.3             # move in X axis at 0.3 m/s
    move_cmd.angular.z = 0.0

    now = time.time()
    while time.time() - now < y_time:
        pub.publish(move_cmd)           # publish to Robot
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
        pub.publish(move_cmd)           # publish to Robot
        rate.sleep()

    print("Exit")


if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
