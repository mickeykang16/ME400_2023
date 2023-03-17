#! /usr/bin/env python3
import rospy
import time

from adafruit_servokit import ServoKit
from std_msgs.msg import Int32

## initialize the servo kit instance
kit = ServoKit(channels=16)

## callback function for the rear motor
def callback(data):    
    kit.servo[4].angle = data.data

## callback function for the front motor
def callback2(data):
    kit.servo[11].angle = data.data

## callback function for the servo motor
def callback3(data):
    print(data.data)
    kit.servo[15].angle = data.data

def start():
    kit = ServoKit(channels=16)

    ## initalize the esc of the motor
    kit.servo[4].angle = 0
    kit.servo[11].angle = 0
    kit.servo[15].angle = 90
    time.sleep(0.5)

    rospy.Subscriber("rear_motor", Int32, callback)
    rospy.Subscriber("front_motor",Int32, callback2)
    rospy.Subscriber("servo_motor",Int32, callback3)

    # starts the node
    rospy.init_node('motor')
    rospy.spin()

if __name__ == '__main__':
    start()
