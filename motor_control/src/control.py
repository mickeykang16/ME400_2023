#! /usr/bin/env python
import rospy
import time
import pigpio
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot
pi = pigpio.pi()
pi.set_mode(18,pigpio.OUTPUT)
pi.set_mode(12,pigpio.OUTPUT)
# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    val1 = abs(data.axes[4])*50000+50000
    val2 = abs(data.axes[3])*50000+50000
    pi.hardware_PWM(18,50,val1)
    pi.hardware_PWM(13,50,val2)
# Intializes everything
def start():
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()