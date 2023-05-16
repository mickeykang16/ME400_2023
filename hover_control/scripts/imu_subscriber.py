#! /usr/bin/env python3
import rospy
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import rospkg
import os
import sys

HOVERING_POWER = 60.0
# from tf.transformations import euler_from_quaternion
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
class imuSubscriber:
    def __init__(self):
        self.imu_subscriber = rospy.Subscriber("imu9250", Imu, self.imu_callback)
        self.is_first_imu = False
        self.linear_x_sum = 0.0
        self.linear_y_sum = 0.0
        self.linear_z_sum = 0.0
        self.num_imu_msg = 0
        self.num = 2000
        
        
            
    def imu_callback(self, msg):
        self.num_imu_msg += 1
        if self.num_imu_msg < self.num:
            self.linear_x_sum += msg.linear_acceleration.x
            self.linear_y_sum += msg.linear_acceleration.y
            self.linear_z_sum += msg.linear_acceleration.z
        elif self.num_imu_msg == self.num:
            print("x average:", self.linear_x_sum / self.num_imu_msg)
            print("y average:", self.linear_y_sum / self.num_imu_msg)
            print("z average:", self.linear_z_sum / self.num_imu_msg)

    
if __name__=='__main__':
    
    # os.system('sudo chmod +777 /dev/hidraw2')
    rospy.init_node("imu_subscriber")
    # time.sleep(1)
    controller = imuSubscriber()
    rospy.spin()


    