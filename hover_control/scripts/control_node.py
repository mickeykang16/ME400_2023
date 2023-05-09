#! /usr/bin/env python3
import rospy
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from PWM_control import PWM_control
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
    
class HovercraftController:
    def __init__(self):
        self.controller = PWM_control(bidirectional=True, max = 60.0)
        self.imu_subscriber = rospy.Subscriber("imu/data", Imu, self.imu_callback)
        self.cmd_vel_subscriber = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)
        self.debug_timer = rospy.Timer(rospy.Duration(0.5), self.debug_timer_callback)
        self.is_first_imu = False
        self.is_first_twist = False
        self.last_twist_time = rospy.Time.now() - rospy.Duration(1)
        self.initial_yaw = 0.0
        self.yaw = 0.0
        self.yaw_velocity = 0.0
        self.twist_msg = Twist()
        
        
    def timer_callback(self, event):
        twist_time_out = (rospy.Time.now() - self.last_twist_time) > rospy.Duration(0.5)
        if (self.is_first_imu and self.is_first_twist and not twist_time_out and self.twist_msg.linear.z > 0.9):
            if self.controller.get_throttle(0) == 0.0:
                self.controller.set_throttle(0, HOVERING_POWER)
            P = 0.05
            D = 0.01
            yaw_error_deg = (self.yaw - self.initial_yaw) * 180 / math.pi
            yaw_velocity_deg = self.yaw_velocity * 180/ math.pi
            if (yaw_error_deg > 180.0):
                yaw_error_deg -= 360.0
            elif (yaw_error_deg < -180.0):
                yaw_error_deg += 360.0
            self.controller.force_control(self.twist_msg.linear.x, \
                self.twist_msg.linear.y, 
                -P * yaw_error_deg - D * yaw_velocity_deg)
            # self.controller.force_control(0, \
            #     0, 
            #     -P * yaw_error_deg - D * yaw_velocity_deg)
        else:
            self.controller.stop_all()
            
    def debug_timer_callback(self, event):
        print("Hovering motor:", self.controller.get_throttle(0), "%")
        print("FL motor:", self.controller.get_throttle(1), "%")
        print("FR motor:", self.controller.get_throttle(2), "%")
        print("Tail motor:", self.controller.get_throttle(3), "%")
        
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x,
                                                    orientation_q.y,
                                                    orientation_q.z, 
                                                    orientation_q.w)
        yaw = yaw
        if (not self.is_first_imu):
            self.last_yaw = yaw
            self.initial_yaw = yaw
            self.is_first_imu = True
        self.yaw = yaw
        self.yaw_velocity = msg.angular_velocity.z
        
        #print("Initial yaw:",self.initial_yaw, "\nCurrent yaw:", self.yaw)
    
    def twist_callback(self, msg):
        self.last_twist_time = rospy.Time.now()
        if (not self.is_first_twist):
            self.is_first_twist = True
        
        self.twist_msg = msg

    
if __name__=='__main__':
    
    # os.system('sudo chmod +777 /dev/hidraw2')
    rospy.init_node("control_node")
    # time.sleep(1)
    controller = HovercraftController()
    rospy.spin()


    