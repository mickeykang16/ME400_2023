#! /usr/bin/env python3
import rospy
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from PWM_control import PWM_control
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
        self.controller = PWM_control(bidirectional=True)
        self.imu_subscriber = rospy.Subscriber("imu/data", Imu, self.imu_callback)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)
        self.is_first_imu = False
        self.initial_yaw = 0.0
        self.yaw = 0.0
        
    def timer_callback(self, event):
        if (self.is_first_imu):
            yaw_error_deg = (self.yaw - self.initial_yaw) * 180 / math.pi
            self.controller.force_control(0.0, 0.0, -0.08 * yaw_error_deg)
            # print("current yaw: ", self.yaw * 180 / math.pi)
            print(yaw_error_deg)
            
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x,
                                                    orientation_q.y,
                                                    orientation_q.z, 
                                                    orientation_q.w)
        if (not self.is_first_imu):
            self.last_yaw = yaw
            self.initial_yaw = yaw
            self.is_first_imu = True
        
        self.yaw = yaw

    
if __name__=='__main__':
    
    rospy.init_node("control_node")
    time.sleep(1)
    controller = HovercraftController()
    rospy.spin()


    