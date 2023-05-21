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
# sys.path.remove(os.path.dirname(__file__))
from std_msgs.msg import Float32
from laser_processing.msg import ControlDebug
from ds4_driver.msg import Status
from nav_msgs.msg import Odometry

HOVERING_POWER = 55.0

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

def get_yaw(orientation_q):
    (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x,
                                                    orientation_q.y,
                                                    orientation_q.z, 
                                                    orientation_q.w)
    return yaw

class HovercraftController:
    def __init__(self):
        self.is_first_imu = False
        self.is_first_twist = False
        self.is_autonomous = False
        self.last_twist_time = rospy.Time.now() - rospy.Duration(2)
        self.last_imu_time = rospy.Time.now() - rospy.Duration(2)
        self.initial_yaw = 0.0
        self.yaw = 0.0
        self.yaw_velocity = 0.0
        self.twist_msg = Twist()
        self.last_odom_time = rospy.Time.now() - rospy.Duration(2)
        self.odom_msg = Odometry()
        self.last_target_time = rospy.Time.now() - rospy.Duration(2)
        self.target_msg = Odometry()
        self.debug_str = ""
        self.controller = PWM_control(bidirectional=True, max = 60.0)
        self.odom_subscriber = rospy.Subscriber("robot_odom", Odometry, self.odom_callback)
        self.target_subscriber = rospy.Subscriber("target_odom", Odometry, self.target_callback)
        self.imu_subscriber = rospy.Subscriber("imu/data", Imu, self.imu_callback)
        self.cmd_vel_subscriber = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)
        self.joy_subscriber = rospy.Subscriber("status", Status, self.joy_callback)
        self.debug_timer = rospy.Timer(rospy.Duration(0.5), self.debug_timer_callback)
        self.debug_publisher = rospy.Publisher("control/debug", ControlDebug, queue_size=5)
        # timer callback for controller
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)
        
    def is_manual_timeout(self):
        twist_time_out = (rospy.Time.now() - self.last_twist_time) > rospy.Duration(0.5)
        imu_time_out = (rospy.Time.now() - self.last_imu_time) > rospy.Duration(0.2)
        
        if twist_time_out:
            self.debug_str = "Control Error: Joystick timeout"
        elif imu_time_out:
            self.debug_str = "Control Error: IMU timeout"
        return twist_time_out or imu_time_out
    
    def is_auto_timeout(self):
        odom_time_out = (rospy.Time.now() - self.last_odom_time) > rospy.Duration(0.2)
        target_time_out = (rospy.Time.now() - self.last_target_time) > rospy.Duration(1.0)
        
        if odom_time_out:
            self.debug_str = "Control Error: Odometry timeout"
        elif target_time_out:
            self.debug_str = "Control Error: Target point timeout"
        
        return odom_time_out or target_time_out
        
    def timer_callback(self, event):
        debug_msg = ControlDebug()
        yaw_error_deg = 0
        yaw_velocity_deg = 0
        # Operation Condition
        # 1. Imu received at least once
        # 2. twist(from joy) received at leat once
        # 3. No timeout msgs
        can_operate = self.is_first_imu and self.is_first_twist and (not self.is_manual_timeout())
        
        yaw_P = 0.05
        yaw_D = 0.008
        
        pose_P = 0.1
        pose_D = 0.00
        # Manual operation case
        if (can_operate and (not self.is_autonomous) and self.twist_msg.linear.z > 0.9):
            if self.controller.get_throttle(0) == 0.0:
                self.controller.set_throttle(0, HOVERING_POWER)
            
            yaw_error_deg = -(self.yaw - self.initial_yaw) * 180 / math.pi
            yaw_velocity_deg = -self.yaw_velocity * 180/ math.pi
            if (yaw_error_deg > 180.0):
                yaw_error_deg -= 360.0
            elif (yaw_error_deg < -180.0):
                yaw_error_deg += 360.0
                
            self.controller.force_control(self.twist_msg.linear.x, \
                self.twist_msg.linear.y, 
                yaw_P * yaw_error_deg + yaw_D * yaw_velocity_deg)
        # Autonomous
        elif (can_operate and self.is_autonomous and (not self.is_auto_timeout())):
            # Activate main hovering motor
            if self.controller.get_throttle(0) == 0.0:
                self.controller.set_throttle(0, HOVERING_POWER)
            
            target_yaw = get_yaw(self.target_msg.pose.pose.orientation)
            
            yaw_error_deg = (self.initial_yaw - self.yaw) * 180 / math.pi
            yaw_velocity_deg = -self.yaw_velocity * 180/ math.pi
            if (yaw_error_deg > 180.0):
                yaw_error_deg -= 360.0
            elif (yaw_error_deg < -180.0):
                yaw_error_deg += 360.0
                
            pose_error_x = self.target_msg.pose.pose.position.x - self.odom_msg.pose.pose.position.x
            pose_error_y = self.target_msg.pose.pose.position.y - self.odom_msg.pose.pose.position.y
            vel_error_x = self.target_msg.twist.twist.linear.x - self.odom_msg.twist.twist.linear.x
            vel_error_y = self.target_msg.twist.twist.linear.y - self.odom_msg.twist.twist.linear.y
            
            
            self.controller.force_control(pose_P * pose_error_x + pose_D * vel_error_x,
                                          pose_P * pose_error_y + pose_D * vel_error_y, 
                                        yaw_P * yaw_error_deg + yaw_D * yaw_velocity_deg)
            debug_msg.pose_x_error = pose_error_x
            debug_msg.pose_y_error = pose_error_y
            debug_msg.vel_x_error = vel_error_x
            debug_msg.vel_y_error = vel_error_y
            
        # Only command "stop" if it is not stopped
        elif (self.controller.get_throttle(0) != 0.0):
            self.controller.stop_all()
            self.debug_str = "Stopped"
        else:
            self.debug_str = "Standby"
                    
        # make debug msg and publish
        
        debug_msg.header.stamp = rospy.Time.now()
        debug_msg.motor_0_thrust = self.controller.get_throttle(0)
        debug_msg.motor_1_thrust = self.controller.get_throttle(1)
        debug_msg.motor_2_thrust = self.controller.get_throttle(2)
        debug_msg.motor_3_thrust = self.controller.get_throttle(3)
        debug_msg.yaw_rad = self.yaw
        debug_msg.initial_yaw_rad = self.initial_yaw
        debug_msg.torque = self.controller.get_torque()
        debug_msg.f_x = self.controller.get_f_x()
        debug_msg.f_y = self.controller.get_f_y()
        debug_msg.is_auto = self.is_autonomous
        debug_msg.yaw_error_deg = yaw_error_deg
        debug_msg.vel_yaw_error_deg = yaw_velocity_deg
        self.debug_publisher.publish(debug_msg)
            
    def debug_timer_callback(self, event):
        if (self.is_autonomous):
            mode = "AUTO"
        else:
            mode = "Manual"
        print("Control Mode: ", mode)
        print("Hovering motor:", self.controller.get_throttle(0), "%")
        print("FL motor:", self.controller.get_throttle(1), "%")
        print("FR motor:", self.controller.get_throttle(2), "%")
        print("Tail motor:", self.controller.get_throttle(3), "%")
        print("Debug: ", self.debug_str)
        print("-----------------------------------")
        
    def imu_callback(self, msg):
        self.last_imu_time = rospy.Time.now()
        orientation_q = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x,
                                                    orientation_q.y,
                                                    orientation_q.z, 
                                                    orientation_q.w)

        if (not self.is_first_imu):
            self.initial_yaw = yaw
            self.is_first_imu = True
        self.yaw = yaw
        self.yaw_velocity = msg.angular_velocity.z
            
    def twist_callback(self, msg):
        self.last_twist_time = rospy.Time.now()
        if (not self.is_first_twist):
            self.is_first_twist = True
        self.twist_msg = msg
    
    def joy_callback(self, msg):
        auto_button = bool(msg.button_triangle)
        left_kill = bool(msg.button_l2)
        right_kill = bool(msg.button_r2)
        manual_button = bool(msg.button_cross)
        # pushed both kill switches or manual button, switch to manual mode
        if (self.is_autonomous and ((left_kill and right_kill) or manual_button)):
            self.is_autonomous = False
        # pushed auto button and not pushed manual button, switch to auto mode
        elif ((not self.is_autonomous) and auto_button and (not manual_button)):
            self.is_autonomous = True
    
    def odom_callback(self, msg):
        self.last_odom_time = rospy.Time.now()
        self.odom_msg = msg
        
    def target_callback(self, msg):
        self.last_target_time = rospy.Time.now()
        self.target_msg = msg

    
if __name__=='__main__':
    rospy.init_node("control_node")
    controller = HovercraftController()
    rospy.spin()
