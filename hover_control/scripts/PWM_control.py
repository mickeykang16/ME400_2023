from board import SCL, SDA
import busio
import numpy as np
import math
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
import rospkg
PWM_FREQUENCY = 50
MAX_THROTTLE_MS = 2.0
MIN_THROTTLE_MS = 1.0
GRAVITY=9.81

# use SI unit excpet unit is specified
class PWM_control:
    def __init__(self, bidirectional=False, radius_cm = 19.0):
        rospack = rospkg.RosPack()
        rospack.list()
                
        # i2c_bus = busio.I2C(SCL, SDA)
        i2c_bus = busio.I2C(1, 0)
        # Create a simple PCA9685 class instance.
        pca = PCA9685(i2c_bus)
        # Set the PWM frequency to 60hz.
        pca.frequency = PWM_FREQUENCY
        self.pwm_ = pca
        self.throttle_ = [0.0, 0.0, 0.0, 0.0]
        self.bidirectional = bidirectional
        
        # init throttle thrust mapping
        self.thrust_map = []
        self.zero_throttle_idx = -1
        f = open(rospack.get_path('hover_control') + "/data/thrust_new.txt", 'r')
        lines = f.readlines()
        idx = 0
        for line in lines:
            tokens = line.strip().split()
            if len(tokens) == 1:
                self.thrust_map.append([int(tokens[0]), None])
            elif len(tokens) == 2:
                throttle = int(tokens[0])
                self.thrust_map.append([throttle, float(tokens[1])/1000*GRAVITY])
                if throttle == 0:
                    self.zero_throttle_idx = idx
            else:
                pass
            idx = idx + 1
        assert self.zero_throttle_idx > 0
        self.thrust_map.reverse()
        print(self.thrust_map)
        
        # init control_matrix
        #        -r        r        r
        #   sin(pi/3) sin(pi/3)     0
        #  -cos(pi/3) cos(pi/3)     -1
        # -> front_left, front_right, back 
        
        sin = math.sin(math.pi/3)
        cos = math.cos(math.pi/3)
        self.control_mat = np.array([[-radius_cm/100.0, radius_cm/100.0, radius_cm/100.0],[sin, sin, 0.0],[-cos, cos, -1.0]])
        # self.control_mat_inv = np.linalg.inv(control_mat)
        self.stop_all()

    def __del__(self):
        self.stop_all()

    def set_throttle(self, channel=0, throttle_input=0.0):
        if self.bidirectional:
            throttle_percent = min(100.0, max(-100.0, throttle_input))
        else:
            throttle_percent = min(100.0, max(0.0, throttle_input))
        assert channel >= 0 and channel <= 3, 'There are only four channels [0, 3]'

        self.throttle_[channel] = throttle_percent
        if self.bidirectional:
            ms = MIN_THROTTLE_MS + (100.0 + throttle_percent) * (MAX_THROTTLE_MS - MIN_THROTTLE_MS) / 200
        else:
            ms = MIN_THROTTLE_MS + throttle_percent * (MAX_THROTTLE_MS - MIN_THROTTLE_MS) / 100
        ms = min(MAX_THROTTLE_MS, max(MIN_THROTTLE_MS, ms))
        # TODO: FIND OUT REASON FOR THIS OFFSET LATER
        ms = ms + (1.625 - 1.5)
        print(ms)
        duty_cyc = int(0xffff * self.pwm_.frequency * ms / 1000)
        # print(duty_cyc)
        self.pwm_.channels[channel].duty_cycle = duty_cyc
        
    def stop_all(self):
        self.set_throttle(0, 0.0)
        self.set_throttle(1, 0.0)
        self.set_throttle(2, 0.0)
        self.set_throttle(3, 0.0)

    def get_throttle(self, channel=0):
        assert channel >= 0 and channel <= 3, 'There are only four channels [0, 3]'
        return self.throttle_[channel]
    
    def force_to_throttle(self, force = 0.0):
        # TODO: find nice mapping between force and % throttle
        # for now on, just naively assume linear with some constant factor
        
        if not self.bidirectional and force < 0.0:
            return 0.0
        
        # force should be between map[lower] and map[higher] 
        idx1 = 0
        idx2 = 0
        if (force >= 0.0):
            for i in range(self.zero_throttle_idx, len(self.thrust_map) - 1):
                lower = self.thrust_map[i][1]
                upper = self.thrust_map[i+1][1]
                if force >= lower:
                    if upper == None:
                        return self.thrust_map[i][0]
                    elif force <= upper:
                        idx1 = i
                        idx2 = i+1
                        break
        else:
            for i in range(self.zero_throttle_idx, 1, -1):
                lower = self.thrust_map[i][1]
                upper = self.thrust_map[i-1][1]
                if force <= lower:
                    if upper == None:
                        return self.thrust_map[i][0]
                    elif force >= upper:
                        idx1 = i-1
                        idx2 = i
                        break   
        t1 = self.thrust_map[idx1][0]
        t2 = self.thrust_map[idx2][0]
        f1 = self.thrust_map[idx1][1]
        f2 = self.thrust_map[idx2][1]
        r = (force - f1)/(f2 - f1)
        throttle = t1 + r * (t2 - t1)
        return throttle
        
    def force_control(self, f_x = 0.0, f_y = 0.0, torque = 0.0):
        desired_f = np.array([torque, f_x, f_y])
        motor_force = np.linalg.solve(self.control_mat, desired_f)
        self.set_throttle(1, self.force_to_throttle(motor_force[0]))
        self.set_throttle(2, self.force_to_throttle(motor_force[1]))
        self.set_throttle(3, self.force_to_throttle(motor_force[2]))