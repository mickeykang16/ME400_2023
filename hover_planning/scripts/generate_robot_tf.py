#!/usr/bin/env python
import rospy
import tf_conversions
import tf
import tf2_ros
import laser_processing.msg
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np

# ADD IMU
class robotTF():
    def __init__(self, map_size=(82, 42), resolution=0.05):
        self.initialize_origin = False
        rospy.Subscriber('/map', nav_msgs.msg.OccupancyGrid, self.get_map)
        rospy.Subscriber('/lanes', laser_processing.msg.Lanes, self.generate_robot_pose)
        self.map = np.zeros(map_size)
        self.x = 0
        self.y = 0
        self.distance_left = 0
        self.distance_right = 0
        self.distance_front = 0
        self.distance_back = 0
    
    def get_map(self, msg):
        self.map = np.array(msg.data).reshape(self.map.shape)
        self.map = np.where(self.map == -1, 50, self.map)
        print(self.map)

    def generate_robot_pose(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "laser"
        
        if not self.initialize_origin:
            self.x = msg.distance_back
            self.y = -msg.distance_left
            self.initialize_origin = True
        else:
            diff_l = -(msg.distance_left - self.distance_left)
            diff_r = msg.distance_right - self.distance_right
            diff_f = -(msg.distance_front - self.distance_front)
            diff_b = msg.distance_back - self.distance_back
            diff_lr = []
            diff_fb = []
            print(diff_b, diff_f, diff_l, diff_r)
            if abs(diff_l) < 0.15:
                diff_lr.append(diff_l)
            if abs(diff_r) < 0.15:
                diff_lr.append(diff_r)
            if abs(diff_f) < 0.15:
                diff_fb.append(diff_f)
            if abs(diff_b) < 0.15:
                diff_fb.append(diff_b)
            self.x += sum(diff_fb) / len(diff_fb)
            self.y += sum(diff_lr) / len(diff_lr)
        # lane processing, do not provide values when the line does not exist
        # also conside the absolute position, in the case of goint straight msg.distance_back,
        # shorter one will be msg.distance_back+3 => can be classified by the total sum of front and back distance
        # periodically update the self.x to the mean b/w current self.x and absolute values
        t.transform.translation.x = 4-msg.distance_front # self.x #  # back, front
        t.transform.translation.y = -msg.distance_left # self.y #  # left, right
        t.transform.translation.z = 0
        # Currently noisy!
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.distance_left = msg.distance_left
        self.distance_right = msg.distance_right
        self.distance_front = msg.distance_front
        self.distance_back = msg.distance_back

        br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('robot_tf2_broadcaster')
    robot = robotTF()
    rospy.spin()
    # rosbag play -l 2023-05-10-08-16-02.bag --clock