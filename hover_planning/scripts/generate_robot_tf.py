#!/usr/bin/env python
import rospy, rospkg
import tf_conversions, tf2_ros
import laser_processing.msg
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
import os, json
from math import sqrt

class Waypoints():
    def __init__(self, pt_type=0, x=0, y=0, vx=0, vy=0, yaw=0):
        self.type = pt_type
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.quat = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)

class robotTF():
    def __init__(self, map_size=(202, 92), resolution=0.05):
        self.init_yaw = 0
        # publish nav_msgs/Odometry.msg
        # rospy.Subscriber('/imu/data', sensor_msgs.msg.Imu, self.update_with_imu) # only yaw? use it latter
        self.map = np.zeros(map_size)
        self.prev_x = 0
        self.prev_y = 0
        self.x = 0
        self.y = 0
        self.yaw = -np.pi/2
        self.init_origin = False
        # self.prev_time_imu = 0
        self.prev_msg = [0, 0, 0, 0]
        self.use_back = True
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('hover_planning')
        self.stop_start = 0
        self.arrive = False
        self.waypoint_index = 0
        self.threshold = 0.3
        self.waypoints = self.generate_waypoints(file_path)
        self.prev_time = 0
        rospy.Subscriber('/map', nav_msgs.msg.OccupancyGrid, self.get_map)
        rospy.Subscriber('/lanes', laser_processing.msg.Lanes, self.generate_robot_pose)
        self.robot_odom_publisher = rospy.Publisher('/robot_odom', nav_msgs.msg.Odometry, queue_size=10)
        self.target_odom_publisher = rospy.Publisher('/target_odom', nav_msgs.msg.Odometry, queue_size=10)


    def generate_waypoints(self, file_path):
        path = os.path.join(file_path, "data/waypoints.json")
        # print(path)
        return_list = []
        with open(path, "r") as f:
            infos = json.load(f)
            for info in infos:
                return_list.append(Waypoints(info["type"], info["x"], info["y"], info["vx"], info["vy"], info["yaw"]))
        return return_list
    
    def get_map(self, msg):
        self.map = np.array(msg.data).reshape(self.map.shape)
        self.map = np.where(self.map == -1, 50, self.map)
        print(self.map)
    
    def generate_robot_pose(self, msg):
        if self.prev_msg[0] == msg.distance_front and self.prev_msg[2] == msg.distance_left:
            return
        else:
            self.prev_msg[0] = msg.distance_front
            self.prev_msg[1] = msg.distance_back
            self.prev_msg[2] = msg.distance_left
            self.prev_msg[3] = msg.distance_right
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "laser"
        self.prev_x = self.x
        self.prev_y = self.y
        self.yaw = np.pi/2-msg.angle_left # np.pi-msg.angle_left

        # print(abs(msg.distance_back + msg.distance_front - 10), self.use_back)
        # consider the closer distance, smoothly change?
        if abs(msg.distance_back + msg.distance_front - 10) < 0.05:
            if msg.distance_back < msg.distance_front:
                self.use_back = True
            else:
                self.use_back = False
        elif abs(msg.distance_back + msg.distance_front - 10) < 0.2:
            # Maybe at the stopping point?
            if msg.distance_front < 3.0:
                self.use_back = False
        elif msg.distance_front > 5 and msg.distance_back < 3.5:
            self.use_back = True

        # watch out when one distance value is weird!
        if self.use_back:
            if msg.distance_back + msg.distance_front < 3.5 and msg.distance_front > 0.5:
                x_dist = msg.distance_back + 8
            else:
                x_dist = msg.distance_back
            if self.init_origin and abs(x_dist - self.x) > 0.7 and abs(self.x - (10 - msg.distance_front)) < 0.5:
                self.x = 10 - msg.distance_front
            else:
                self.x = x_dist
        else:
            x_dist = 10 - msg.distance_front
            if self.init_origin and abs(x_dist - self.x) > 0.7:
                if msg.distance_back + msg.distance_front < 3.5:
                    self.x = msg.distance_back + 8
                else:
                    self.x = msg.distance_back
            else:
                self.x = x_dist

        y_dist = -msg.distance_left
        if self.init_origin and abs(self.y - y_dist) > 0.5:
            # transform right distance to relative left
            if self.x > 7:
                self.y = -4.5 + msg.distance_right
            else:
                self.y = -2.5 + msg.distance_right
        else:
            self.y = y_dist
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0

        # Use yaw from the lidar data
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)
        if not self.init_origin:
            self.init_origin = True
        
        self.publish_msg()
    
    def publish_msg(self):
        # first implement robot pose publisher
        curr_time = rospy.Time.now()
        robot_msg = nav_msgs.msg.Odometry()
        robot_msg.header.stamp = curr_time
        robot_msg.header.frame_id = "map"
        robot_msg.child_frame_id = "robot"
        robot_msg.pose.pose.position.x = self.x
        robot_msg.pose.pose.position.y = self.y
        robot_msg.pose.pose.position.z = 0
        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw+np.pi/2)
        robot_msg.pose.pose.orientation.x = quat[0]
        robot_msg.pose.pose.orientation.y = quat[1]
        robot_msg.pose.pose.orientation.z = quat[2]
        robot_msg.pose.pose.orientation.w = quat[3]
        if self.prev_time != 0:
            dt = (curr_time - self.prev_time).to_sec()
            if dt != 0:
                robot_msg.twist.twist.linear.x = (self.x - self.prev_x) / dt
                robot_msg.twist.twist.linear.y = (self.y - self.prev_y) / dt
                robot_msg.twist.twist.linear.z = 0
        # maybe twist, too?
        self.robot_odom_publisher.publish(robot_msg)
        self.prev_time = curr_time

        # then implement pose checker
        # in the case of type 1, if the difference becomes smaller than threshold, load the next values
        waypoint = self.waypoints[self.waypoint_index]
        dist = sqrt((self.x - waypoint.x)**2+(self.y-waypoint.y)**2)
        if dist < self.threshold and waypoint.type == 0:
            self.waypoint_index += 1
        elif dist < 0.15 and waypoint.type == 1:
            if self.arrive == False:
                self.arrive = True
                self.stop_start = curr_time
            elif (curr_time - self.stop_start).to_sec() > 5:
                # need to stop
                self.waypoint_index += 1
                self.arrive = False
        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = len(self.waypoints) - 1
        
        ret_waypoint = self.waypoints[self.waypoint_index]
        # if waypoint.type == 0 and ret_waypoint.type == 1:
        #     self.stop_start = curr_time

        way_msg = nav_msgs.msg.Odometry()
        way_msg.header.stamp = curr_time
        way_msg.header.frame_id = "map"
        way_msg.child_frame_id = "robot"
        way_msg.pose.pose.position.x = ret_waypoint.x
        way_msg.pose.pose.position.y = ret_waypoint.y
        way_msg.pose.pose.position.z = 0
        quat = ret_waypoint.quat
        way_msg.pose.pose.orientation.x = quat[0]
        way_msg.pose.pose.orientation.y = quat[1]
        way_msg.pose.pose.orientation.z = quat[2]
        way_msg.pose.pose.orientation.w = quat[3]
        way_msg.twist.twist.linear.x = ret_waypoint.vx
        way_msg.twist.twist.linear.y = ret_waypoint.vy
        way_msg.twist.twist.linear.z = 0
        # maybe twist, too?
        self.target_odom_publisher.publish(way_msg)
        

if __name__ == '__main__':
    rospy.init_node('robot_tf2_broadcaster')
    robot = robotTF()
    rospy.spin()
    # rosbag play -l 2023-05-10-08-16-02.bag --clock