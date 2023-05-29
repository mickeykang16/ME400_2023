#!/usr/bin/env python
import rospy, rospkg
import tf_conversions, tf2_ros
import laser_processing.msg
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import numpy as np
import os, json
from math import sqrt

# TODOS: generate velocity based on the waypoint
# decide when to set velocity to 0 when the next or current waypoint type is 1
# max vel 0.4, max acc 0.5 -> parameter

class Waypoints():
    def __init__(self, pt_type=0, x=0, y=0, yaw=0, vel_scale=1):
        self.type = pt_type
        self.x = x
        self.y = y
        self.quat = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
        self.vel_scale = vel_scale
    def set_velocity(self, vx, vy):
        self.vx = vx * self.vel_scale
        self.vy = vy * self.vel_scale
    

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
        self.prev_vx = 0
        self.prev_vy = 0
        self.yaw = -np.pi/2
        self.init_origin = False
        # self.prev_time_imu = 0
        # print(rospy.get_param_names())
        self.interpolate_distance = float(rospy.get_param("/transform/interpolate_distance"))
        self.mean_v = float(rospy.get_param("/transform/mean_v"))
        self.max_a = float(rospy.get_param("/transform/max_a"))
        self.prev_msg = [0, 0, 0, 0]
        self.use_back = True
        self.prev_back = True
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('hover_planning')
        self.stop_start = 0
        self.arrive = False
        self.waypoint_index = 0
        self.waypoints, self.stopping_waypoints = self.generate_waypoints(file_path)
        self.prev_time = 0
        rospy.Subscriber('/map', nav_msgs.msg.OccupancyGrid, self.get_map)
        rospy.Subscriber('/lanes', laser_processing.msg.Lanes, self.generate_robot_pose)
        self.robot_odom_publisher = rospy.Publisher('/robot_odom', nav_msgs.msg.Odometry, queue_size=10)
        self.target_odom_publisher = rospy.Publisher('/target_odom', nav_msgs.msg.Odometry, queue_size=10)
        self.wall_debug_publisher = rospy.Publisher('/wall_use_back', std_msgs.msg.Bool, queue_size = 10)

    def generate_waypoints(self, file_path):
        path = os.path.join(file_path, "data/waypoints.json")
        # print(path)
        return_list = []
        stop_list = []

        with open(path, "r") as f:
            infos = json.load(f)
            for info in infos:
                wp = Waypoints(info["type"], info["x"], info["y"], info["yaw"], info["vel_scale"])
                if len(return_list) == 0:
                    # the first point, assume go straight
                    wp.set_velocity(vx=self.mean_v, vy=0)
                else:
                    prev_wp = return_list[-1]
                    x_diff = wp.x - prev_wp.x
                    y_diff = wp.y - prev_wp.y
                    dist = sqrt(x_diff**2 + y_diff**2)
                    ratio = self.mean_v / dist
                    vx = ratio * x_diff
                    vy = ratio * y_diff
                    wp.set_velocity(vx, vy)
                return_list.append(wp)
                if wp.type >= 1:
                    stop_list.append(wp)
        # generate vx and vy commponent and the stoppint waypoint
        return return_list, stop_list
    
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
        self.prev_back = self.use_back
        self.yaw = np.pi/2-msg.angle_left # np.pi-msg.angle_left

        # print(abs(msg.distance_back + msg.distance_front - 10), self.use_back)
        # consider the closer distance, smoothly change?
        if abs(msg.distance_back + msg.distance_front - 10) < 0.05:
            if msg.distance_back < msg.distance_front:
                self.use_back = True
            else:
                self.use_back = False
        elif msg.distance_front < 6.0 and msg.distance_back > 3.5:
            self.use_back = False
        # elif abs(msg.distance_back + msg.distance_front - 10) < 0.5 or abs(msg.distance_back + msg.distance_front - 2) < 0.5:
        #     # Maybe at the stopping point?
        #     if msg.distance_front < 5.5:
        #         self.use_back = False
        #     else:
        #         self.use_back = True
        elif msg.distance_front > 5.5 and msg.distance_back < 4.0:
            self.use_back = True

        # watch out when one distance value is weird!
        if self.use_back:
            if msg.distance_back + msg.distance_front < 3.5 and msg.distance_front > 0.1:
                x_dist = msg.distance_back + 8
            else:
                x_dist = msg.distance_back
            if self.init_origin and abs(x_dist - self.x) > 0.7 and abs(self.x - (10 - msg.distance_front)) < 0.5:
                self.x = 10 - msg.distance_front
                self.use_back = False
            else:
                self.x = x_dist
        else:
            x_dist = 10 - msg.distance_front
            if self.init_origin and abs(x_dist - self.x) > 1.0:
                self.use_back = True
                if msg.distance_back + msg.distance_front < 3.5:
                    self.x = msg.distance_back + 8
                else:
                    self.x = msg.distance_back
            else:
                self.x = x_dist
        
        # if self.prev_back != self.use_back:
        #     print(self.prev_x, self.x)

        y_dist = -msg.distance_left
        self.y = y_dist
        if (msg.distance_front + msg.distance_back) < 4:
            self.y = -4.5+msg.distance_right
        # if self.init_origin and abs(self.y - y_dist) > 1.2:
        #     # transform right distance to relative left
        #     if self.x > 7:
        #         self.y = -4.5 + msg.distance_right
        #     else:
        #         self.y = -2.5 + msg.distance_right
        # else:
        #     self.y = y_dist
        
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
        
        self.publish_msg(msg.header.stamp)
    
    def __interpolate_point__(self, waypoint, dist):
        # use self.x, self.y and waypoint to get the middle point
        # get the distance fraction and calcuate the ration, multiply to the x and y diff
        ratio = self.interpolate_distance / dist # need to consider direction
        if ratio < 1:
            ret_x = self.x + (waypoint.x - self.x) * ratio
            ret_y = self.y + (waypoint.y - self.y) * ratio
        else:
            ret_x = waypoint.x
            ret_y = waypoint.y
        return ret_x, ret_y

    def possible_to_stop(self, dist, vel):
        # using 2as = v^2 - v0^2
        required_dist = vel**2 / (2*self.max_a)
        if required_dist < dist:
            return True
        else:
            return False
        # return (vel**2 / (2*self.max_a)) > dist
    
    def publish_msg(self, ref_stamp):
        # first implement robot pose publisher
        curr_time = ref_stamp
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
        vx, vy = 0, 0
        if self.prev_time != 0:
            dt = (curr_time - self.prev_time).to_sec()
            if dt != 0:
                # vx = (self.x - self.prev_x) / dt
                # if abs(vx - self.prev_vx) > 0.5:
                #     vx = self.prev_vx
                # vy = (self.y - self.prev_y) / dt
                # if abs(vy - self.prev_vy) > 0.5:
                #     vy = self.prev_vy
                vx = (self.x - self.prev_x) / dt
                vy = (self.y - self.prev_y) / dt
                robot_msg.twist.twist.linear.x = vx
                robot_msg.twist.twist.linear.y = vy
                robot_msg.twist.twist.linear.z = 0
                # self.prev_vx = vx
                # self.prev_vy = vy
        # maybe twist, too?
        self.robot_odom_publisher.publish(robot_msg)
        self.prev_time = curr_time

        # then implement pose checker
        # in the case of type 1, if the difference becomes smaller than threshold, load the next values
        waypoint = self.waypoints[self.waypoint_index]
        if self.waypoint_index < len(self.waypoints) - 1:
            nxt_waypoint = self.waypoints[self.waypoint_index + 1]
        else:
            nxt_waypoint = waypoint
        dist = sqrt((self.x - waypoint.x)**2+(self.y-waypoint.y)**2)
        dist_nxt = sqrt((self.x - nxt_waypoint.x)**2+(self.y-nxt_waypoint.y)**2)

        # in the case of self.use_back != self.prev_back => make sure it does not go backward weirdly
        # check the difference in self.prev_x and self.x, too
        # HERE
        # if self.use_back != self.prev_back and self.prev_x > self.x:
        # do not go backward, read the waypoint with further distance

        # if it is close enough to the current waypoint or the next way point is closser
        if (dist < self.interpolate_distance or dist > dist_nxt) and waypoint.type == 0:
            self.waypoint_index += 1
        elif dist < 0.2 and waypoint.type >= 1:
            if self.arrive == False:
                self.arrive = True
                self.stop_start = curr_time
            elif (waypoint.type == 1 and (curr_time - self.stop_start).to_sec() > 4) or (waypoint.type == 2 and (curr_time - self.stop_start).to_sec() > 0.5):
                # need to stop
                self.waypoint_index += 1
                self.arrive = False
                # pass the current type 1 point -> delete it from the self.stop_list
                # self.stopping_waypoints.pop(0)
          
        # for better time check, allow more error
        if waypoint.type >= 1 and self.arrive and dist > 0.25:
            self.arrive = False

        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = len(self.waypoints) - 1
        
        # need to check whether the ret_waypoint dist > self.interpolate_dist, unless type 1
        while True:
            ret_waypoint = self.waypoints[self.waypoint_index]
            ret_dist = sqrt((self.x - ret_waypoint.x)**2+(self.y-ret_waypoint.y)**2)
            if self.waypoint_index == len(self.waypoints) - 1:
                way_x, way_y = self.__interpolate_point__(ret_waypoint, ret_dist)
                break
            elif ret_waypoint.type >= 1 or ret_dist >= self.interpolate_distance:
                way_x, way_y = self.__interpolate_point__(ret_waypoint, ret_dist)
                break
            else:
                self.waypoint_index += 1
        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = len(self.waypoints) - 1
        
        # nearest_stop_wp = self.stopping_waypoints[0]
        # target_vx = ret_waypoint.vx
        # target_vy = ret_waypoint.vy
        # currently, directly change the waypoint velocity
        # may need to set it as only current vx (but that can make the control noisy)
        if ret_waypoint.type >= 1:
            if (not self.possible_to_stop(abs(self.x - ret_waypoint.x), vx)) and (abs(self.x - ret_waypoint.x) < 1):
                ret_waypoint.vx = 0
            if (not self.possible_to_stop(abs(self.y - ret_waypoint.y), vy)) and (abs(self.y - ret_waypoint.y) < 1):
                ret_waypoint.vy = 0
        
        way_msg = nav_msgs.msg.Odometry()
        way_msg.header.stamp = curr_time
        way_msg.header.frame_id = "map"
        way_msg.child_frame_id = "robot"
        way_msg.pose.pose.position.x = way_x
        way_msg.pose.pose.position.y = way_y
        if ret_waypoint.vx == 0:
            way_msg.pose.pose.position.z = -1
        else:
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
        
        bool_msg = std_msgs.msg.Bool()
        bool_msg.data = self.use_back
        self.wall_debug_publisher.publish(bool_msg)
        

if __name__ == '__main__':
    rospy.init_node('robot_tf2_broadcaster')
    robot = robotTF()
    rospy.spin()
    # rosbag play -l 2023-05-10-08-16-02.bag --clock
