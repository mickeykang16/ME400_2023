#!/usr/bin/env python

# get the core waypoints in the simulation
# interpolate waypoints

# can use rosbag --pause -> subscribe and store at the time???????
# just estimate the stopping point, subscribe the odometry point

import rospy, rospkg
import nav_msgs.msg
import tf
import json, os
from math import sqrt
import numpy as np
save_points = []

def save_json():
    # save it into the json, need to modify them little bit
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('hover_planning')
    path = os.path.join(file_path, "data/waypoints.json")
    return_list = []
    # get the start point, removing the first before starting part
    start_time = 0
    start_val = save_points[0][-1]
    for i in range(1, len(save_points)):
        cur_val = save_points[i][-1]
        if abs(cur_val - start_val) > 0.1:
            start_time = i-1
            break
    step_size = 5 * 2
    prev_save_time = start_time
    index = start_time
    np_save_points = np.array(save_points)
    np_save_points[1:, -1] =  np_save_points[0:-1,-1] - np_save_points[1:,-1]
    while index < len(save_points):
        temp_list = np_save_points[index:index+step_size,-1]
        if abs(sum(temp_list) / len(temp_list) - temp_list[0]) < 0.0006:
            # stopping point
            if len(return_list) == 0 or return_list[-1]["type"] != 1:
                return_dict = {"type": 1, "x": save_points[index][0], "y": save_points[index][1],\
                                "yaw": save_points[index][2]}
                return_list.append(return_dict)
            prev_save_time = index
        # elif temp_list:
        #     # save for the corner points
        #     # corner: when the change in
        #     pass
        elif index == prev_save_time + step_size:
            return_dict = {"type": 0, "x": save_points[index][0], "y": save_points[index][1],\
                            "yaw": save_points[index][2]}
            return_list.append(return_dict)
            prev_save_time = index
        index += 1

    with open(path, "w") as f:
        json.dump(return_list, f)

def filter_odom(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    rpy = tf.transformations.euler_from_quaternion(quat)
    ret = [x, y, rpy[2], sqrt(x**2+y**2)]
    save_points.append(ret)


if __name__ == "__main__":
    print("GEN")
    rospy.init_node("waypoint_saver")
    rospy.Subscriber('/robot_odom', nav_msgs.msg.Odometry, filter_odom)
    rospy.spin()
    rospy.on_shutdown(save_json)
