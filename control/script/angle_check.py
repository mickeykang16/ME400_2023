import rospy
from sensor_msgs.msg import Imu
from laser_line_extraction.msg import LineSegmentList, LineSegment
import tf

current_imu = [1, 0, 0, 0]
current_lidar = [1, 0, 0, 0]

def callback(data, rot):
    for line in data.line_segments:
        print(line.angle)
        print(rot)
        # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    

if __name__ == '__main__':
    rospy.init_node('angle')
    tf_listener = tf.TransformListener()
    r = rospy.Rate(1)
    get_trnsform = False
    # Takes some time due to the transform broadcast time, maybe able to change to static one
    while not get_trnsform:
        if tf_listener.canTransform(
                target_frame="imu", source_frame="laser", time=rospy.Time(0)):

            trans, rot = tf_listener.lookupTransform(
                target_frame="imu", source_frame="laser", time=rospy.Time(0))

            get_trnsform = True
        r.sleep()
    
    rospy.Subscriber("/line_segments", LineSegmentList, callback, rot)
    rospy.spin()
