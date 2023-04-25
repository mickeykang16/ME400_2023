import rospy
from sensor_msgs.msg import Imu, LaserScan

def callback(data, pub):
    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = 'imu'
    imu.orientation = data.orientation
    imu.orientation_covariance = data.orientation_covariance
    imu.angular_velocity = data.angular_velocity
    imu.angular_velocity_covariance = data.angular_velocity_covariance
    imu.linear_acceleration = data.linear_acceleration
    imu.linear_acceleration_covariance = data.linear_acceleration_covariance
    pub.publish(imu)

def callback2(data, pub):
    scan = LaserScan()
    scan = data
    scan.header.stamp = rospy.Time.now()
    pub.publish(scan)

if __name__ == '__main__':
    rospy.init_node('label')
    pub = rospy.Publisher('/imunew', Imu, queue_size=10)
    rospy.Subscriber("/imu9250", Imu, callback, pub)
    pub2 = rospy.Publisher('/scannew', LaserScan, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, callback2, pub2)
    rospy.spin()