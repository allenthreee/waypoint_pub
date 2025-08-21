#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3
import tf.transformations

class OdomToMavros:
    def __init__(self):
        rospy.init_node('odom_to_mavros', anonymous=True)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_cb)
        
        # Publishers
        self.vision_pose_pub = rospy.Publisher('/uav0/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.vision_speed_pub = rospy.Publisher('/uav0/mavros/vision_speed/speed_vector', Vector3, queue_size=10)
        
        # Variables
        self.last_time = rospy.Time.now()
        
        rospy.loginfo("Odom to MAVROS node initialized")
        
    def odom_cb(self, msg):
        # Current time
        current_time = rospy.Time.now()
        
        # Convert Odometry to PoseStamped
        vision_pose = PoseStamped()
        vision_pose.header.stamp = current_time
        vision_pose.header.frame_id = "world"
        vision_pose.pose = msg.pose.pose
        
        # Convert Odometry twist to speed vector
        speed_vector = Vector3()
        speed_vector.x = msg.twist.twist.linear.x
        speed_vector.y = msg.twist.twist.linear.y
        speed_vector.z = msg.twist.twist.linear.z
        
        # Publish the messages
        self.vision_pose_pub.publish(vision_pose)
        self.vision_speed_pub.publish(speed_vector)
        
        # Calculate yaw from quaternion
        quat = (
            vision_pose.pose.orientation.x,
            vision_pose.pose.orientation.y,
            vision_pose.pose.orientation.z,
            vision_pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = math.degrees(euler[2])  # Convert to degrees
        
        # Print pose and yaw with 2 decimal points
        # rospy.loginfo_throttle(1.0,  # Print at 1Hz to avoid flooding the console
        #     "Position: x={:.2f}, y={:.2f}, z={:.2f} | Yaw: {:.2f}°".format(
        #         vision_pose.pose.position.x,
        #         vision_pose.pose.position.y,
        #         vision_pose.pose.position.z,
        #         yaw
        #     )
        # )
        print("Position: x={:.2f}, y={:.2f}, z={:.2f} | Yaw: {:.2f}°".format(
            vision_pose.pose.position.x,
            vision_pose.pose.position.y,
            vision_pose.pose.position.z,
            yaw
        ))
        
        self.last_time = current_time

if __name__ == '__main__':
    try:
        converter = OdomToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
