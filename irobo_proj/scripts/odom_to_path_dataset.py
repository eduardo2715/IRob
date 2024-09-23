#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        rospy.init_node('odom_to_path', anonymous=True)

        # Publisher for the Path message
        self.path_pub = rospy.Publisher('/odom_path', Path, queue_size=10)

        # Initialize Path message
        self.path = Path()
        self.path.header.frame_id = "odom"

        # Subscriber to the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Create a new Path message each time
        self.path = Path()
        self.path.header.frame_id = "odom"

        # Create a PoseStamped from the odom message
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append pose to the path
        self.path.poses.append(pose)

        # Update the header with the current time
        self.path.header.stamp = rospy.Time.now()

        # Publish the path
        self.path_pub.publish(self.path)


if __name__ == '__main__':
    try:
        OdomToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

