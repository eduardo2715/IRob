#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        rospy.init_node('odom_to_path', anonymous=True)

        # Publisher for the Path message
        self.path_pub = rospy.Publisher('/odom_path', Path, queue_size=10)

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "odom"

        # Subscriber to the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Create a PoseStamped message from the Odometry message
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append the PoseStamped to the path (keep previous poses)
        self.path.poses.append(pose)

        # Update the header timestamp
        self.path.header.stamp = rospy.Time.now()

        # Publish the accumulated path
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        OdomToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

