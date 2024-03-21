#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class OdomListener:
    def __init__(self):
        rospy.init_node('odom_listener', anonymous=True)
        self.odom_subscriber = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.total_distance = 0.0
        self.last_pose = None

    def odom_callback(self, odom_msg):
        current_pose = odom_msg.pose.pose

        if self.last_pose is not None:
            delta_x = current_pose.position.x - self.last_pose.position.x
            delta_y = current_pose.position.y - self.last_pose.position.y
            delta_z = current_pose.position.z - self.last_pose.position.z

            distance = (delta_x**2 + delta_y**2 + delta_z**2)**0.5
            self.total_distance += distance

            rospy.loginfo('Distância total percorrida: %.2f metros', self.total_distance)

        self.last_pose = current_pose

if __name__ == '__main__':
    odom_listener = OdomListener()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
