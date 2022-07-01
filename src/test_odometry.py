#!/usr/bin/python


#/usr/bin/python3


import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3




if __name__ == "__main__":

    rospy.init_node('odometry_publisher')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        

        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        delta_x = (vx * cos(theta) - vy * sin(theta)) * dt
        delta_y = (vx * sin(theta) + vy * cos(theta)) * dt
        delta_theta = vtheta * dt

        x += delta_x
        y += delta_y
        theta += delta_theta

        # since all odometry is 6DOF, we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set position
        odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
        
        # Set Velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vtheta))

        odom_pub.publish(odom)
        print(odom)

        last_time = current_time
        r.sleep()





