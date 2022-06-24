#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import tf

class TestMovement():
    def __init__(self):

        rospy.init_node("test")
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rate = 20
        r = rospy.Rate(rate)
        self.tf_listener = tf.TransformListener()



        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")


    def update(self, x, y, z, theta, speed, turn_speed):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.speed = speed
        self.turn_speed = turn_speed

        self.condition.notify()
        self.condition.release()



    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
