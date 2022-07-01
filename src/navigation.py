#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import threading
import sys, select

TwistMsg = Twist


class NavigationThread(threading.Thread):
    def __init__(self, rate):
        super(NavigationThread, self).__init__()
        self.publisher = rospy.Publisher('/cmd_vel', TwistMsg, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.speed = 0.0
        self.turn_speed = 0.0
        self.done = False


        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    
    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i %= 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
    
    
    def update(self, x, y, z, theta, speed, turn_speed):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.speed = speed
        self.turn_speed = turn_speed

    
    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)

    

    def run(self):
        twist = Twist()

        # if stamped:
        #     twist =  twist_msg.twist
        #     twist_msg.header.stamp = rospy.Time.now()
        #     twist_msg.header.frame_id = twist_frame
        # else:
        #     twist = twist_msg
        
        while not self.done:
            # if stamped:
            #     twist_msg.header.stamp = rospy.Time.now()
            
            # Wait for a new message or timeout

            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.theta * self.turn_speed

            
            self.publisher.publish(twist)
            self.update()


        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

# TODO: This is where the navigation logic will go
def navigate():
    return 1, 0, 0, 0, 1, 0


        

if __name__ == '__main__':

    rospy.init_node('navigation')

    stamped = rospy.get_param("~stamped", False)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    twist_frame = rospy.get_param("~frame_id", '')
    repeat = rospy.get_param('~repeat_rate', 0.0)

    if stamped:
        TwistMsg = TwistStamped
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = NavigationThread(repeat)


    x, y, z, theta, speed, turn_speed = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 

    try:
        pub_thread.wait_for_subscribers()

        while True:
            x, y, z, theta, speed, turn_speed = navigate()
            pub_thread.update(x, y, z, theta, speed, turn_speed)
            pub_thread.run()    

    
    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()    
