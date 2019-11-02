#!/usr/bin/env python
import rospy
import sys
import time
import math
import tf

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
0:             preset waypoint 0, keep still
1:             preset waypoint 1, relative dist: (1.0, 1.0, 0.5)
2:             preset waypoint 2, relative dist: (1.0, -1.0, 0.5)
s:             mpc control stops
c:             mpc control continues
q:             quit

please don't have caps lock on.
"""


class mpcTaskKeyboardInterface:
    def init(self):
        rospy.init_node('hydrus_mpc_task', anonymous=True)
        self.__settings = termios.tcgetattr(sys.stdin)
        print msg

        ## pub
        self.__mpc_target_odom_pub = rospy.Publisher('/mpc/target_odom', Odometry, queue_size=1)
        self.__mpc_target_nav_pub = rospy.Publisher('/uav/nav', FlightNav, queue_size=1)
        self.__mpc_stop_flag_pub = rospy.Publisher('/mpc/stop_cmd', Bool, queue_size=1)

        #sub
        self.__hydrus_odom = Odometry()
        self.__hydrus_cog_odom_sub = rospy.Subscriber('/uav/cog/odom', Odometry, self.__cogOdomCallback)
        time.sleep(0.5)
        rospy.Timer(rospy.Duration(0.01), self.__timerCallback)

    def __cogOdomCallback(self, msg):
        self.__hydrus_odom = msg

    def __sendMpcTargetOdom(self, pos_offset, period):
        mpc_target_odom = Odometry()
        mpc_target_odom.header.stamp = rospy.Time.now() + rospy.Duration(period)
        mpc_target_odom.pose.pose.position.x = self.__hydrus_odom.pose.pose.position.x + pos_offset[0]
        mpc_target_odom.pose.pose.position.y = self.__hydrus_odom.pose.pose.position.y + pos_offset[1]
        mpc_target_odom.pose.pose.position.z = self.__hydrus_odom.pose.pose.position.z + pos_offset[2]
        current_quaternion = (
            self.__hydrus_odom.pose.pose.orientation.x,
            self.__hydrus_odom.pose.pose.orientation.y,
            self.__hydrus_odom.pose.pose.orientation.z,
            self.__hydrus_odom.pose.pose.orientation.w)
        current_euler = tf.transformations.euler_from_quaternion(current_quaternion)
        roll = current_euler[0]
        pitch = current_euler[1]
        yaw = current_euler[2]

        target_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        mpc_target_odom.pose.pose.orientation.x = target_quaternion[0]
        mpc_target_odom.pose.pose.orientation.y = target_quaternion[1]
        mpc_target_odom.pose.pose.orientation.z = target_quaternion[2]
        mpc_target_odom.pose.pose.orientation.w = target_quaternion[3]

        self.__mpc_target_odom_pub.publish(mpc_target_odom)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__settings)
        return key

    def __timerCallback(self, event):
	key = self.getKey()
	print "the key value is %d" % ord(key)
	# takeoff and landing
	if key == '0':
            self.__sendMpcTargetOdom([0.0, 0.0, 0.0], 2.0)
	if key == '1':
            self.__sendMpcTargetOdom([1.0, 1.0, 0.5], 2.0)
	if key == '2':
            self.__sendMpcTargetOdom([1.0, -1.0, 0.5], 2.0)
	if key == 's':
            print "Mpc control stops"
            msg = Bool()
            msg.data = True
            self.__mpc_stop_flag_pub.publish(msg)
	if key == 'c':
            print "Mpc control continues"
            msg = Bool()
            msg.data = False
            self.__mpc_stop_flag_pub.publish(msg)
	if key == 'q':
            print "force quit by visit unexisted function"
	    error()

if __name__ == '__main__':
    try:
        mpc_task_interface = mpcTaskKeyboardInterface()
        mpc_task_interface.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

