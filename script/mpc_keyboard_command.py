#!/usr/bin/env python
import rospy
import sys
import time
import math

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
1:             preset waypoint 1, relative dist: (1.0, 1.0, 0.5)
2:             preset waypoint 2, relative dist: (1.0, -1.0, 0.5)
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

        #sub
        self.__hydrus_odom = Odometry()
        self.__hydrus_cog_odom_sub = rospy.Subscriber('/uav/cog/odom', Odometry, self.__cogOdomCallback)
        time.sleep(0.5)
        rospy.Timer(rospy.Duration(0.01), self.__timerCallback)

    def __cogOdomCallback(self, msg):
        self.__hydrus_odom = msg

    def __sendMpcTargetOdom(self, pos_offset, period):
        mpc_target_odom = Odometry()
        mpc_target_odom = self.__hydrus_odom
        mpc_target_odom.header.stamp = rospy.Time.now() + rospy.Duration(period)
        mpc_target_odom.pose.pose.position.x += pos_offset[0]
        mpc_target_odom.pose.pose.position.y += pos_offset[1]
        mpc_target_odom.pose.pose.position.z += pos_offset[2]
        self.__mpc_target_odom_pub.publish(mpc_target_odom)

    def __sendFlightNacCmd(self, pos_offset, period): ## todo: change to seperate file to subscribe reference state and publish nav cmd
        nav_msg = FlightNav()
        nav_msg.control_frame = nav_msg.WORLD_FRAME
        nav_msg.target = nav_msg.COG
        if axis == 0:
            nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE
            nav_msg.target_pos_x = self.__hydrus_odom.pose.pose.position.x + move_gap
            nav_msg.target_pos_y = self.__hydrus_odom.pose.pose.position.y
        elif axis == 1:
            nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE
            nav_msg.target_pos_x = self.__hydrus_odom.pose.pose.position.x
            nav_msg.target_pos_y = self.__hydrus_odom.pose.pose.position.y + move_gap
        elif axis == 2:
            nav_msg.pos_z_nav_mode = nav_msg.POS_MODE
            nav_msg.target_pos_z = self.__hydrus_odom.pose.pose.position.z + move_gap
        self.__hydrus_nav_cmd_pub.publish(nav_msg)

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
	if key == '1':
            self.__sendMpcTargetOdom([1.0, 1.0, 0.5], 2.0)
	if key == '2':
            self.__sendMpcTargetOdom([1.0, -1.0, 0.5], 2.0)
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

