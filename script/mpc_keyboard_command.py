#!/usr/bin/env python
import rospy
import sys
import time
import math
import tf

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from aerial_robot_msgs.msg import MpcWaypointList

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
0:             preset waypoint 0, keep still
1:             preset waypoint 1, relative dist: (1.0, 1.0, 0.5)
2:             preset waypoint 2, relative dist: (1.0, -1.0, 0.5)
o:             circle motion
l:             circle speed increase
k:             circle speed decrease
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
        self.__mpc_target_waypoints_pub = rospy.Publisher('/mpc/target_waypoints', MpcWaypointList, queue_size=1)
        self.__mpc_target_nav_pub = rospy.Publisher('/uav/nav', FlightNav, queue_size=1)
        self.__mpc_stop_flag_pub = rospy.Publisher('/mpc/stop_cmd', Bool, queue_size=1)
        self.__mpc_target_traj_pub = rospy.Publisher("/mpc/circle_traj", Path, queue_size=1);

        #sub
        self.__hydrus_odom = Odometry()
        self.__hydrus_cog_odom_sub = rospy.Subscriber('/uav/cog/odom', Odometry, self.__cogOdomCallback)

        self.__circle_motion_flag = False
        self.__circle_mpc_mode = True ## otherwise: flight_nav pos_vel cmd mode

        ## simulation test
        self.__circle_radius = 5.0
        self.__ang_vel = 0.05 ## 0.3
        self.__ang_vel_change_unit = 0.05
        ## 326 test
        # self.__circle_radius = 1.2
        # self.__ang_vel = 0.2
        # self.__ang_vel_change_unit = 0.1

        time.sleep(0.5)
        rospy.Timer(rospy.Duration(0.01), self.__timerCallback)
        rospy.Timer(rospy.Duration(0.01), self.__circleMotionCallback)

    def __cogOdomCallback(self, msg):
        self.__hydrus_odom = msg

    def __sendMpcTargetOdom(self, pos_offset, period):
        mpc_waypoints = MpcWaypointList()
        mpc_waypoints.mode = mpc_waypoints.FULL
        mpc_waypoints.header.stamp = rospy.Time.now()

        mpc_waypoints.list.append(Odometry())
        mpc_waypoints.list[0].header.stamp = rospy.Time.now() + rospy.Duration(period)
        mpc_waypoints.list[0].pose.pose.position.x = self.__hydrus_odom.pose.pose.position.x + pos_offset[0]
        mpc_waypoints.list[0].pose.pose.position.y = self.__hydrus_odom.pose.pose.position.y + pos_offset[1]
        mpc_waypoints.list[0].pose.pose.position.z = self.__hydrus_odom.pose.pose.position.z + pos_offset[2]
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
        mpc_waypoints.list[0].pose.pose.orientation.x = target_quaternion[0]
        mpc_waypoints.list[0].pose.pose.orientation.y = target_quaternion[1]
        mpc_waypoints.list[0].pose.pose.orientation.z = target_quaternion[2]
        mpc_waypoints.list[0].pose.pose.orientation.w = target_quaternion[3]

        mpc_waypoints.list[0].twist.twist.linear.x = 0.0
        mpc_waypoints.list[0].twist.twist.linear.y = 0.0
        mpc_waypoints.list[0].twist.twist.linear.z = 0.0
        mpc_waypoints.list[0].twist.twist.angular.x = 0.0
        mpc_waypoints.list[0].twist.twist.angular.y = 0.0
        mpc_waypoints.list[0].twist.twist.angular.z = 0.0

        self.__mpc_target_waypoints_pub.publish(mpc_waypoints)

    def __sendCircleCommand(self):
        if self.__circle_mpc_mode:
            time_gap = 0.1
            candidate = 21
            mpc_waypoints = MpcWaypointList()
            mpc_waypoints.mode = mpc_waypoints.FULL
            mpc_waypoints.header.stamp = rospy.Time.now()

            for i in range(0, candidate):
                mpc_waypoints.list.append(Odometry())
                mpc_waypoints.list[i].header.stamp = mpc_waypoints.header.stamp + rospy.Duration(time_gap * i)
                relative_time = mpc_waypoints.list[i].header.stamp.to_sec() - self.__circle_start_time.to_sec()
                mpc_waypoints.list[i].pose.pose.position.x = self.__circle_start_odom.pose.pose.position.x - self.__circle_radius + self.__circle_radius * math.cos(relative_time * self.__ang_vel + self.__circle_start_ang)
                mpc_waypoints.list[i].pose.pose.position.y = self.__circle_start_odom.pose.pose.position.y + self.__circle_radius * math.sin(relative_time * self.__ang_vel + self.__circle_start_ang)
                mpc_waypoints.list[i].pose.pose.position.z = self.__circle_start_odom.pose.pose.position.z
                current_quaternion = (
                    self.__circle_start_odom.pose.pose.orientation.x,
                    self.__circle_start_odom.pose.pose.orientation.y,
                    self.__circle_start_odom.pose.pose.orientation.z,
                    self.__circle_start_odom.pose.pose.orientation.w)
                current_euler = tf.transformations.euler_from_quaternion(current_quaternion)
                roll = current_euler[0]
                pitch = current_euler[1]
                yaw = current_euler[2]

                target_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                mpc_waypoints.list[i].pose.pose.orientation.x = target_quaternion[0]
                mpc_waypoints.list[i].pose.pose.orientation.y = target_quaternion[1]
                mpc_waypoints.list[i].pose.pose.orientation.z = target_quaternion[2]
                mpc_waypoints.list[i].pose.pose.orientation.w = target_quaternion[3]

                mpc_waypoints.list[i].twist.twist.linear.x = -math.sin(relative_time * self.__ang_vel + self.__circle_start_ang) * self.__circle_radius * self.__ang_vel
                mpc_waypoints.list[i].twist.twist.linear.y = math.cos(relative_time * self.__ang_vel + self.__circle_start_ang) * self.__circle_radius * self.__ang_vel
                mpc_waypoints.list[i].twist.twist.linear.z = 0.0
                mpc_waypoints.list[i].twist.twist.angular.x = 0.0
                mpc_waypoints.list[i].twist.twist.angular.y = 0.0
                ## mpc_waypoints.list[i].twist.twist.angular.z = self.__ang_vel
                mpc_waypoints.list[i].twist.twist.angular.z = 0.0

            self.__mpc_target_waypoints_pub.publish(mpc_waypoints)
        else: ## direct flight_nav pos_vel cmd mode
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.control_frame = nav_msg.WORLD_FRAME
            nav_msg.target = nav_msg.COG
            nav_msg.pos_xy_nav_mode = nav_msg.POS_VEL_MODE
            relative_time = rospy.Time.now().to_sec() - self.__circle_start_time.to_sec()
            nav_msg.target_pos_x = self.__circle_start_odom.pose.pose.position.x - self.__circle_radius + self.__circle_radius * math.cos(relative_time * self.__ang_vel + self.__circle_start_ang)
            nav_msg.target_pos_y = self.__circle_start_odom.pose.pose.position.y + self.__circle_radius * math.sin(relative_time * self.__ang_vel + self.__circle_start_ang)
            nav_msg.target_vel_x = -math.sin(relative_time * self.__ang_vel + self.__circle_start_ang) * self.__circle_radius * self.__ang_vel
            nav_msg.target_vel_y = math.cos(relative_time * self.__ang_vel + self.__circle_start_ang) * self.__circle_radius * self.__ang_vel
            nav_msg.pos_z_nav_mode = nav_msg.POS_MODE
            nav_msg.target_pos_z = self.__circle_start_odom.pose.pose.position.z
            self.__mpc_target_nav_pub.publish(nav_msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__settings)
        return key

    def __circleMotionCallback(self, event):
        if self.__circle_motion_flag:
            self.__sendCircleCommand()

    def __publishMpcTargetCirclePath(self):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/world"
        num = 361
        ang_gap = 2.0 * 3.14159 / (num - 1)
        center = [self.__circle_start_odom.pose.pose.position.x - self.__circle_radius,
                  self.__circle_start_odom.pose.pose.position.y, self.__circle_start_odom.pose.pose.position.z]
        for i in range(0, num):
            msg.poses.append(PoseStamped())
            msg.poses[i].header = msg.header
            msg.poses[i].pose.position.x = center[0] + math.cos(ang_gap * i) * self.__circle_radius
            msg.poses[i].pose.position.y = center[1] + math.sin(ang_gap * i) * self.__circle_radius
            msg.poses[i].pose.position.z = center[2]
        self.__mpc_target_traj_pub.publish(msg)

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
	if key == 'o':
            self.__circle_start_time = rospy.Time.now()
            self.__circle_start_ang = 0.0
            self.__circle_start_odom = self.__hydrus_odom
            self.__circle_motion_flag = True
            self.__publishMpcTargetCirclePath()
            rospy.loginfo("Circle motion starts")
	elif key == 'k':
            self.__circle_start_ang = (rospy.Time.now().to_sec() - self.__circle_start_time.to_sec()) * self.__ang_vel + self.__circle_start_ang
            self.__ang_vel -= self.__ang_vel_change_unit
            self.__circle_start_time = rospy.Time.now()
            rospy.loginfo("Current vel decrease to: %f", self.__ang_vel * self.__circle_radius)
	elif key == 'l':
            self.__circle_start_ang = (rospy.Time.now().to_sec() - self.__circle_start_time.to_sec()) * self.__ang_vel + self.__circle_start_ang
            self.__ang_vel += self.__ang_vel_change_unit
            self.__circle_start_time = rospy.Time.now()
            rospy.loginfo("Current vel increase to: %f", self.__ang_vel * self.__circle_radius)
        else:
            self.__circle_motion_flag = False
            rospy.loginfo("Circle motion stops")
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

