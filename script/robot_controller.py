#!/usr/bin/env python3
# coding: UTF-8

import rospy
import dxl_moduler
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf_conversions
import math

class Controller:
    def __init__(self) -> None:
        rospy.init_node("robot_controller")

        rospy.Subscriber("cmd_vel", Twist, self.twist_callback)

        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=10)

        self.br = tf2_ros.TransformBroadcaster()

        dxl_port                 = rospy.get_param("~dxl_port", "/dev/ttyUSB0")
        self.dxl_l_id            = rospy.get_param("~dxl_l_id", 1)
        self.dxl_r_id            = rospy.get_param("~dxl_r_id", 2)
        self.tread_merter        = rospy.get_param("~tread", 0.16)
        self.wheel_radius_merter = rospy.get_param("~wheel_radius_merter", 0.06)
        self.odom_header_frame   = rospy.get_param("~odom_header_frame", "odom")
        self.odom_child_frame    = rospy.get_param("~odom_child_frame", "base_footprint")

        self.moduler = dxl_moduler.DxlClass(dxl_port)
        self.moduler.write_operateing_mode(self.dxl_l_id, mode=1)
        self.moduler.write_operateing_mode(self.dxl_r_id, mode=1)

        self.moduler.write_torque_state(self.dxl_l_id, state=1)
        self.moduler.write_torque_state(self.dxl_r_id, state=1)

        self.previous_time = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_theta = 0.0
        self.v_ld = 0
        self.v_rd = 0

        self.DXL_RPM2MperS_RATIO = 0.229 * 2.0*math.pi/60.0 * self.wheel_radius_merter

    def twist_callback(self, data):
        v = data.linear.x
        w = data.angular.z

        v_l = v - self.tread_merter/2.0*w
        v_r = v + self.tread_merter/2.0*w

        self.v_ld = int(v_l/self.DXL_RPM2MperS_RATIO)
        self.v_rd = int(v_r/self.DXL_RPM2MperS_RATIO)

        if self.v_ld >= 1023:
            self.v_ld = 1023
        elif self.v_ld <= -1023:
            self.v_ld = -1023

        if self.v_rd >= 1023:
            self.v_rd = 1023
        elif self.v_rd <= -1023:
            self.v_rd = -1023

    def main(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.previous_time == 0.0:
                dt = 0.0
                self.previous_time = rospy.Time.now().to_sec()
            else:
                dt = rospy.Time.now().to_sec() - self.previous_time
                self.previous_time = rospy.Time.now().to_sec()

            # callbackで割り込むと、readの方と干渉する
            self.moduler.write_goal_velocity(self.dxl_l_id, self.v_ld)
            self.moduler.write_goal_velocity(self.dxl_r_id, -self.v_rd)

            v_ld = self.moduler.read_present_velocity(self.dxl_l_id)
            v_rd = -self.moduler.read_present_velocity(self.dxl_r_id)
            v_l = v_ld*self.DXL_RPM2MperS_RATIO
            v_r = v_rd*self.DXL_RPM2MperS_RATIO

            #参考 https://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html
            v = (v_l + v_r)/2
            w = (v_r - v_l)/self.tread_merter

            theta = self.previous_theta + w*dt

            x_dot = v*math.cos(theta)
            y_dot = v*math.sin(theta)

            # x = self.previous_x + x_dot*dt*math.cos(theta + theta_dot*dt/2)
            # y = self.previous_y + y_dot*dt*math.sin(theta + theta_dot*dt/2)

            x = self.previous_x + x_dot*dt
            y = self.previous_y + y_dot*dt

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)

            # odom ================================================================
            odom_msg = Odometry()
            odom_msg.header.frame_id = self.odom_header_frame
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.child_frame_id = self.odom_child_frame
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w

            self.pub_odom.publish(odom_msg)
            # odom ================================================================

            # tf ==================================================================
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.odom_header_frame
            t.child_frame_id = self.odom_child_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.br.sendTransform(t)
            # tf ==================================================================


            self.previous_theta = theta
            self.previous_x = x
            self.previous_y = y

            r.sleep()

if __name__ == "__main__":
    mycon = Controller()
    mycon.main()