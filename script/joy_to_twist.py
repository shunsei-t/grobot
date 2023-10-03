#!/usr/bin/env python3
# coding: UTF-8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped

auto = False

class Joyctrl:
    def __init__(self):
        rospy.init_node('joy_to_twist_new')
        rospy.Subscriber('/joy', Joy, self.control)
        self.twist_pub = rospy.Publisher('/twist_joy', Twist, queue_size=1)
        rospy.spin()


    def control(self, msg):#joy to twist
        L_horizontal = msg.axes[6]  #左ジョイスティック（左右）
        L_vertical = msg.axes[7]    #左ジョイスティック（上下）
        a_z = L_horizontal * 0.6
        l_x = L_vertical * 0.1
        velocity = [a_z, l_x]

        t = Twist() #Twistのインスタンスを生成

        t.angular.z, t.linear.x = velocity #twistにjoyから取得したデータを当てはめる

        self.twist_pub.publish(t)    #twistを配信


if __name__ == '__main__':
    myjoy = Joyctrl()
