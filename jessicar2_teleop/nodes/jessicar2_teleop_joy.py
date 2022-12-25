#!/usr/bin/env python

import sys
import rospy
import math
from time import sleep

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

MAXCOLOR = 8

msg = """
Control Your Robot!
---------------------------
'A' : Change RGB led Rainbow color
RED = 0,        // red
GREEN,          // green
BLUE,           // blue
YELLOW,         // yellow
PURPLE,         // purple
CYAN,           // cyan
WHITE,          // white
ALL_OFF         // off(black)

CTRL-C to quit
"""

class TeleopJoyNode:
    def __init__(self):
        self.timer = 0
        self.auto_mode = False
        self.headlight_on = False
        self.colorIdx = 0
        self.max_fwd_vel = rospy.get_param("~max_fwd_vel") #jessicar2_MAX_LIN_VEL = 1.20
        self.max_rev_vel = rospy.get_param("~max_rev_vel")
        self.max_ang_vel = rospy.get_param("~max_ang_vel") #jessicar2_MAX_ANG_VEL = 1.80
        rospy.Subscriber("/joy",  Joy, self.cb_joy, queue_size=1)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.pub_led = rospy.Publisher('rgbled', Int16, queue_size = 4)
        
        print(msg)

        rospy.Timer(rospy.Duration(0.05), self.timer_update)
        self.twist = Twist()

    def cb_joy(self, joymsg):
        if self.auto_mode == False:
            if joymsg.buttons[2] == 1:
                self.auto_mode = True
                rospy.loginfo("AUTO MODE ON")
        else:
            if joymsg.buttons[2] == 1:
                self.auto_mode = False
                rospy.loginfo("AUTO MODE OFF")
        #button 1 press, then acccept joy stick

        if joymsg.axes[1] > 0.0:
            self.twist.linear.x = joymsg.axes[1] * self.max_fwd_vel
        elif joymsg.axes[1] < 0.0:
            self.twist.linear.x = joymsg.axes[1] * self.max_rev_vel
        else:
            self.twist.linear.x = 0.0

        self.twist.angular.z = joymsg.axes[0] * self.max_ang_vel
        print(self.twist.linear.x, self.twist.angular.z)

        #button X press, then stop motor
        if joymsg.buttons[3] == 1: 
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0 
            print("Emergency stop")
            
        #button A press
        if joymsg.buttons[0] == 1:           
            print("Color Enum:", self.colorIdx)
            self.pub_led.publish(self.colorIdx)
            self.colorIdx += 1

            if self.colorIdx == MAXCOLOR:
                self.colorIdx = 0

        self.twist.linear.y = 0.0 
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        
    def timer_update(self, event):
        self.timer+=1
        if self.auto_mode == False:
            self.pub_twist.publish(self.twist)

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    try :
        rospy.init_node('teleop_joy_node')
        node = TeleopJoyNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
