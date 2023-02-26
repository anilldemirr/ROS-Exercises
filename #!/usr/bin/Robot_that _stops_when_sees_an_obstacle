#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LazerVerisi():
    def __init__(self):
        rospy.init_node("Lazer_Dugumu")
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.hiz_mesaji = Twist()
        rospy.Subscriber("scan",LaserScan,self.LazerCallback)
        rospy.spin()

    def LazerCallback(self,mesaj):
        sol_on = list(mesaj.ranges[0:9])
        sag_on = list(mesaj.ranges[350:359])
        on = sol_on + sag_on
        min_on = min(on)
        print(min_on)
        if min_on < 0.5:
            self.hiz_mesaji.linear.x = 0.0
            self.pub.publish(self.hiz_mesaji)
        else:
            self.hiz_mesaji.linear.x = 0.30
            self.pub.publish(self.hiz_mesaji)

object = LazerVerisi()
