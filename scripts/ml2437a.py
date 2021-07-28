#! /usr/bin/env python3

node = "ml2437a"

import time
import sys
import pymeasure
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32

class ml2437a(object):
    def __init__(self):
        host = rospy.get_param("~host")
        gpibport = rospy.get_param("~gpibport")
        com = pymeasure.gpib_prologix(host, gpibport)

        self.pm = pymeasure.Anritsu.ml2437a(com)

        #ave_onoff = rospy.get_param("~ave_onoff")
        #ave_num = rospy.get_param("~ave_num")
        #self.pm.set_average_onoff(ave_onoff)
        #self.pm.set_average_count(ave_num)
        com.send("AVG A, OFF,")


    def power(self,ch):
        power = self.pm.measure(ch)
        return power


if __name__ == '__main__':
    ch_num = 1
    rospy.init_node(node)
    pub = rospy.Publisher('ml2437a', Float64, queue_size=1)
    pm = ml2437a()

    while not rospy.is_shutdown():
        for ch in range(0,ch_num):
            power = pm.power(ch+1)
            pub.publish(power)
        continue
