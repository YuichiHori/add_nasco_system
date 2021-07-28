#! /usr/bin/env python3

node = "ms2830a"

import time
import sys
import pymeasure
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32

class ms2830a(object):
    def __init__(self):
        host = rospy.get_param("~host")
        gpibport = rospy.get_param("~gpibport")
        self.com = pymeasure.gpib_prologix(host, gpibport)
        self.com.open()


    def trace_data(self):
        self.com.send("TRAC? TRAC1")
        data_str = self.com.readline()
        #data_list = data_str.replace("\n","").split(",") #\nを削除してリストに
        return data_str


if __name__ == '__main__':
    rospy.init_node(node)
    spa = ms2830a()
    pub = rospy.Publisher('ms2830a', String, queue_size=1)

    while not rospy.is_shutdown():
        spec = spa.trace_data()
        pub.publish(spec)
        continue
