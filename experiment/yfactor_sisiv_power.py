#!/usr/bin/env python3


name = 'yfactor_sisv_power_necstdb'


import sys
import rospy
import time
import std_msgs.msg
from std_msgs.msg import Int32
import datetime
import numpy

sys.path.append('/home/amigos/catkin_ws/src/nasco_system/scripts/')
import nasco_controller
import logger_controller
#import jpynb_controller


rospy.init_node(name)

con = nasco_controller.controller(node=False)
logger = logger_controller.logger()
#jpynb = jpynb_controller.jpynb()

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
dir_name = name + '/' + date + '.necstdb'
#dir_name_jpynb = name + '/' + date

# set params.
beam = '2l'
interval = 1
sis_param = {
    'initial_voltage' : 0,  # mV
    'final_voltage' : 15,     # mV
    'step' : 0.05              # mV
}
sis_arr = numpy.arange(sis_param['initial_voltage'], sis_param['final_voltage']+sis_param['step'], sis_param['step'])

loatt_cur =  7.8

slider_pub = rospy.Publisher('slider_location_cmd', Int32, queue_size=1)

# initialize.
print('[INFO] : Initializing... ')
con.sis.output_sis_voltage(beam, sis_param["initial_voltage"])
con.loatt.output_loatt_current(beam, loatt_cur)
slider_pub.publish(20000)
time.sleep(interval)

# measure sisiv.
print('[INFO] : Start to measure yfactor.')
logger.start(dir_name)

for vol in sis_arr:
    con.sis.output_sis_voltage(beam=beam, voltage=vol)
    slider_pub.publish(20000)
    time.sleep(interval)
    slider_pub.publish(8000)
    time.sleep(interval)

print('[INFO] : Finish measure yfactor')
logger.stop()

# finalize.
print('[INFO] : Finalizing... ')
con.sis.output_sis_voltage(beam, 0.)
con.loatt.output_loatt_current(beam, 20)
slider_pub.publish(20000)

time.sleep(1.)
