#!/usr/bin/env python

import argparse
import atexit
import logging
import os
import sys

import rospy
import scipy.signal as signal
#from logger import Logger
from numpy import genfromtxt

#from modules.canbus.proto import chassis_pb2
#from modules.common.proto import pnc_point_pb2
#from modules.common.proto import drive_state_pb2
#from modules.control.proto import pad_msg_pb2
from modules.localization.proto import localization_pb2
#from modules.planning.proto import planning_pb2

def callback(data):
    #local_es.CopyFrom(data)   
    time_of_pub = data.header.timestamp_sec
    posex=data.pose.position.x
    posey=data.pose.position.y
    posez=data.pose.position.z
    #rospy.loginfo(time_of_pub-rospy.get_time(),posex,posey,posez)  #todo
    rospy.loginfo(time_of_pub-rospy.get_time())
    print(posex,posey,posez)
    f.write(str(posex)+';'+str(posey)+';'+str(posez)+'\n')
def listener():
    rospy.init_node('zyh_is_listening', anonymous=True)
    rospy.Subscriber("/apollo/localization/pose", localization_pb2.LocalizationEstimate, callback)    
    rospy.spin()
if __name__ == '__main__':
    f=open('test.txt','w')
    listener()
    f.close()
