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
#from modules.localization.proto import localization_pb2
from modules.drivers.gnss.proto import gnss_pb2
from modules.drivers.gnss.proto import imu_pb2
from modules.drivers.gnss.proto import gnss_best_pose_pb2
#from modules.planning.proto import planning_pb2

def callbackg(data):
    #local_es.CopyFrom(data)   
    time_of_pub = data.header.timestamp_sec
    posex=data.position.lon
    posey=data.position.lat
    posez=data.position.height
    #rospy.loginfo(time_of_pub-rospy.get_time(),posex,posey,posez)  #todo
    rospy.loginfo(time_of_pub-rospy.get_time())
    #print(posex,posey,posez)
    print('GNSS',posex,posey,posez)
    f.write(str(posex)+';'+str(posey)+';'+str(posez)+'\n')
def callbacki(data):
    #local_es.CopyFrom(data)   
    time_of_pub = data.header.timestamp_sec
    lvx=data.linear_acceleration.x
    lvy=data.linear_acceleration.y
    lvz=data.linear_acceleration.z
    avx=data.angular_velocity.x
    avy=data.angular_velocity.y
    avz=data.angular_velocity.z
    #rospy.loginfo(time_of_pub-rospy.get_time(),posex,posey,posez)  #todo
    rospy.loginfo(time_of_pub-rospy.get_time())
    #print(lvx,lvy,lvz)
    #print(avx,avy,avz)
    fi.write(str(lvx)+';'+str(lvy)+';'+str(lvz)+str(avx)+';'str(avy)+';'str(avz)+';''\n')
def callbackb(data):
    #local_es.CopyFrom(data)   
    time_of_pub = data.header.timestamp_sec
    la=data.latitude
    lo=data.longitude

    #rospy.loginfo(time_of_pub-rospy.get_time(),posex,posey,posez)  #todo
    rospy.loginfo(time_of_pub-rospy.get_time())
    #print(la,lo)
    print('bestpos:',la,lo)
    fb.write(str(la)+';'+str(lo)+';''\n')
def listener():
    rospy.init_node('zyh_is_listening', anonymous=True)
    #rospy.Subscriber("/apollo/localization/pose", localization_pb2.LocalizationEstimate, callback) 
    rospy.Subscriber("/apollo/sensor/gnss/odometry", gnss_pb2.Gnss, callbackg)
    rospy.Subscriber("/apollo/sensor/gnss/imu", imu_pb2.Imu, callbacki)
    rospy.Subscriber("/apollo/sensor/gnss/best_pos", gnss_best_pose_pb2.GnssBestPose, callbackb)
    rospy.spin()
if __name__ == '__main__':
    f=open('testg.txt','w')
    fi=open('testi.txt','w')
    fb=open('testb.txt','w')
    listener()
    f.close() 
    fi.close()
    fb.close()
