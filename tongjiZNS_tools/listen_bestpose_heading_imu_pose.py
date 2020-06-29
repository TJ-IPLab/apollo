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
#from modules.drivers.gnss.proto import gnss_pb2
from modules.drivers.gnss.proto import imu_pb2
from modules.drivers.gnss.proto import gnss_best_pose_pb2
from modules.drivers.gnss.proto import heading_pb2
from modules.localization.proto import localization_pb2
#from modules.planning.proto import planning_pb2

def callbackg(data):
    #print('GNSS')
    #local_es.CopyFrom(data)   
    time_of_pub = data.header.timestamp_sec
    posex=data.position.lon
    posey=data.position.lat
    posez=data.position.height
    #rospy.loginfo(time_of_pub-rospy.get_time(),posex,posey,posez)  #todo
    #rospy.loginfo(time_of_pub-rospy.get_time())
    #print(posex,posey,posez)
    f.write(str(time_of_pub)+str(posex)+';'+str(posey)+';'+str(posez)+'\n')
def callbackimu(data):
    #local_es.CopyFrom(data)
    #print('IMU received')   
    time_of_pub = data.header.timestamp_sec
    lvx=data.linear_acceleration.x
    lvy=data.linear_acceleration.y
    lvz=data.linear_acceleration.z
    avx=data.angular_velocity.x
    avy=data.angular_velocity.y
    avz=data.angular_velocity.z
    #rospy.loginfo(time_of_pub-rospy.get_time(),posex,posey,posez)  #todo
    #rospy.loginfo(time_of_pub-rospy.get_time())
    #print(avx,avy,avz)
    fimu.write(str(time_of_pub)+';'+str(lvx)+';'+str(lvy)+';'+str(lvz)+';'+str(avx)+';'+str(avy)+';'+str(avz)+';''\n')
def callbackbestpose(data):
    #local_es.CopyFrom(data) 
    #print('bestpose received')  
    time_of_pub = data.header.timestamp_sec
    la=data.latitude
    lo=data.longitude

    #rospy.loginfo(time_of_pub-rospy.get_time(),posex,posey,posez)  #todo
    #rospy.loginfo(time_of_pub-rospy.get_time())
    fbestpose.write(str(time_of_pub)+';'+str(la)+';'+str(lo)+';''\n')
def callbackheading(data):
    #local_es.CopyFrom(data) 
    #print('heading received')  
    time_of_pub = data.header.timestamp_sec
    he=data.heading
    fheading.write(str(time_of_pub)+';'+str(he)+';''\n')
def callbackpose(data):
    #local_es.CopyFrom(data) 
    #print('pose received')  
    time_of_pub = data.header.timestamp_sec
    posx=data.pose.position.x
    posy=data.pose.position.y
    posz=data.pose.position.z
    orix=data.pose.orientation.qx
    oriy=data.pose.orientation.qy
    oriz=data.pose.orientation.qz
    oriw=data.pose.orientation.qw
    fpose.write(str(time_of_pub)+';'+str(posx)+';'+str(posy)+';'+str(posz)+';'+str(orix)+';'+str(oriy)+';'+str(oriz)+';'+str(oriw)+';''\n')
def listener():
    rospy.init_node('listening', anonymous=True)
    #rospy.Subscriber("/apollo/localization/pose", localization_pb2.LocalizationEstimate, callback) 
    #rospy.Subscriber("/apollo/sensor/gnss/odometry", localization_pb2.Gnss, callbackg)
    rospy.Subscriber("/apollo/sensor/gnss/imu", imu_pb2.Imu, callbackimu)
    rospy.Subscriber("/apollo/sensor/gnss/best_pose", gnss_best_pose_pb2.GnssBestPose, callbackbestpose)
    rospy.Subscriber("/apollo/sensor/gnss/heading", heading_pb2.Heading, callbackheading)
    rospy.Subscriber("/apollo/localization/pose", localization_pb2.LocalizationEstimate, callbackpose)
    rospy.spin()
if __name__ == '__main__':
    #f=open('testg.txt','w')
    fimu=open('/apollo/tongjiZNS_data/data_tmp/gnssimu.txt','w')
    fbestpose=open('/apollo/tongjiZNS_data/data_tmp/gnssbestpose.txt','w')
    fheading=open('/apollo/tongjiZNS_data/data_tmp/gnssheading.txt','w')
    fpose=open('/apollo/tongjiZNS_data/data_tmp/localizationpose.txt','w')
    listener()
    #f.close() 
    fimu.close()
    fbestpose.close()
    fheading.close()
    fpose.close()
