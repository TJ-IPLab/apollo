#-- coding:UTF-8 --
#!/usr/bin/env python

"""

Record GPS and IMU data

"""



import atexit

import logging

import math

import os

import sys  #sys是Python的一个「标准库」，也就是官方出的「模块」，是「System」的简写，封装了一些系统的信息和接口。

import time
import numpy as np
np.set_printoptions(threshold=np.inf)



from cyber_py import cyber

from gflags import FLAGS



from common.logger import Logger   #利用from imort就可以使用其他py文件中的函数，从common.logger.py中的Logger()函数。

from modules.canbus.proto import chassis_pb2  

from modules.localization.proto import localization_pb2  

from modules.drivers.gnss.proto import imu_pb2  #zhujiqi

from modules.drivers.gnss.proto import gnss_best_pose_pb2 #zhujiaqi

from modules.drivers.gnss.proto import heading_pb2  #zhujiaqi

from modules.drivers.proto import pointcloud_pb2 #shenxiangxiang

count1=0;
count2=0;


time_of_pub_last=0


def callbacklocalization(data):
    time_of_pub = data.header.timestamp_sec
    posx=data.pose.position.x #以原点向东，以m为单位
    posy=data.pose.position.y #以原点向北，以m为单位
    posz=data.pose.position.z #天，m
    orix=data.pose.orientation.qx #四元数，右前上到东北天坐标系的姿态角
    oriy=data.pose.orientation.qy
    oriz=data.pose.orientation.qz
    oriw=data.pose.orientation.qw
    fpose.write(str(time_of_pub)+';'+str(posx)+';'+str(posy)+';'+str(posz)+';'+str(orix)+';'+str(oriy)+';'+str(oriz)+';'+str(oriw)+';''\n')

def callbackbestpose(data):
    #local_es.CopyFrom(data) 
    #print('bestpose received')  
    time_of_pub = data.header.timestamp_sec
    sol_status=data.sol_status
    sol_type=data.sol_type
    lati=data.latitude #纬度，单位度
    longi=data.longitude #经度，单位度
    height=data.height_msl+data.undulation #高度，单位m
    lati_RMS=data.latitude_std_dev #纬度RMS单位m
    longi_RMS=data.longitude_std_dev #经度RMS单位m
    height_RMS=data.height_std_dev #高度RMS单位m
    num_sats_in_solution=data.num_sats_in_solution #解算卫星个数
    time_measurement=data.measurement_time  #测量时间
    global count1
    count1=count1+1
    fbestpose.write(str(time_of_pub)+';'+str(time_measurement)+';'+str(sol_status)+';'+str(sol_type)+';'+str(lati)+';'+str(longi)+';'+str(height)+';'+str(lati_RMS)+';'+str(longi_RMS)+';'+str(height_RMS)+';'+str(num_sats_in_solution)+';'+str(count1)+';''\n')


def callbackheading(data):
    #local_es.CopyFrom(data)
    #print('heading received')
    time_of_pub = data.header.timestamp_sec
    he=data.heading
    he_RMS=data.heading_std_dev
    time_measurement=data.measurement_time  #测量时间
    fheading.write(str(time_of_pub)+';'+str(time_measurement)+';'+str(he)+';'+str(he_RMS)+';''\n')

def callbackimu(data):
    #local_es.CopyFrom(data) 
    #print('bestpose received')
    time_of_pub=data.header.timestamp_sec
    # lvx=data.linear_acceleration.x #前左上 m/s^2，是原始的消息，经过drivers发出来，就是右前上的车身坐标系了，所以可直接用
    # lvy=data.linear_acceleration.y
    # lvz=data.linear_acceleration.z
    # avx=data.angular_velocity.x #前左上，rad/s是原始的消息，经过drivers发出来，就是右前上的车身坐标系了，所以可直接用
    # avy=data.angular_velocity.y
    # avz=data.angular_velocity.z
    global count2
    count2=count2+1
    global time_of_pub_last
    if (time_of_pub-time_of_pub_last)>0.015 :
        print("i miss one",time_of_pub-time_of_pub_last)
    time_of_pub_last=data.header.timestamp_sec
   # fimu.write(str(time_of_pub)+';'+str(lvx)+';'+str(lvy)+';'+str(lvz)+';'+str(avx)+';'+str(avy)+';'+str(avz)+';'+str(count2)+';''\n')

def callbackchassis(data):
    #local_es.CopyFrom(data)
    #print('IMU received')   
    time_of_pub = data.header.timestamp_sec
    v=data.speed_mps #车速 m/s
    rr=data.wheel_speed.wheel_spd_rr #后右，轮速
    rl=data.wheel_speed.wheel_spd_rl #后左，轮速
    fr=data.wheel_speed.wheel_spd_fr #前右，轮速
    fl=data.wheel_speed.wheel_spd_fl #前左，轮速
    fchassis.write(str(time_of_pub)+';'+str(v)+';'+str(rr)+';'+str(rl)+';'+str(fr)+';'+str(fl)+';''\n')

def callbackvelodyne(data):
    #local_es.CopyFrom(data)
    #print('velodyne received')  
    time_of_pub = data.measurement_time
    #point = data.point
    fvelodyne.write(str(time_of_pub)+'\n')
    num = 0
    for i in range(len(data.point)):
        fvelodyne.write(str(data.point[i].x)+';'+str(data.point[i].y)+';'+str(data.point[i].z)+';'+str(data.point[i].x)+';')
        num = num + 1
    width=data.width
    height=data.height
    #y=data.point.y
    #z=data.point.z
    #intensity=data.point.intensity
    #fvelodyne.write(str(time_of_pub)+';'+str(x)+';'+str(y)+';'+str(z)+';'+str(intensity)+';''\n')
    fvelodyne.write('\n'+';'+str(width)+';'+str(height)+';'+'\n')
    fvelo_verify.write(str(time_of_pub)+';'+str(num-width*height)+'\n')

def main(argv):

    """ 

    Main node   

    """

    node = cyber.Node("rtk_recorder")
    argv = FLAGS(argv)



    node.create_reader('/apollo/localization/pose',

                        localization_pb2.LocalizationEstimate,

                        callbacklocalization)

    node.create_reader('/apollo/sensor/gnss/best_pose',

                        gnss_best_pose_pb2.GnssBestPose,

                        callbackbestpose)

    node.create_reader('/apollo/sensor/gnss/heading',

                        heading_pb2.Heading,

                        callbackheading)

    node.create_reader('/apollo/sensor/gnss/imu',

                       imu_pb2.Imu,

                       callbackimu)

    # node.create_reader('/apollo/canbus/chassis',

    #                    chassis_pb2.Chassis,

    #                    callbackchassis)

    # node.create_reader('/apollo/sensor/lidar16/PointCloud2',

    #                    pointcloud_pb2.PointCloud,

    #                    callbackvelodyne)
                       
    print("error_test")


    while not cyber.is_shutdown():

        time.sleep(0.000001)


if __name__ == '__main__':

    cyber.init()

    #f=open('testg.txt','w')
    fimu=open('/apollo/TJZNtools/data/1012/gnssimu.txt','w')
    fbestpose=open('/apollo/TJZNtools/data/1012/gnssbestpose.txt','w')
    fheading=open('/apollo/TJZNtools/data/1012/gnssheading.txt','w')
    fpose=open('/apollo/TJZNtools/data/1012/localizationpose.txt','w')
    #fchassis=open('/apollo/TJZNtools/data/localizationpose.txt','w')
    #fvelodyne=open('/apollo/TJZNtools/data/velodynedata.txt','w') 
    #fvelo_verify=open('/apollo/TJZNtools/data/velo_verify.txt','w')   
    #listener()
    print("error2")
 
    main(sys.argv)

    #f.close()
    fimu.close()
    fbestpose.close()
    fheading.close()
    fpose.close()
    #fchassis.close()
    #fvelodyne.close() 
    #fvelo_verify.close()

    cyber.shutdown()
else :
    print("error")


