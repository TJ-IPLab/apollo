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



from cyber_py import cyber

from gflags import FLAGS



from common.logger import Logger   #利用from imort就可以使用其他py文件中的函数，从common.logger.py中的Logger()函数。

from modules.canbus.proto import chassis_pb2  

from modules.localization.proto import localization_pb2  

from modules.drivers.gnss.proto import imu_pb2  #zhujiqi

from modules.drivers.gnss.proto import gnss_best_pose_pb2 #zhujiaqi

from modules.drivers.gnss.proto import heading_pb2  #zhujiaqi

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
    fbestpose.write(str(time_of_pub)+';'+str(sol_status)+';'+str(sol_type)+';'+str(lati)+';'+str(longi)+';'+str(height)+';'+str(lati_RMS)+';'+str(longi_RMS)+';'+str(height_RMS)+';'+str(num_sats_in_solution)+';''\n')

def callbackheading(data):
    #local_es.CopyFrom(data)
    #print('heading received')
    time_of_pub = data.header.timestamp_sec
    he=data.heading
    he_RMS=data.heading_std_dev
    fheading.write(str(time_of_pub)+';'+str(he)+';'+str(he_RMS)+';''\n')

def callbackimu(data):
    #local_es.CopyFrom(data) 
    #print('bestpose received')
    time_of_pub=data.header.timestamp_sec
    lvx=data.linear_acceleration.x #前左上 m/s^2，是原始的消息，经过drivers发出来，就是右前上的车身坐标系了，所以可直接用
    lvy=data.linear_acceleration.y
    lvz=data.linear_acceleration.z
    avx=data.angular_velocity.x #前左上，rad/s是原始的消息，经过drivers发出来，就是右前上的车身坐标系了，所以可直接用
    avy=data.angular_velocity.y
    avz=data.angular_velocity.z
    fimu.write(str(time_of_pub)+';'+str(lvx)+';'+str(lvy)+';'+str(lvz)+';'+str(avx)+';'+str(avy)+';'+str(avz)+';''\n')
    
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

    node.create_reader('/apollo/canbus/chassis',

                       chassis_pb2.Chassis,

                       callbackchassis)
                       



    while not cyber.is_shutdown():

        time.sleep(0.002)


if __name__ == '__main__':

    cyber.init()

    #f=open('testg.txt','w')
    fimu=open('/apollo/gnssimu.txt','w')
    fbestpose=open('/apollo/gnssbestpose.txt','w')
    fheading=open('/apollo/gnssheading.txt','w')
    fpose=open('/apollo/localizationpose.txt','w')
    fchassis=open('/apollo/chassis.txt','w')    
    #listener()
 
    main(sys.argv)

    #f.close()
    fimu.close()
    fbestpose.close()
    fheading.close()
    fpose.close()
    fchassis.close() 

    cyber.shutdown()


