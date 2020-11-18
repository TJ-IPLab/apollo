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
    posx=data.pose.position.x
    posy=data.pose.position.y
    posz=data.pose.position.z
    orix=data.pose.orientation.qx
    oriy=data.pose.orientation.qy
    oriz=data.pose.orientation.qz
    oriw=data.pose.orientation.qw
    fpose.write(str(time_of_pub)+';'+str(posx)+';'+str(posy)+';'+str(posz)+';'+str(orix)+';'+str(oriy)+';'+str(oriz)+';'+str(oriw)+';''\n')

def callbackbestpose(data):
    #local_es.CopyFrom(data) 
    #print('bestpose received')  
    time_of_pub = data.header.timestamp_sec
    sol_status=sol_status
    sol_type=sol_type
    lati=data.latitude #纬度，单位度
    longi=data.longitude #经度，单位度
    height=data.height_msl+data.undulation #高度，单位m
    lati_RMS=data.latitude_std_dev #纬度RMS单位m
    longi_RMS=data.longitude_std_dev #经度RMS单位m
    height_RMS=data.height_std_dev #高度RMS单位m
    num_sats_in_solution=data.num_sats_in_solution #解算卫星个数
    fpose.write(str(time_of_pub)+';'+str(sol_status)+';'+str(sol_type)+';'+str(lati)+';'+str(longi)+';'+str(height)+';'+str(lati_RMS)+';'+str(longi_RMS)+';'+str(height_RMS)+';'+str(num_sats_in_solution)+';''\n')



def main(argv): #主节点，主思想，是创建一个cyber的节点，去创建reader来读取其中的消息，传递进来List列表

    """ 

    Main node   

    """

    node = cyber.Node("rtk_recorder")  #创建一个cyber的节点，在Node节点中进行消息订阅和发布，这个节点的名称可以自己定义
                                       #在Node节点中可以创建Reader订阅消息，也可以创建Writer发布消息，每个Node节点中可以创建多个Reader和Writer。
    argv = FLAGS(argv)


    #node.create_reader('/apollo/canbus/chassis',

                      # chassis_pb2.Chassis,

                      # chassis_callback)   #一个消息，也就是ros中的一个话题，对应一个channel,只能对应一个writer或者reader
                                                    #创建一个reader，来接收消息，订阅消息，调用了chassis_callback函数

                                            ########不断create_reader就可以接收#####

    node.create_reader('/apollo/localization/pose',

                       localization_pb2.LocalizationEstimate,

                       callbacklocalization)  #同理，所以想接收几个话题的消息，就需要有几个reader,同时因为里面使用了recorder，它是record_file
                                                      #所以想保存几个文件，就设计几个record_file就可以了

 #node.create_reader('/apollo/localization/pose',  #改成IMU

                      # localization_pb2.LocalizationEstimate,

                      # callbackimu)
                

 node.create_reader('/apollo/sensor/gnss/best_pose',  #改成BESTpos

                       gnss_best_pose_pb2.GnssBestPose,

                       callbackbestpose)

 #node.create_reader('/apollo/localization/pose',  #heading

                      # localization_pb2.LocalizationEstimate,

                     #  callbackheading)


    while not cyber.is_shutdown():

        time.sleep(0.002)


if __name__ == '__main__':  #所以如果直接执行的话，那所有的函数是在这里按顺序执行的，而非函数部分和main函数是从程序的开始到结尾按顺序执行的

    cyber.init()

    #f=open('testg.txt','w') #zhujiaqi
    fimu=open('/apollo/tongjiZNS/tools/ssh/data/gnssimu.txt','w')
    fbestpose=open('/apollo/tongjiZNS/tools/ssh/data/gnssbestpose.txt','w')
    fheading=open('/apollo/tongjiZNS/tools/ssh/data/gnssheading.txt','w')
    fpose=open('/apollo/tongjiZNS/tools/ssh/data/localizationpose.txt','w')
    #listener()
 
    main(sys.argv)  #sys.argv其实可以看作是一个列表，所以才能用[]提取其中的元素。其第一个元素是程序本身，随后才依次是外部给予的参数。

     #f.close()  #zhujiaqi
    fimu.close()
    fbestpose.close()
    fheading.close()
    fpose.close() 

    cyber.shutdown()

    #每个python模块（python文件，也就是此处的 test.py 和 import_test.py）都包含内置的变量 __name__，
    #当该模块被直接执行的时候，__name__ 等于文件名（包含后缀 .py ）；如果该模块 import 到其他模块中，则该模块的 __name__ 等于模块名称（不包含后缀.py）。

    #而 “__main__” 始终指当前执行模块的名称（包含后缀.py）。进而当模块被直接执行时，__name__ == 'main' 结果为真。

