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

from modules.transform.proto import transform_pb2 #wangguanbei



def callbackheading(data):
    #local_es.CopyFrom(data)
    #print('heading received')
    time_of_pub = data.header.timestamp_sec
    # print(data)
    he=data.heading
    he_RMS=data.heading_std_dev
    time_measurement=data.measurement_time  #测量时间
    fheading.write(str(time_of_pub)+';'+str(time_measurement)+';'+str(he)+';'+str(he_RMS)+';''\n')

def callbacktransform(data):
    tf0=data.transforms[0]
    print(tf0.header.frame_id)
    print(tf0.child_frame_id)
    tf0=data.transforms[1]
    print(tf0.header.frame_id)
    print(tf0.child_frame_id)
    tf0=data.transforms[2]
    print(tf0.header.frame_id)
    print(tf0.child_frame_id)
    tf0=data.transforms[3]
    print(tf0.header.frame_id)
    print(tf0.child_frame_id)
    tf0=data.transforms[4]
    print(tf0.header.frame_id)
    print(tf0.child_frame_id)

def main(argv):

    """ 

    Main node   

    """

    node = cyber.Node("rtk_recorde")
    argv = FLAGS(argv)



    node.create_reader('/tf_static',

                       transform_pb2.TransformStampeds,

                       callbacktransform)





    while not cyber.is_shutdown():

        time.sleep(0.002)


if __name__ == '__main__':

    cyber.init()
 
    main(sys.argv)

    cyber.shutdown()


