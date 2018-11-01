#!/usr/bin/env python

# Convert rosbag(sensor_msgs/PointCloud2 && Odometry) to dsv format
# Usage: 1. rosbag_saver_velopackets_2_pointcloud.launch (convert velodyne_packets to PointCloud2)
#        2. run this script

# DSV format definiton:
#   point3d         ang
#   point3d         shv (x, y, z)
#   long long       millisec
#   point3fi        points[POINT_NUM_PER_BLOCK]     // POINT_NUM_PER_BLOCK = 32*12 default    point3fi(float*3, u_char)
#   MATRIX          rot                             // double[3][3]

import rospy
import struct
import sys
import tf
import numpy as np
import datetime
import rosbag

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion

ROS_BAG_PATH = "/media/gaobiao/GaoBiaoBigDisk/GuiLin201712/hongling/Round1/intermediate_result/2_pointcloud_and_nav.bag"
DSV_PATH = "/media/gaobiao/GaoBiaoBigDisk/GuiLin201712/hongling/Round1/intermediate_result/hongling_round1_2.dsv"
VELO_TOPIC = "/velodyne_points"
NAV_TOPIC = "/navsat/odom"
VELO_HZ_RATIO = 2    # guilin data's velodyne is 20HZ, half circle data in 1 frame, so we need to combine 2 frames data into one.
POINT_NUM_PER_BLOCK = 32 * 12
BLOCK_NUM_PER_FRAME = int(580 / 2)
SIZE_OF_POINT = 32

def parse_nav_msg(navMsg):
    gx = navMsg.pose.pose.position.x
    gy = navMsg.pose.pose.position.y
    gz = navMsg.pose.pose.position.z
    euler = tf.transformations.euler_from_quaternion((navMsg.pose.pose.orientation.x, \
                                                      navMsg.pose.pose.orientation.y, \
                                                      navMsg.pose.pose.orientation.z, \
                                                      navMsg.pose.pose.orientation.w))
    return gx, gy, gz, euler[0], euler[1], euler[2]

def get_millisec(stamp):
    unix_time = stamp.secs + stamp.nsecs / 1E9
    time_of_day = datetime.datetime.fromtimestamp(unix_time).strftime("%Y-%m-%d %H:%M:%S.%f")
    time_of_day_s = time_of_day.split(' ')[1].split('.')
    (H,M,S) = time_of_day_s[0].split(':')
    MS = time_of_day_s[1]
    time = float(H) * 3600000 + float(M) * 60000 + float(S) * 1000 + float(MS) / 1e3
    return time

def get_next_point(data, ptCnt):
    if ptCnt >= data.width:
        point3fi = struct.pack("4f", 0,0,0,0)
    else:
        point3fi =  data.data[ptCnt*SIZE_OF_POINT+0:ptCnt*SIZE_OF_POINT+4]
        point3fi += data.data[ptCnt*SIZE_OF_POINT+4:ptCnt*SIZE_OF_POINT+8]
        point3fi += data.data[ptCnt*SIZE_OF_POINT+8:ptCnt*SIZE_OF_POINT+12]
        point3fi += data.data[ptCnt*SIZE_OF_POINT+16:ptCnt*SIZE_OF_POINT+20]
    ptCnt += 1
    return point3fi, ptCnt

# main
outf = open(DSV_PATH, "wb")
bag = rosbag.Bag(ROS_BAG_PATH)

for topic, msg, ts in bag.read_messages(topics = [NAV_TOPIC, VELO_TOPIC]):
    # nav/Odometry messages
    if topic == NAV_TOPIC:
        navMsg = msg
        nav_millisec = int(get_millisec(ts))
    # sensor_msgs/PointCloud2 message
    if topic == VELO_TOPIC:
        gx, gy, gz, roll, pitch, yaw = parse_nav_msg(navMsg)
        millisec = int(get_millisec(ts))
        sNav = struct.pack("6dq", roll, pitch, yaw, gx, gy, gz, millisec)
        print(millisec, nav_millisec, roll, pitch, yaw)
        # parse velodyne points
        ptCnt = 0
        for i in range(BLOCK_NUM_PER_FRAME / VELO_HZ_RATIO):
            outf.write(sNav)
            for j in range(POINT_NUM_PER_BLOCK):
                sPoint3fi, ptCnt = get_next_point(msg, ptCnt)
                outf.write(sPoint3fi)

outf.close()
bag.close()
