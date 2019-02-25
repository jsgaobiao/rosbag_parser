#!/usr/bin/env python

# Convert rosbag(sensor_msgs/PointCloud2 && ros_dwdx) to dsv format
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
import math
import numpy as np
import datetime
import rosbag

from rcs_msg_wrapper.msg import dwdx
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion

ROS_BAG_PATH = "/home/gaobiao/Documents/201-2019/huaishuling_0.bag"
DSV_PATH = "/home/gaobiao/Documents/201-2019/huaishuling_0.dsv"
VELO_TOPIC = "/pandar_points"
NAV_TOPIC = "/ros_dwdx"
VELO_HZ_RATIO = 1
POINT_NUM_PER_BLOCK = 32 * 12
BLOCK_NUM_PER_FRAME = 180 # velo64: 290
SIZE_OF_POINT = 48

def parse_nav_msg(data):
    gx = float(data.global_x) * 0.1
    gy = float(data.global_y) * 0.1
    gz = data.global_h * 0.1
    heading = data.heading * 0.01
    pitch = data.pitch * 0.01
    roll = data.roll * 0.01
    return gy, gx, gz, math.radians(roll), math.radians(pitch), -math.radians(heading)

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
        point3fi = struct.pack("4fi", 0,0,0,0,0)
    else:
        # x
        point3fi =  data.data[ptCnt*SIZE_OF_POINT+0:ptCnt*SIZE_OF_POINT+4]
        # y
        point3fi += data.data[ptCnt*SIZE_OF_POINT+4:ptCnt*SIZE_OF_POINT+8]
        # z
        point3fi += data.data[ptCnt*SIZE_OF_POINT+8:ptCnt*SIZE_OF_POINT+12]
        # intensity
        point3fi += data.data[ptCnt*SIZE_OF_POINT+16:ptCnt*SIZE_OF_POINT+20]
        # intensity = struct.unpack("f", data.data[ptCnt*SIZE_OF_POINT+16:ptCnt*SIZE_OF_POINT+20])

        # timestamp [Pandar40]
        # point3fi += data.data[ptCnt*SIZE_OF_POINT+24:ptCnt*SIZE_OF_POINT+32]
        # ts = struct.unpack("q", data.data[ptCnt*SIZE_OF_POINT+24:ptCnt*SIZE_OF_POINT+32])

        # ring [Pandar40]
        point3fi += data.data[ptCnt*SIZE_OF_POINT+32:ptCnt*SIZE_OF_POINT+36]

    ptCnt += 1
    return point3fi, ptCnt

# main
outf = open(DSV_PATH, "wb")
bag = rosbag.Bag(ROS_BAG_PATH)
navMsg = None

for topic, msg, ts in bag.read_messages(topics = [NAV_TOPIC, VELO_TOPIC]):
    # nav/Odometry messages
    if topic == NAV_TOPIC:
        navMsg = msg
    # sensor_msgs/PointCloud2 message
    if topic == VELO_TOPIC:
        if navMsg:
            gx, gy, gz, roll, pitch, yaw = parse_nav_msg(navMsg)
            millisec = int(get_millisec(ts))
            sNav = struct.pack("6dq", roll, pitch, yaw, gx, gy, gz, millisec)
            print(millisec, roll, pitch, yaw)
            # parse velodyne points
            ptCnt = 0
            for i in range(BLOCK_NUM_PER_FRAME / VELO_HZ_RATIO):
                outf.write(sNav)
                for j in range(POINT_NUM_PER_BLOCK):
                    sPoint3fi, ptCnt = get_next_point(msg, ptCnt)
                    outf.write(sPoint3fi)

outf.close()
bag.close()
