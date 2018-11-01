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

ROS_BAG_PATH = "/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/LiDAR_CAM_INS_2017-12-12-15-08-33_1.bag"
NAV_PATH = "/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/1.nav"
NAV_TOPIC = "/navsat/odom"

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

# main
outf = open(NAV_PATH, "w")
bag = rosbag.Bag(ROS_BAG_PATH)

for topic, msg, ts in bag.read_messages(topics = [NAV_TOPIC]):
    # nav/Odometry messages
    if topic == NAV_TOPIC:
        navMsg = msg
        gx, gy, gz, roll, pitch, yaw = parse_nav_msg(navMsg)
        millisec = int(get_millisec(ts))
        print(millisec, roll, pitch, yaw)
        outf.write("%d\t%f\t%f\t%f\t%f\t%f\t%f\t0\n" % (millisec, roll, pitch, yaw, gx, gy, gz))

outf.close()
bag.close()
