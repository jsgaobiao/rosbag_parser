#!/usr/bin/env python

# Filter the video in rosbag

import rospy
import struct
import sys
import tf
import cv2
import numpy as np
import datetime
import rosbag

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion

ROS_BAG_PATH = "/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/LiDAR_CAM_INS_2017-12-12-15-13-15_2.bag"
VIDEO_PATH = "/media/gaobiao/SeagateBackupPlusDrive/201/201-2018/data/guilin/hongling_Round1/2.avi"
VIDEO_TOPIC = "/camera/image_color/compressed"
VIDEO_HZ_RATIO = 10

def get_millisec(stamp):
    unix_time = stamp.secs + stamp.nsecs / 1E9
    time_of_day = datetime.datetime.fromtimestamp(unix_time).strftime("%Y-%m-%d %H:%M:%S.%f")
    time_of_day_s = time_of_day.split(' ')[1].split('.')
    (H,M,S) = time_of_day_s[0].split(':')
    MS = time_of_day_s[1]
    time = float(H) * 3600000 + float(M) * 60000 + float(S) * 1000 + float(MS) / 1e3
    return time

# main
bag = rosbag.Bag(ROS_BAG_PATH)
outf = open(VIDEO_PATH + '.ts', 'w')
isFirstFrame = True

for topic, msg, ts in bag.read_messages(topics = [VIDEO_TOPIC]):
    if topic == VIDEO_TOPIC:
        millisec = int(get_millisec(ts))
        outf.write("%d\n" % millisec)
        print(millisec)
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if isFirstFrame:
            isFirstFrame = False
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            height = np.shape(image_np)[0]
            width = np.shape(image_np)[1]
            #open file for writing
            videowriter = cv2.VideoWriter(VIDEO_PATH, fourcc, VIDEO_HZ_RATIO, (width, height))

        cv2.putText(image_np, str(millisec), (20,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 4)
        videowriter.write(image_np)

videowriter.release()
outf.close()
bag.close()
