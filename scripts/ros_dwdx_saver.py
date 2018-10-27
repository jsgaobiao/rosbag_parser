#!/usr/bin/env python

import rospy
import struct
import sys
import tf
import re
import numpy as np
import datetime

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion
from rcs_msg_wrapper.msg import dwdx


class DwdxDecoder:
    def __init__(self, prefix):
        self.dwdx_file = None
        self.nav_path = prefix.split('.')[0] + ".dwdx"
        self.dwdx_file = open(self.nav_path, "w")

        # # garage_honglin_round1
        # self.origin_x = 432156.101671
        # self.origin_y = 2782892.03132
        # self.origin_z = 149.103001548

        # garage_honglin_round2
        # self.origin_x = 432082.96876
        # self.origin_y = 2782647.71575
        # self.origin_z = 149.147030

        # self.nav_origin_topic = '/ros_dwdx'
        self.nav_topic = '/ros_dwdx'

        # self.nav_origin_sub = rospy.Subscriber(self.nav_origin_topic, Pose, self._dwdx_origin_callback)
        # self.nav_sub = None
        self.nav_sub = rospy.Subscriber(self.nav_topic, dwdx, self._callback)

    # def _dwdx_origin_callback(self, origin_pose):
    #
    #     self.origin_x = origin_pose.position.x
    #     self.origin_y = origin_pose.position.y
    #     self.origin_z = origin_pose.position.z
    #     print ('nav origin: ', self.origin_x, self.origin_y, self.origin_z)
    #     # sub once
    #     self.nav_origin_sub.unregister()
    #     # start sub nav frame
    #     self.nav_sub = rospy.Subscriber(self.nav_topic, Odometry, self._callback)

    def _callback(self, data):
        global cnt
        global last_time, last_time_of_day, last_unix_time
        unix_time = data.header.stamp.secs + data.header.stamp.nsecs / 1E9  # ms
        time_of_day = datetime.datetime.fromtimestamp(unix_time).strftime("%Y-%m-%d %H:%M:%S.%f")
        time_of_day_s = time_of_day.split(' ')[1].split('.')
        (H,M,S) = time_of_day_s[0].split(':')
        MS = time_of_day_s[1]
        time = float(H) * 3600000 + float(M) * 60000 + float(S) * 1000 + float(MS) / 1e3
        if (time < last_time):
            print("ERROR ! %.0f  %.0f" % (last_time, time), last_time_of_day, time_of_day, last_unix_time, unix_time)
        last_time = time
        last_time_of_day = time_of_day
        last_unix_time = unix_time

        gx = float(data.global_x) * 0.1
        gy = float(data.global_y) * 0.1
        gh = data.global_h * 0.1
        longi = float(data.longitude) * 0.000001
        lati  = float(data.latitude) * 0.000001
        heading = data.heading * 0.01
        pitch = data.pitch * 0.01
        roll = data.roll * 0.01
        zone = data.zone
        mileage = data.mileage * 0.1

        if (mileage != 0):
            self.dwdx_file.write("%.0f\t" % time)
            self.dwdx_file.write(str(gx) + '\t' +
                                 str(gy) + '\t' +
                                 str(gh) + '\t')
            self.dwdx_file.write(str(longi) + '\t' +
                                str(lati) + '\t')
            self.dwdx_file.write(str(roll) + '\t' +
                                str(pitch) + '\t' +
                                str(heading) + '\t')
            self.dwdx_file.write(str(mileage) + '\t' +
                                str(zone) + '\n')
        else:
            cnt += 1
            rospy.loginfo("error dwdx data %d\n", cnt)

    def close(self):
        if self.dwdx_file is not None:
            self.dwdx_file.close()


def main(args):
    global cnt
    global last_time, last_time_of_day, last_unix_time
    print args
    cnt = 0
    last_time = 0
    last_unix_time = 0
    rospy.init_node('ros_dwdx_saver')
    prefix = "ros_dwdx.txt" #rospy.get_param("file_prefix")
    print "ros_dwdx saved in: %s" % prefix
    nav_decoder = DwdxDecoder(prefix)

    rospy.spin()

    nav_decoder.close()
    rospy.loginfo('closed')


if __name__ == '__main__':
    main(sys.argv)
