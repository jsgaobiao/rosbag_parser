#!/usr/bin/env python

# Gao Biao 2018.06.22

import rospy
import struct
import sys

from velodyne_msgs.msg import VelodynePacket, VelodyneScan
from sensor_msgs.msg import PointCloud2

# params for HDL64
PTNUM_PER_BLK = 12 * 32
SIZE_OF_POINT = 32
global cnt
global lastMilli
cnt = 0
lastMilli = 0

class VelodyneDecoder:
    def __init__(self, prefix):
        self.lidar_file = None
        self.vdnFileName = prefix.split('.')[0] + ".vdn"
        self.lidar_file = open(self.vdnFileName, "wb")

        listenTopic = '/velodyne_points'
        rospy.Subscriber(listenTopic, PointCloud2, self.callback)

    def callback(self, data):
        global cnt
        global lastMilli
        milli = long(data.header.stamp.secs*1000 + data.header.stamp.nsecs/1e6)  # ms
        # deal with 20hz velodyne_packets
        if (cnt % 2 == 0):
            lastMilli = milli
        else:
            milli = lastMilli
        cnt += 1
        print(milli, data.width)
        if (data.width != 110592):
            rospy.logerr('velodyne data missing')
            return
        j = 0
        while (j < data.width):
            if (j % PTNUM_PER_BLK == 0):
                self.lidar_file.write(struct.pack('q', milli))
            for i in range(PTNUM_PER_BLK):
                point3fi =  data.data[j*SIZE_OF_POINT+0:j*SIZE_OF_POINT+4]
                point3fi += data.data[j*SIZE_OF_POINT+4:j*SIZE_OF_POINT+8]
                point3fi += data.data[j*SIZE_OF_POINT+8:j*SIZE_OF_POINT+12]
                point3fi += data.data[j*SIZE_OF_POINT+16:j*SIZE_OF_POINT+20]
                self.lidar_file.write(point3fi)
                j += 1

    def close(self):
        if self.lidar_file is not None:
            self.lidar_file.close()


def main(args):
    print args
    rospy.init_node('velodyne_pointcloud_2_vdn')
    prefix = rospy.get_param("file_prefix")
    print "vdn saved in: %s" % prefix
    vdnDecoder = VelodyneDecoder(prefix)

    rospy.spin()

    vdnDecoder.close()
    rospy.loginfo('closed')


if __name__ == '__main__':
    main(sys.argv)
