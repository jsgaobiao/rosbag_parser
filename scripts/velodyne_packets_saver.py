#!/usr/bin/env python

import rospy
import struct
import sys

from velodyne_msgs.msg import VelodynePacket, VelodyneScan

class VelodyneDecoder:
    def __init__(self, prefix):
        self.lidar_file = None
        self.velodyne_packets_path = prefix + ".hdl"
        self.lidar_file = open(self.velodyne_packets_path, "w")

        topic = '/velodyne_top_driver/velodyne_packets'
        rospy.Subscriber(topic, VelodyneScan, self.callback)

    def callback(self, data):
        for packet in data.packets:
            if len(data.packets) == 288:
                time = packet.stamp.secs*1000 + packet.stamp.nsecs/1e6  # ms
                time_bytes = struct.pack('L', time)
                self.lidar_file.write(time_bytes)
                self.lidar_file.write(packet.data)
            else:
                rospy.logerr('data missing')

    def close(self):
        if self.lidar_file is not None:
            self.lidar_file.close()


def main(args):
    print args
    rospy.init_node('velodyne_packets_saver')
    prefix = rospy.get_param("file_prefix")
    print "hdl saved in: %s" % prefix
    hdl_decoder = VelodyneDecoder(prefix)

    rospy.spin()

    hdl_decoder.close()
    rospy.loginfo('closed')


if __name__ == '__main__':
    main(sys.argv)
