#!/usr/bin/env python

import rospy
import struct
import sys
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion
from dwdx.msg import dwdx


class NavDecoder:
    def __init__(self, prefix):
        self.nav_file = None
        self.nav_path = prefix.split('.')[0] + ".nav"
        self.nav_file = open(self.nav_path, "w")

        # # garage_honglin_round1
        self.origin_x = 432156.101671
        self.origin_y = 2782892.03132
        self.origin_z = 149.103001548

        # garage_honglin_round2
        # self.origin_x = 432082.96876
        # self.origin_y = 2782647.71575
        # self.origin_z = 149.147030

        self.nav_origin_topic = '/navsat/origin'
        self.nav_topic = '/navsat/odom'

        # self.nav_origin_sub = rospy.Subscriber(self.nav_origin_topic, Pose, self._nav_origin_callback)
        # self.nav_sub = None
        self.nav_sub = rospy.Subscriber(self.nav_topic, Odometry, self._callback)

    def _nav_origin_callback(self, origin_pose):

        self.origin_x = origin_pose.position.x
        self.origin_y = origin_pose.position.y
        self.origin_z = origin_pose.position.z

        print ('nav origin: ', self.origin_x, self.origin_y, self.origin_z)

        # sub once
        self.nav_origin_sub.unregister()

        # start sub nav frame
        self.nav_sub = rospy.Subscriber(self.nav_topic, Odometry, self._callback)

    def _callback(self, data):
        time = data.header.stamp.secs * 1000000 + data.header.stamp.nsecs / 1E3  # ms
        self.nav_file.write("%d\t" % time)
        quater = [data.pose.pose.orientation.x,
                  data.pose.pose.orientation.y,
                  data.pose.pose.orientation.z,
                  data.pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quater)
        self.nav_file.write(str(euler[0]) + '\t' +
                            str(euler[1]) + '\t' +
                            str(euler[2]) + '\t')
        self.nav_file.write(str(self.origin_x + data.pose.pose.position.x) + '\t' +
                            str(self.origin_y + data.pose.pose.position.y) + '\t' +
                            str(self.origin_z + data.pose.pose.position.z) + '\t')
        self.nav_file.write("0 0 0 105\n")

    def close(self):
        if self.nav_file is not None:
            self.nav_file.close()


def main(args):
    print args
    rospy.init_node('gps_saver')
    prefix = rospy.get_param("file_prefix")
    print "gps saved in: %s" % prefix
    nav_decoder = NavDecoder(prefix)

    rospy.spin()

    nav_decoder.close()
    rospy.loginfo('closed')


if __name__ == '__main__':
    main(sys.argv)
