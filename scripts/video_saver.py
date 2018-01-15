#!/usr/bin/env python
import sys

import numpy as np
import cv2

from sensor_msgs.msg import CompressedImage
import rospy

VERBOSE = False


class ImageDecoder:
    def __init__(self, prefix):
        self.subscriber = rospy.Subscriber("/camera/image_color/compressed", CompressedImage, self.callback, queue_size=1)
        self.firstframe = True
        self.videowriter = None
        self.tswriter = None

        self.videofilename = prefix + ".avi"
        self.timestampfilename = prefix + ".ts"

    def callback(self, ros_data):
        if VERBOSE:
            print 'received image of type: "%s"' % ros_data.format
        #### direct conversion to CV2 ####
        stamp = ros_data.header.stamp.secs * 1000 + ros_data.header.stamp.nsecs / 1e6

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.firstframe:
            self.firstframe = False
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            height = np.shape(image_np)[0]
            width = np.shape(image_np)[1]
            #open file for writing
            self.videowriter = cv2.VideoWriter(self.videofilename, fourcc, 20.0, (width, height))
            self.tswriter = open(self.timestampfilename, "w")

        self.videowriter.write(image_np)
        stamp = "%d\n" % stamp
        self.tswriter.writelines(stamp)

    def close(self):
        if self.tswriter is not None:
            self.tswriter.close()

        if self.videowriter is not None:
            self.videowriter.release()


def main(args):
    print args
    rospy.init_node("video_saver")

    prefix = rospy.get_param("file_prefix")

    print "video saved in: %s" % prefix

    saver = ImageDecoder(prefix)

    rospy.spin()

    saver.close()

    rospy.loginfo('closed')


if __name__ == '__main__':
    main(sys.argv)