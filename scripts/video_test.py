#!/usr/bin/env python3
# coding=utf-8

from __future__ import absolute_import, print_function

import sys
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
import argparse
from sensor_msgs.msg import Image, CompressedImage


print(sys.argv)
parser = argparse.ArgumentParser()
parser.add_argument('--image-topic', type=str, required=True, help='topic name for receive image')
parser.add_argument('args', nargs=argparse.REMAINDER)
parser.parse_args()

args = parser.parse_args()

IMG_TOPIC = args.image_topic


# ============================================ROS
def cb_image(img):
    # store img to global scope
    if"compressed" in IMG_TOPIC:
        img = bridge.compressed_imgmsg_to_cv2(img, "passthrough")
    else:
        img = bridge.imgmsg_to_cv2(img, "passthrough")

    cv2.imshow('video', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('video-test')
    print('Init......')

    bridge = CvBridge()

    if"compressed" in IMG_TOPIC:
        img_subscriber = rospy.Subscriber(IMG_TOPIC, CompressedImage, cb_image)
    else:
        img_subscriber = rospy.Subscriber(IMG_TOPIC, Image, cb_image)


    rospy.spin()
