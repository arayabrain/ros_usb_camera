#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

def start_node():
    rospy.init_node('usb_camera')
    rospy.loginfo('usb camera started')

    if not rospy.has_param("~device_id"):
        rospy.set_param("~device_id", 0)
    device_id = rospy.get_param("~device_id")
    if not rospy.has_param("~width"):
        rospy.set_param("~width", 640)
    width = rospy.get_param("~width")
    if not rospy.has_param("~height"):
        rospy.set_param("~height", 480)
    height = rospy.get_param("~height")

    if not rospy.has_param("~framerate"):
        rospy.set_param("~framerate", 15)
    framerate = rospy.get_param("~framerate")
    pub = rospy.Publisher('~image', Image, queue_size=10)

    src = 'v4l2src device=/dev/video' + str(device_id) + ' ! image/jpeg,width=' + str(width) + ', height=' + str(height) + ', framerate=(fraction)' + str(framerate) + '/1 !jpegdec !videoconvert !appsink sync=0'
    print(src)
    cap=cv2.VideoCapture(src)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame_read = cap.read()
        try:
            pub.publish(bridge.cv2_to_imgmsg(frame_read, "bgr8"))
        except Exception as e:
            pass

    cap.release()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass