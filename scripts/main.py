#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
print(cv2.getBuildInformation())
from cv_bridge import CvBridge, CvBridgeError


def start_node():
    rospy.init_node("usb_camera")
    rospy.loginfo("usb camera started")

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

    if not rospy.has_param("~omni_camera"):
        rospy.set_param("~omni_camera", False)
    omni_camera = rospy.get_param("~omni_camera")

    pub_front_camera = None
    pub_back_camera = None
    if omni_camera:
        pub_front_camera = rospy.Publisher("~front_image", Image, queue_size=10)
        pub_back_camera = rospy.Publisher("~back_image", Image, queue_size=10)
    else:
        pub_front_camera = rospy.Publisher("~image", Image, queue_size=10)

    src = (
        "v4l2src device=/dev/video"
        + str(device_id)
        + " ! image/jpeg,width="
        + str(width)
        + ", height="
        + str(height)
        + ", framerate=(fraction)"
        + str(framerate)
        + "/1 !jpegdec !videoconvert !appsink sync=0"
    )
    # src = "videotestsrc ! videoconvert ! appsink"
    print(src)
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print("GStreamerパイプラインを開くことができませんでした")
        return
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame_read = cap.read()
        try:
            if omni_camera:
                upper_frame = frame_read[0 : int(frame_read.shape[0] / 2)]
                lower_frame = frame_read[int(frame_read.shape[0] / 2) :]
                pub_front_camera.publish(bridge.cv2_to_imgmsg(lower_frame, "bgr8"))
                pub_back_camera.publish(bridge.cv2_to_imgmsg(upper_frame, "bgr8"))
            else:
                pub_front_camera.publish(bridge.cv2_to_imgmsg(frame_read, "bgr8"))
        except Exception as e:
            # print(e)
            pass

    cap.release()


if __name__ == "__main__":
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
