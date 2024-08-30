#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def gstreamer_pipeline(mode):
    return (
        f"thetauvcsrc mode={mode} ! queue ! decodebin ! autovideoconvert ! video/x-raw,format=BGRx ! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink"
    )


def start_node():
    rospy.init_node("theta_camera")
    rospy.loginfo("theta camera started")

    # rosparam から mode を取得し、デフォルト値を "4K" に設定
    mode = rospy.get_param("~mode", "4K")

    # mode が "2K" または "4K" でない場合はエラーを出力し、デフォルト値 "4K" を使用
    if mode not in ["2K", "4K"]:
        rospy.logerr(f"Invalid mode '{mode}'. Using default mode '4K'.")
        mode = "4K"

    src = gstreamer_pipeline(mode)
    rospy.loginfo(f"Using pipeline: {src}")

    cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
    pub_theta_camera = rospy.Publisher("~image", Image, queue_size=100)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        try:
            pub_theta_camera.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception as e:
            # rospy.logerr(f"Error publishing image: {e}")
            pass

    cap.release()


if __name__ == "__main__":
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass