#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


def gstreamer_pipeline(mode, compression):
    if compression:
        return (
            f"thetauvcsrc mode={mode} ! queue ! decodebin ! autovideoconvert ! video/x-raw,format=YUY2 ! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink"
        )
    else:
        return (
            f"thetauvcsrc mode={mode} ! queue ! decodebin ! autovideoconvert ! video/x-raw,format=xBGR ! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink"
        )


def start_node():
    rospy.init_node("theta_camera")
    rospy.loginfo("theta camera started")

    # rosparam から mode を取得し、デフォルト値を "4K" に設定
    mode = rospy.get_param("~mode", "4K")
    use_compression = rospy.get_param("~use_compression", True)

    # mode が "2K" または "4K" でない場合はエラーを出力し、デフォルト値 "4K" を使用
    if mode not in ["2K", "4K"]:
        rospy.logerr(f"Invalid mode '{mode}'. Using default mode '4K'.")
        mode = "4K"

    src = None
    pub_theta_camera = None
    if use_compression:
        src = gstreamer_pipeline(mode, True)
        pub_theta_camera = rospy.Publisher("~image/compressed", CompressedImage, queue_size=2)
    else:
        src = gstreamer_pipeline(mode, False)
        pub_theta_camera = rospy.Publisher("~image", Image, queue_size=2)
    rospy.loginfo(f"Using pipeline: {src}")

    cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
    
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        try:
            if use_compression:
                timestamp = rospy.Time.now()
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = timestamp
                compressed_msg.format = "jpeg"
                compressed_msg.data = np.array(
                    cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]).tobytes()
                pub_theta_camera.publish(compressed_msg)
            else:
                pub_theta_camera.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")
            pass

    cap.release()


if __name__ == "__main__":
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass