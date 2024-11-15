#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Header

class ImageTimestampNode:
    def __init__(self):
        rospy.init_node('timestamp_node', anonymous=True)
        
        # パラメータの設定（必要に応じて調整してください）
        self.input_image_topic = rospy.get_param('~input_image_topic', '/camera/image_raw')
        self.input_compressed_image_topic = rospy.get_param('~input_compressed_image_topic', '/camera/compressed')
        self.output_image_topic = rospy.get_param('~output_image_topic', '/camera/image_timestamped')
        self.compression_format = rospy.get_param('~compression_format', 'jpeg')
        self.compression_quality = rospy.get_param('~compression_quality', 80)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.input_image_topic, Image, self.image_callback)
        self.image_sub = rospy.Subscriber(
            self.input_compressed_image_topic, 
            CompressedImage, 
            self.compressed_image_callback
        )
        self.image_pub = rospy.Publisher(self.output_image_topic, Image, queue_size=10)

    def compressed_image_callback(self, compressed_msg):
        try:
            # CompressedImageをnumpy arrayに変換
            np_arr = np.frombuffer(compressed_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            
            # ヘッダーを設定
            img_msg.header = Header()
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = compressed_msg.header.frame_id
            # パブリッシュ
            self.image_pub.publish(img_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def image_callback(self, msg):
        try:
            # 現在のタイムスタンプを取得
            timestamp = rospy.Time.now()
            
            # 新しいタイムスタンプを設定
            msg.header.stamp = timestamp
            
            # 新しいメッセージをパブリッシュ
            self.image_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ImageTimestampNode()
        rospy.loginfo(f"Subscribing to {node.input_image_topic}")
        rospy.loginfo(f"Subscribing to {node.input_compressed_image_topic}")
        rospy.loginfo(f"Publishing to {node.output_image_topic}")

        node.run()
    except rospy.ROSInterruptException:
        pass