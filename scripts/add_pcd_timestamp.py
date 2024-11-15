#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2


class PointCloudTimestampNode:
    def __init__(self):
        rospy.init_node('pointcloud_timestamp_node', anonymous=True)
        
        # パラメータの設定（必要に応じて調整してください）
        self.input_topic = rospy.get_param('~input_pcd_topic', '/pointcloud')
        self.output_topic = rospy.get_param('~output_pcd_topic', '/pointcloud_timestamped')
        
        self.pcd_sub = rospy.Subscriber(self.input_topic, PointCloud2, self.pointcloud_callback)
        self.pcd_pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=10)
        
        rospy.loginfo(f"Subscribing to {self.input_topic}")
        rospy.loginfo(f"Publishing to {self.output_topic}")

    def pointcloud_callback(self, msg):
        try:
            # 現在のタイムスタンプを取得
            timestamp = rospy.Time.now()
            
            # 新しいタイムスタンプを設定
            msg.header.stamp = timestamp
            
            # 新しいメッセージをパブリッシュ
            self.pcd_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PointCloudTimestampNode()
        node.run()
    except rospy.ROSInterruptException:
        pass