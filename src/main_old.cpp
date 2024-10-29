#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    ros::init (argc, argv, "img_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher compressed_image_pub = nh.advertise<sensor_msgs::CompressedImage>("image", 10);
    cv::Mat image;
    //Input video
    int camera_device_id_, rate_;
    nh.param("camera_device_id", camera_device_id_, 0);
    nh.param("rate", rate_, 10);
    cout << "camera_device_id:" << camera_device_id_ << " rate:" << rate_ << endl;
    ros::Rate looprate (rate_);

    VideoCapture cap;
    bool ok = cap.open(camera_device_id_);

// 圧縮パラメータの設定
    nh.setParam("/compressed_img_publisher/image/compressed/format", "jpeg");
    nh.setParam("/compressed_img_publisher/image/compressed/jpeg_quality", 80);

    Mat frame;
    std::vector<uchar> compressed_image;
    while(ros::ok()) {
        cap>>frame;
        // JPEGに圧縮
        if (frame.empty()) {
            ROS_WARN("Captured frame is empty");
            continue;
        }
        cv::imencode(".jpg", frame, compressed_image);

        sensor_msgs::CompressedImage compressed_msg;
        compressed_msg.header.stamp = ros::Time::now();
        compressed_msg.format = "jpeg";
        compressed_msg.data = compressed_image;
        compressed_image_pub.publish(compressed_msg);
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}