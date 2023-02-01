#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    ros::init (argc, argv, "img_publisher");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("image", 10);
    cv::Mat image;
    //Input video
    int camera_device_id_, rate_;
    nh.param("camera_device_id", camera_device_id_, 0);
    nh.param("rate", rate_, 10);
    cout << "camera_device_id:" << camera_device_id_ << " rate:" << rate_ << endl;
    ros::Rate looprate (rate_);

    VideoCapture cap;
    bool ok = cap.open(camera_device_id_);
    Mat frame;
    while(ros::ok()) {
        cap>>frame;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}