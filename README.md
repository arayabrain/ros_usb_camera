# ros_usb_camera

## Installation
### Cloning this repository
```
cd ~/catkin_ws/src
git clone https://github.com/arayabrain/ros_usb_camera.git
```
### Installation of dependencies
```
```

## Running
```
rosrun usb_camera usb_camera
```

## Input/Output topic
### Input

None

### Output
- image [Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)


## Parameters
- camera_device_id(int): Device id of the camera. Default value is 0.
- rate(int): Publish rate of the image. Default value is 10.
