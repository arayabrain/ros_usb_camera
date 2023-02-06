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
rosrun usb_camera main.py
```

## Input/Output topic
### Input

None

### Output
- image [Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)


## Parameters
- device_id(int): Device id of the camera. Default value is 0.
- width(int): Width of the image. Default value is 640.
- height(int): Height of the image. Default value is 480.
- framerate(int): Publish rate of the image. Default value is 15.
