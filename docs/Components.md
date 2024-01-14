# ROS Noetic Components

Viam components can map to various ROS messages by making use of both the [viam-sdk](https://python.viam.dev/) and
[rospy](http://wiki.ros.org/rospy) apis. We will use the rospy api to access ROS messages, and we will use the viam 
sdk to convert these message to component data (in the form of python dictionaries) which will allow for integration
into the viam platform.

## Base
the [ros base](../components/ros_base.py) component supports publishing `Twist` message to move the robot

### Configuration
```json
{
  "attributes": {
    "publish_time": "0.2",
    "ros_topic": "/viamrosone/cmd_vel"
  },
  "depends_on": [],
  "name": "rosbase",
  "type": "base",
  "model": "viamlabs:ros2:base"
}
```

## Cameras

In viam a [camera component](https://docs.viam.com/components/camera/) will take data from the camera and store the 
image and access time in a protobuf message that can be used to send data to the platform.

the `ros_camera` supports the `ROSIamge` and `CompressedImage`. if we set the compressed flag we will attempt to 
use compressed images, if the topic does not support the type an error will be raised.

This is valuable when used in conjunction with the viam web/mobile based apis for viewing our robot cameras in 
real-time over the internet. The Viam api's make use of both webrtc & dynamic dns to provide secure real-time 
view of data on the robot.

These images can also be stored in our data platform to train ML models that not only can our robot use, but
can also be deployed to other robots in our fleet.

**Please note**, a viam camera component is not a ROS image message, which means that certain ROS headers will 
be lost, if we require the image to be used in a ROS bag we can make the camera a [sensor](#sensor). Please note
that if we are using the image as a sensor, we **must** use the compressed image.

#### configuration

```json
{
  "type": "camera",
  "model": "viam-soleng:noetic:camera",
  "attributes": {
    "ros_topic": "/camera/image_raw/compressed",
    "compressed": "true"
  },
  "depends_on": [],
  "name": "ros_camera"
}
```

## IMU (Movement Sensors)

In viam a [movement sensor](https://docs.viam.com/components/movement-sensor/) represents an IMU, GPS, gyroscope, etc.
Depending on the device and values returned certain fields will be populated while others will be discarded. 

At this time we have only integrated the [rospy imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) 
message type.

### configuration

```json
{
  "model": "viamlabs:ros2:imu",
  "attributes": {
    "ros_topic": "/viamrosone/imu"
  },
  "depends_on": [],
  "name": "imu",
  "type": "movement_sensor"
}
```

## Lidar

In viam a lidar is considered a [camera component](https://docs.viam.com/components/camera/) which returns a point
cloud instead of an image.

The noetic lidar component will convert the [ros laserscan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)
message into a [pcd formatted structure](https://pointclouds.org/documentation/tutorials/pcd_file_format.html) which 
can be consumed by used by the viam [SLAM service](https://docs.viam.com/mobility/slam/).

This provides an option to quickly configure a SLAM service using lidar that has only ROS drivers.

### configuration

```json
{
  "name": "lidar",
  "type": "camera",
  "model": "viamlabs:ros2:lidar",
  "attributes": {
    "ros_topic": "/viamrosone/scan"
  },
  "depends_on": []
}
```

## Sensor

In viam the most general component is the [sensor](https://docs.viam.com/components/sensor/) which can be used for all
ros topics that we subscribe to.

The [ros_sensor.py](../components/ros_sensor.py) supports the ability to read any message that the ROS environment is
configured for, these messages are then converted to a python dictionary that can be stored in the viam cloud
[data management platform](https://docs.viam.com/data/).

Using this platform we can programmatically access this data for analysis, charting, etc.

**NOTE**: soon we will be offering tooling to convert json messages back into ROS binary format as well as bundling 
into rosbag files for replay and troubleshooting.

### configuration   

```json
{ 
      "depends_on": [],
      "name": "usb_camera",
      "type": "sensor",
      "model": "viam-soleng:noetic:sensor",
      "service_configs": [
        {
          "attributes": {
            "capture_methods": [
              {
                "capture_frequency_hz": 1,
                "method": "Readings",
                "additional_params": {},
                "disabled": true
              }
            ]
          },
          "type": "data_manager"
        }
      ],
      "attributes": {
        "ros_topic": "/camera/image_raw/compressed",
        "ros_msg_package": "sensor_msgs.msg",
        "ros_msg_type": "CompressedImage",
        "events": [
          {"name":  "EVENT_NAME", "eval_start":  "PYTHON CODE HERE", "eval_stop":  "PYTHON_CODE_HERE"}
        ]
      }
    }
```