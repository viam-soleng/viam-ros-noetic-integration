# ROS Noetic Module

Viam supports the ROS Noetic module using the rospy and viam python libraries.

## install

Clone to local machine, install and configure viam, use viam-config.json as example to start.

## TODO
- update config to provide support for all messages
- create as module 
- add other sensors as needed
- document
- move to environment variables for module

## messages

This is a simple integration providing sensor support for:
1. [can_msgs/Frame](http://docs.ros.org/en/noetic/api/can_msgs/html/msg/Frame.html)
2. [sensor_msgs/BatteryState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/BatteryState.html)
3. [sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html)
4. [sensor_msgs/CompressedImages](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html)
5. [sensor_msgs/FluidPressure](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/FluidPressure.html)
6. [sensor_msgs/Illuminance](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Illuminance.html)]
7. [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
8. [sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)
9. [sensor_msgs/Joy](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Joy.html)
10. [sensor_msgs/LaserEcho](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserEcho.html)
11. [sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)
12. [sensor_msgs/MagneticField](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html)
13. [sensor_msgs/NavSatFix](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html)
14. [sensor_msgs/NavSatStatus](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatStatus.html)
15. [sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
16. [sensor_msgs/PointField](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointField.html)
17. [sensor_msgs/Range](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Range.html)
18. [sensor_msgs/Temperature](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Temperature.html)
19. [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html)
20. [geometry_msgs/Pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html)
21. [geometry_msgs/PoseWithCovariance](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html)
22. [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
23. [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
24. [geometry_msgs/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)
25. [geometry_msgs/TwistWithCovariance](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html)
26. [geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)
27. [geometry_msgs/Quaternion](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html)
28. [geometry_msgs/QuaternionStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/QuaternionStamped.html)
29. [geometry_msgs/Transform](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Transform.html)
30. [geometry_msgs/TransformStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TransformStamped.html)
31. [geometry_msgs/Vector3](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html)
32. [geometry_msgs/Vector3Stamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3Stamped.html)
33. [geometry_msgs/Wrench](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Wrench.html)
34. [geometry_msgs/WrenchStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/WrenchStamped.html)
35. [std_msgs](http://docs.ros.org/en/noetic/api/std_msgs/html/index-msg.html)
36. [std_msgs/Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)
37. [nav_msgs/GridCells](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/GridCells.html)
38. [nav_msgs/MapMetaData](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/MapMetaData.html)
39. [nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
40. [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
41. [nav_msgs/Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
42. [stereo_msgs/DisparityImage](http://docs.ros.org/en/noetic/api/stereo_msgs/html/msg/DisparityImage.html)


## learning types

with `rospy` we need to use:
1. __slots__: this will determine the fields to look at
2. __getattribute__ or getattr: this will get the attribute based on the slots
3. types: this will determine the type of the item we are looking at

This solution will be used to convert the msgs to json format and then we will need to convert 
back to ros msg types, below is a simple example of just how to get some info:

```python
import std_msgs.msg as sm
r = sm.Range()
for i in sm.Range.__slots__:
    print(type(getattr(r, i)))
```

We will use this to create 1 sensor component that will dynamically load the required message type, store
it as json. 

### convert json to ros

TODO: will need to test binary conversion and validate for simulation

### Module info
```json
{
  "modules": [
    {
      "name": "ros-noetic",
      "executable_path": "/home/viam/nuport/run.sh",
      "type": "local",
      "env": {
        "ROS_MASTER": "http://localhost:11311",
        "LD_PRELOAD": "/usr/lib/aarch64-linux-gnu/libgomp.so.1",
        "ROS_ENV": "/opt/ros/noetic/setup.bash",
        "OVERLAYS": ""
      }
    }
  ]
}
```