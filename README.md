# ROS Noetic Module

Viam supports the ROS Noetic module using the rospy and viam python libraries.

## install

Clone to local machine, install and configure viam, use viam-config.json as example to start.

## TODO
- create as module 
- add other sensors as needed
- document
- VIAM_MODULE_DATA -> can we override just in case

## learning types

with `rospy` we need to use:
1. __slots__: this will determine the fields to look at
2. __getattribute__ or getattr: this will get the attribute based on the slots
3. types: this will determine the type of the item we are looking at

This solution will be used to convert the msgs to json format, then we will need to convert 
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
      "executable_path": "/home/viam/ros-noetic/run.sh",
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