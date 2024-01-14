# ROS Noetic Module

Viam supports the ROS Noetic module using the rospy and viam python libraries.
This module supports selective data capture as well as dynamic tagging, for more 
information on data filtering see [DataFiltering.md](./docs/DataFiltering.md).

## requirements

ROS noetic environment with python3.8

## install
This module can be installed through the [Viam module registry](https://docs.viam.com/registry/)

## install locally

clone the repo to your local system, edit the viam config using the raw json:
```  "modules": [
    {
      "name": "ros-noetic",
      "executable_path": "/home/viam/ros-noetic/run.sh",
      "type": "local",
      "env": {
        "OVERLAYS": "/home/viam/viam_ws/devel/setup.bash",
        "CACHE_DIR": "/opt/viam-cache",
        "CACHE_WINDOW": "20",
        "ROS_MASTER": "http://localhost:11311",
        "LD_PRELOAD": "/usr/lib/aarch64-linux-gnu/libgomp.so.1",
        "ROS_ENV": "/opt/ros/noetic/setup.bash"
      }
    }
  ],
```

to build the ros `logger` service, we need to generate the code from the prot file, run:
```shell
make build_services
```

## configure
The Viam module can be configured with a specific environment to ensure the module starts properly.
the ros module provides support for:
1. ROS_ENV: this is the ROS environment script
2. LD_PRELOAD: this is required for some of the python packages we are using
3. OVERLAYS: these are the custom overlays for our ROS robot (seperated by colon)
4. CACHE_DIR: by default the VIAM_MODULE_DATA directory will be used unless we override the CACHE_DIR variable. this directory needs to exist.
5. CACHE_WINDOW: when using filtering we capture data before and after the event to ensure we can get a full picture of what is occuring
6. ROS_MASTER: the ros master we need to communicate with

WHen configuring the module, the larger the window the more space that will be used.

## add components

the [ros_sensor](./components/ros_sensor.py) is the only component that uses filtering and selective data capture.

the json configuration: 
```json
{
  "type": "sensor",
  "model": "viam-soleng:noetic:sensor",
  "attributes": {
    "events": [
      {
        "eval_stop": "self.msg['event_id'] == 2",
        "name": "simple_event",
        "eval_start": "self.msg['event_id'] == 1"
      }
    ],
    "ros_topic": "/event",
    "ros_msg_package": "viam_event_data.msg",
    "ros_msg_type": "Event"
  },
  "depends_on": [],
  "name": "event"
}
```

every sensor will be associated to a ros topic using the `ros_topic` attribute, the `ros_msg_package` attribute defines 
what package the message can be found in and finally the `ros_msg_type` defines the data type that the ros topic is 
associated with. sensors can have one or more events associated with it. an event is something interesting we want to
track, for example imu acceleration is too fast, or the robot performs a hard stop, etc.

to evaluate if an event has started or stopped our ros_sensor uses the `eval()` function.

every event will have:
1. name: this name will be added to the data so we can search the data by event
2. eval_start: this is statement that python will evaluate, if it is true data will be captured until the event stops
3. eval_stop: this is the statement that python will evaluate, if it is true data will stop being captured

All data captured will include the data before and after the event as defined by the cache window. This will allow us to 
not only capture the interesting event but what led up to the event.

Every sensor that have data capture turned on will also be captured, ensuring we can see the entire picture of what was
occurring on the robot when the event(s) occurred.

if multiple events occur, all events will be associated with the data.

## learning types
The ros_sensor uses introspection to convert the ros data type to json and capture all the data, the `build_msg()` 
function makes use `rospy` and the attributes that are common to all messages:
1. __slots__: this will determine the fields to look at
2. __getattribute__ or getattr: this will get the attribute based on the slots
3. types: this will determine the type of the item we are looking at

```python
import std_msgs.msg as sm
r = sm.Range()
for i in sm.Range.__slots__:
    print(type(getattr(r, i)))
```
