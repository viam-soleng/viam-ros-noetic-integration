# viam workspace

This is a simple noetic ros workspace to send fake events to a queue to
test component level filtering.

## building

```shell
cd <PATH-TO-VIAM-WS>/viam_ws
catkin_make
```

## running

For our testing, we are using a simple usb camera this workspace contains two launch files 
which can launch just the event topic which will generate a simple event or start the 
usb camera and event node.

### event only
```shell
source <PATH-TO-VIAM-WS>/viam_ws/devel/setup.bash
roslaunch viam_event_data event_setup.launch
```

### usb camera & event
```shell
source <PATH-TO-VIAM-WS>/viam_ws/devel/setup.bash
roslaunch viam_event_data usb_camera_and_event.launch
```

## Generate events

All events are just number here so we can test the eval function in the viam sensor

```shell
source <PATH-TO-VIAM-WS>/viam_ws/devel/setup.bash

# turn on event
rostopic pub -1 /event viam_event_data/Event 1 1.0
rostopic pub -1 /event viam_event_data/Event 3 1.0
rostopic pub -1 /event viam_event_data/Event 5 1.0

# turn off event
rostopic pub -1 /event viam_event_data/Event 2 1.0
rostopic pub -1 /event viam_event_data/Event 4 1.0
rostopic pub -1 /event viam_event_data/Event 6 1.0

```