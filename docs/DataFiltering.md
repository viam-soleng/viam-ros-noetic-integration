# Data Filtering

the ROS graph can generate a lot of data, using Viam we can selectively sync only the required data to the cloud.
the [ros_sensor](../components/ros_sensor.py) will create a file based cache if the component is configured for 
[data capture](https://docs.viam.com/data/capture/).

as ros messages are generated on the graph they will be stored in the file cache, when data capture asks for data 
from the sensor, data is removed from the queue, if there is an active event and the data is within the cache window
it is returned. If these requirements are not met then no data is returned to the data capture service.

using the cache we are able to capture the data before the event starts as well as during and after the event. the
`cache_window` variable determines how much data before and after the event is stored.

## Event Processing

only the [ros_sensory.py](../components/ros_sensor.py) will do event processing, the config will be:

```json
{
  "events": [
    {
      "name": "<NAME_OF_EVENT>",
      "eval_start": "python eval",
      "eval_stop": "python eval"
    }
  ]
}
```
because `eval()` is being used, we will have access to all the data in the object, the most interesting elements are:
1. `self.msg`: this is the current message
2. `self.prev_msg`: the is the previous message

we can create rules like:

```json
{
  "name": "sudden_spike",
  "eval_start": "abs(self.prev_msg.x - self.msg.x) >= 100",
  "eval_stop": "abs(self.prev_msg.x - self.msg.x) >= 5 and self.msg.x <= 20"
}
```
was there a change of 100 or more in value x, if so start processes and tag all data that has capture turned on with 
the event name, the event will continue until the eval_stop is true. in this case x needs to be back to a normal value
and the change from the previous message should be no more than 5 units difference.
