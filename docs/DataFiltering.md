# Data Filtering

## Event Processing Rules

**TODO**: determine the rules with the team

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

we will have access to:

1. `self.msg`
2. `self.prev_msg`