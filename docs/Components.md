# ROS Noetic Components

## Cameras

#### configuration

## IMU (Movement Sensors)

### configuration

## Lidar

### configuration

## Sensors

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
        "ros_msg_type": "CompressedImage"
      }
    }
```